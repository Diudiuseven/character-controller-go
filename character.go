package controller

import "math"

// Input is the per-step command. Move is in [-1,1]; Jump is an edge-triggered
// press (true only on the step the button went down).
type Input struct {
	Move float64
	Jump bool
}

// CharState is the high-level locomotion state.
type CharState int

const (
	StateAirborne CharState = iota
	StateGrounded
	StateSliding // on a too-steep slope, sliding down
)

func (s CharState) String() string {
	switch s {
	case StateGrounded:
		return "grounded"
	case StateSliding:
		return "sliding"
	default:
		return "airborne"
	}
}

// Character is the controlled body. Pos is the center of the feet (bottom
// middle of the AABB). All unexported fields are part of the deterministic
// state and are included in snapshots.
type Character struct {
	Pos          Vec2
	Vel          Vec2
	State        CharState
	GroundNormal Vec2
	GroundPlatID int // platform id stood on, or -1
	coyoteTimer  float64
	jumpBufTimer float64
	slideLatched bool
}

// NewCharacter creates a character at the given feet position.
func NewCharacter(pos Vec2) *Character {
	return &Character{
		Pos:          pos,
		State:        StateAirborne,
		GroundNormal: V(0, 1),
		GroundPlatID: -1,
	}
}

const skin = 1e-6

// approach moves v toward target by at most maxDelta. It never overshoots,
// so extreme parameters cannot cause direction reversal.
func approach(v, target, maxDelta float64) float64 {
	if v < target {
		return math.Min(v+maxDelta, target)
	}
	return math.Max(v-maxDelta, target)
}

func (c *Character) aabb(cfg *Config) (Vec2, Vec2) {
	hw := cfg.Width / 2
	return V(c.Pos.X-hw, c.Pos.Y), V(c.Pos.X+hw, c.Pos.Y+cfg.Height)
}

func aabbOverlap(mn, mx Vec2, b Box) bool {
	return mn.X < b.Max.X && mx.X > b.Min.X && mn.Y < b.Max.Y && mx.Y > b.Min.Y
}

// eachBox iterates static boxes (platID -1) then platform boxes, in slice
// order, until f returns false.
func (w *World) eachBox(f func(b Box, platID int) bool) {
	for _, b := range w.Boxes {
		if !f(b, -1) {
			return
		}
	}
	for _, p := range w.Platforms {
		if !f(p.Box(), p.ID) {
			return
		}
	}
}

// overlapsAny reports whether the character AABB at pos touches any box.
func (c *Character) overlapsAny(w *World, cfg *Config, pos Vec2) bool {
	hw := cfg.Width / 2
	mn := V(pos.X-hw, pos.Y)
	mx := V(pos.X+hw, pos.Y+cfg.Height)
	hit := false
	w.eachBox(func(b Box, _ int) bool {
		if aabbOverlap(mn, mx, b) {
			hit = true
			return false
		}
		return true
	})
	return hit
}

// depenetrate pushes the character out of any box overlap using the minimal
// translation axis. Used after platform carry moves the character.
func (c *Character) depenetrate(w *World, cfg *Config) {
	for pass := 0; pass < 4; pass++ {
		any := false
		w.eachBox(func(b Box, _ int) bool {
			mn, mx := c.aabb(cfg)
			if !aabbOverlap(mn, mx, b) {
				return true
			}
			any = true
			pushL := mx.X - b.Min.X // move -x
			pushR := b.Max.X - mn.X // move +x
			pushD := mx.Y - b.Min.Y // move -y
			pushU := b.Max.Y - mn.Y // move +y
			px := math.Min(pushL, pushR)
			py := math.Min(pushD, pushU)
			if py <= px {
				if pushU <= pushD {
					c.Pos.Y += pushU
				} else {
					c.Pos.Y -= pushD
				}
			} else {
				if pushR <= pushL {
					c.Pos.X += pushR
				} else {
					c.Pos.X -= pushL
				}
			}
			return true
		})
		if !any {
			return
		}
	}
}

// resolveHorizontal resolves X-axis overlaps after the horizontal move.
// When grounded and blocked by a low box it attempts a step-up.
func (c *Character) resolveHorizontal(w *World, cfg *Config, desiredX float64, grounded bool) {
	for pass := 0; pass < 4; pass++ {
		any := false
		w.eachBox(func(b Box, _ int) bool {
			mn, mx := c.aabb(cfg)
			if !aabbOverlap(mn, mx, b) {
				return true
			}
			any = true
			// Try step-up: low obstacle, grounded, headroom at target.
			if grounded {
				rise := b.Max.Y - c.Pos.Y
				if rise > 0 && rise <= cfg.MaxStepHeight {
					cand := V(desiredX, c.Pos.Y+rise+skin)
					if !c.overlapsAny(w, cfg, cand) {
						c.Pos = cand
						return true
					}
				}
			}
			pushL := mx.X - b.Min.X
			pushR := b.Max.X - mn.X
			if pushL <= pushR {
				c.Pos.X -= pushL
				if c.Vel.X > 0 {
					c.Vel.X = 0
				}
			} else {
				c.Pos.X += pushR
				if c.Vel.X < 0 {
					c.Vel.X = 0
				}
			}
			return true
		})
		if !any {
			return
		}
	}
}

// resolveVertical resolves Y-axis overlaps after the vertical move. It
// reports whether the character landed on a box top and which platform (or
// -1) supports it.
func (c *Character) resolveVertical(w *World, cfg *Config) (bool, int) {
	landed := false
	platID := -1
	for pass := 0; pass < 4; pass++ {
		any := false
		w.eachBox(func(b Box, id int) bool {
			mn, mx := c.aabb(cfg)
			if !aabbOverlap(mn, mx, b) {
				return true
			}
			any = true
			pushU := b.Max.Y - mn.Y
			pushD := mx.Y - b.Min.Y
			if pushU <= pushD {
				c.Pos.Y += pushU
				if c.Vel.Y < 0 {
					c.Vel.Y = 0
				}
				landed = true
				platID = id
			} else {
				c.Pos.Y -= pushD
				if c.Vel.Y > 0 {
					c.Vel.Y = 0
				}
			}
			return true
		})
		if !any {
			break
		}
	}
	return landed, platID
}

// slopeAngleDeg returns the slope's inclination in degrees.
func slopeAngleDeg(s Slope) float64 {
	k := (s.B.Y - s.A.Y) / (s.B.X - s.A.X)
	return math.Atan(math.Abs(k)) * 180 / math.Pi
}

// slideDir returns the unit vector pointing downhill along the slope.
func slideDir(s Slope) Vec2 {
	d := s.B.Sub(s.A).Normalize()
	if d.Y > 0 {
		d = d.Mul(-1)
	}
	return d
}

// bestSlopeAt returns the highest slope surface at x.
func (w *World) bestSlopeAt(x float64) (Slope, float64, bool) {
	var best Slope
	bestH := 0.0
	found := false
	for _, s := range w.Slopes {
		h, ok := s.HeightAt(x)
		if !ok {
			continue
		}
		if !found || h > bestH {
			best, bestH, found = s, h, true
		}
	}
	return best, bestH, found
}

// interactSlopes applies one-sided slope surfaces: landing, walk snap
// up/down, steep-slope wall blocking and slide surface constraint. It
// reports whether a slope currently supports the character.
func (c *Character) interactSlopes(w *World, cfg *Config, prevY, dx float64, grounded bool) bool {
	s, h, ok := w.bestSlopeAt(c.Pos.X)
	if !ok {
		return false
	}
	angle := slopeAngleDeg(s)
	steep := angle > cfg.MaxSlopeAngleDeg+cfg.SlopeHysteresisDeg
	tanMax := math.Tan((cfg.MaxSlopeAngleDeg + cfg.SlopeHysteresisDeg) * math.Pi / 180)
	snapUp := cfg.MaxStepHeight + math.Abs(dx)*tanMax
	snapDown := cfg.GroundSnapDistance + math.Abs(dx)*tanMax
	feet := c.Pos.Y

	// Sliding: constrain to the surface while it lasts.
	if c.State == StateSliding && c.slideLatched {
		if feet <= h+snapDown || feet < h {
			c.Pos.Y = h
			return true
		}
		return false
	}

	// Landing from the air: crossed the surface while moving down.
	if c.Vel.Y <= 0 && prevY >= h-skin && feet < h {
		c.Pos.Y = h
		if steep {
			dir := slideDir(s)
			c.Vel = dir.Mul(c.Vel.Dot(dir))
		} else {
			c.Vel.Y = 0
		}
		return true
	}

	if !grounded {
		return false
	}

	rise := h - feet
	switch {
	case rise >= -skin && rise <= snapUp:
		// Walkable rise: snap up onto the surface (only if not steep).
		if steep {
			c.blockAtSlope(s, feet, cfg)
			return false
		}
		c.Pos.Y = h
		if c.Vel.Y < 0 {
			c.Vel.Y = 0
		}
		return true
	case rise > snapUp && h < feet+cfg.Height:
		// Too tall to climb this step: act as a wall.
		c.blockAtSlope(s, feet+snapUp, cfg)
		return false
	case rise < -skin && -rise <= snapDown:
		// Descending: snap down to stay glued.
		c.Pos.Y = h
		if c.Vel.Y < 0 {
			c.Vel.Y = 0
		}
		return true
	}
	return false
}

// blockAtSlope clamps horizontal motion so the surface at the character
// center does not exceed targetH, and kills inward horizontal velocity.
func (c *Character) blockAtSlope(s Slope, targetH float64, cfg *Config) {
	k := (s.B.Y - s.A.Y) / (s.B.X - s.A.X)
	if k == 0 {
		return
	}
	xLim := s.A.X + (targetH-s.A.Y)/k
	if k > 0 && c.Vel.X > 0 && c.Pos.X > xLim {
		c.Pos.X = math.Max(xLim, s.A.X)
		c.Vel.X = 0
	}
	if k < 0 && c.Vel.X < 0 && c.Pos.X < xLim {
		c.Pos.X = math.Min(xLim, s.B.X)
		c.Vel.X = 0
	}
}

// stepDownProbe looks for a box top just below the feet (walking down
// steps) and snaps to it. Reports whether support was found.
func (c *Character) stepDownProbe(w *World, cfg *Config) (bool, int) {
	reach := math.Max(cfg.GroundSnapDistance, cfg.MaxStepHeight)
	best := c.Pos.Y - reach
	found := false
	platID := -1
	hw := cfg.Width / 2
	w.eachBox(func(b Box, id int) bool {
		if c.Pos.X+hw <= b.Min.X || c.Pos.X-hw >= b.Max.X {
			return true
		}
		top := b.Max.Y
		if top <= c.Pos.Y+skin && top >= best {
			best = top
			found = true
			platID = id
		}
		return true
	})
	if !found {
		return false, -1
	}
	c.Pos.Y = best
	if c.Vel.Y < 0 {
		c.Vel.Y = 0
	}
	return true, platID
}

// Step advances the character by exactly cfg.Dt using the given input.
//
// Phase order (fixed, deterministic):
//  1. platform carry      2. timers        3. horizontal velocity
//  4. gravity             5. jump          6. horizontal move + step-up
//  7. vertical move       8. slopes        9. ground support update
func (c *Character) Step(w *World, cfg *Config, in Input) {
	dt := cfg.Dt
	move := clamp(in.Move, -1, 1)
	if math.IsNaN(move) {
		move = 0
	}

	// 1. Ride the platform we stand on (translation + rotation about its
	// center). A vanished platform simply drops us: no velocity is added.
	if c.State != StateAirborne && c.GroundPlatID >= 0 {
		if p := w.PlatformByID(c.GroundPlatID); p != nil {
			c.Pos = c.Pos.Add(p.CarryDelta(c.Pos))
			c.depenetrate(w, cfg)
		} else {
			c.GroundPlatID = -1
		}
	}

	// 2. Timers.
	if c.State == StateAirborne && c.coyoteTimer > 0 {
		c.coyoteTimer -= dt
		if c.coyoteTimer < 0 {
			c.coyoteTimer = 0
		}
	}
	if in.Jump {
		c.jumpBufTimer = cfg.JumpBufferTime
	} else if c.jumpBufTimer > 0 {
		c.jumpBufTimer -= dt
		if c.jumpBufTimer < 0 {
			c.jumpBufTimer = 0
		}
	}

	// 3. Horizontal velocity. While sliding, slope dynamics own the
	// velocity; otherwise input accelerates toward the run cap and
	// friction/drag decelerate. approach() never overshoots.
	if c.State == StateSliding {
		if s, _, ok := w.bestSlopeAt(c.Pos.X); ok {
			dir := slideDir(s)
			speed := c.Vel.Dot(dir) - cfg.Gravity*dir.Y*dt
			speed = clamp(speed, -cfg.TerminalVelocity, cfg.TerminalVelocity)
			c.Vel = dir.Mul(speed)
		}
	} else if move != 0 {
		accel := cfg.AirAccel
		if c.State == StateGrounded {
			accel = cfg.GroundAccel
		}
		c.Vel.X = approach(c.Vel.X, move*cfg.MaxRunSpeed, accel*dt)
	} else if c.State == StateGrounded {
		c.Vel.X = approach(c.Vel.X, 0, cfg.GroundFriction*dt)
	} else {
		c.Vel.X *= 1 - cfg.AirDrag*dt
	}

	// 4. Gravity with terminal velocity clamp.
	c.Vel.Y -= cfg.Gravity * dt
	if c.Vel.Y < -cfg.TerminalVelocity {
		c.Vel.Y = -cfg.TerminalVelocity
	}

	// 5. Jump: allowed on ground, while sliding, or inside the coyote
	// window; a buffered press fires immediately on landing.
	canJump := c.State != StateAirborne || c.coyoteTimer > 0
	if c.jumpBufTimer > 0 && canJump {
		c.Vel.Y = cfg.JumpSpeed
		c.State = StateAirborne
		c.slideLatched = false
		c.coyoteTimer = 0
		c.jumpBufTimer = 0
		c.GroundPlatID = -1
	}

	grounded := c.State != StateAirborne

	// 6-8. Move with collision. The displacement is split into a fixed
	// number of equal substeps so no substep moves more than half the
	// smallest character dimension: tunneling through thin colliders is
	// impossible regardless of speed. The substep count is a pure
	// function of the current state, keeping the sim deterministic.
	dx := c.Vel.X * dt
	dy := c.Vel.Y * dt
	maxDisp := math.Min(cfg.Width, cfg.Height) * 0.5
	n := 1 + int(math.Max(math.Abs(dx), math.Abs(dy))/maxDisp)
	fn := float64(n)
	sx, sy := dx/fn, dy/fn
	landedBox := false
	landPlat := -1
	onSlope := false
	for i := 0; i < n; i++ {
		// Horizontal move with box resolution and step-up.
		c.Pos.X += sx
		c.resolveHorizontal(w, cfg, c.Pos.X, grounded)

		// Vertical move with box resolution.
		prevY := c.Pos.Y
		c.Pos.Y += sy
		lb, lp := c.resolveVertical(w, cfg)
		if lb {
			landedBox, landPlat = true, lp
		}

		// Slope surfaces.
		if c.interactSlopes(w, cfg, prevY, sx, grounded) {
			onSlope = true
		}
	}

	// 9. Ground support update.
	switch {
	case onSlope:
		s, _, _ := w.bestSlopeAt(c.Pos.X)
		angle := slopeAngleDeg(s)
		switch {
		case angle > cfg.MaxSlopeAngleDeg+cfg.SlopeHysteresisDeg:
			c.State = StateSliding
			c.slideLatched = true
		case c.slideLatched && angle > cfg.MaxSlopeAngleDeg-cfg.SlopeHysteresisDeg:
			c.State = StateSliding
		default:
			c.State = StateGrounded
			c.slideLatched = false
		}
		c.GroundNormal = s.Normal()
		c.GroundPlatID = -1
		c.coyoteTimer = cfg.CoyoteTime
	case landedBox:
		c.State = StateGrounded
		c.slideLatched = false
		c.GroundNormal = V(0, 1)
		c.GroundPlatID = landPlat
		c.coyoteTimer = cfg.CoyoteTime
	case grounded:
		// Were grounded last step: probe for support just below (steps
		// down). Otherwise we walked off a ledge.
		if ok, id := c.stepDownProbe(w, cfg); ok {
			c.State = StateGrounded
			c.slideLatched = false
			c.GroundNormal = V(0, 1)
			c.GroundPlatID = id
			c.coyoteTimer = cfg.CoyoteTime
		} else {
			c.State = StateAirborne
			c.slideLatched = false
			c.GroundPlatID = -1
		}
	default:
		c.State = StateAirborne
		c.slideLatched = false
	}
}
