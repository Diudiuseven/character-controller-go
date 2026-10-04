package character

import "math"

// Controller is a fixed-step character movement controller.
// Create with NewController, then call Step exactly once per fixed tick.
type Controller struct {
	cfg   Config
	state State
}

// NewController validates cfg and returns a controller with default state.
// All validation errors name the offending parameter.
func NewController(cfg Config) (*Controller, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	return &Controller{cfg: cfg, state: DefaultState()}, nil
}

// Config returns the controller's configuration.
func (c *Controller) Config() Config { return c.cfg }

// State returns a copy of the current simulation state.
func (c *Controller) State() State { return c.state }

// SetState restores a previously serialized state (checkpoint / replay).
func (c *Controller) SetState(s State) { c.state = s }

// Step advances the simulation by exactly one fixed step. The same inputs
// on the same world always produce the same state, regardless of frame rate.
// Call world.Step(dt) before this so platform motion covers the current tick.
func (c *Controller) Step(world *World, in Input) {
	dt := c.cfg.FixedDt
	s := &c.state
	s.StepCount++

	// --- 1. timers ---------------------------------------------------------
	if s.Ground == Grounded {
		s.CoyoteTimer = c.cfg.CoyoteTime
	} else {
		s.CoyoteTimer -= dt
	}
	s.JumpBufferTimer -= dt
	if in.JumpPressed {
		s.JumpBufferTimer = c.cfg.JumpBufferTime
	}

	// --- 2. carry by moving platform (before physics, no fling) ------------
	if s.Ground == Grounded && s.PlatformID >= 0 {
		if p := world.PlatformByID(s.PlatformID); p != nil {
			s.Pos = s.Pos.Add(p.CarryDelta(s.Pos))
		} else {
			// Platform vanished: become airborne and preserve momentum;
			// never add or remove velocity in a single step.
			s.PlatformID = -1
			s.Ground = Airborne
		}
	}

	// --- 3. horizontal movement model --------------------------------------
	// When grounded on a platform, velocity is controlled relative to the
	// platform; the carry step already accounts for platform motion.
	onPlatform := s.Ground == Grounded && s.PlatformID >= 0
	platformVx := 0.0
	if onPlatform {
		if p := world.PlatformByID(s.PlatformID); p != nil {
			platformVx = p.VelAt(s.Pos).X
		}
	}
	if s.Ground == Grounded || s.Ground == Airborne {
		if onPlatform {
			// Match the platform, plus input-relative target.
			target := clampFloat(in.MoveX, -1, 1) * c.cfg.MaxMoveSpeed
			relVx := s.Vel.X - platformVx
			relVx += (target - relVx) * min(1, c.cfg.GroundAccel*dt)
			s.Vel.X = relVx + platformVx
		} else if in.MoveX != 0 {
			// Only accelerate when a direction is held; airborne momentum
			// with no input is preserved (no air braking).
			target := clampFloat(in.MoveX, -1, 1) * c.cfg.MaxMoveSpeed
			rate := c.cfg.GroundAccel
			if s.Ground != Grounded {
				rate = c.cfg.AirAccel
			}
			s.Vel.X += (target - s.Vel.X) * min(1, rate*dt)
		}
		if s.Ground == Grounded && in.MoveX == 0 && !onPlatform {
			s.Vel.X -= s.Vel.X * min(1, c.cfg.GroundFriction*dt)
		}
	}
	// Aerodynamic drag applies in every state.
	s.Vel.X -= s.Vel.X * min(1, c.cfg.AirDrag*dt)

	// --- 4. jump (buffered, with coyote time) ------------------------------
	if s.JumpBufferTimer > 0 && (s.Ground == Grounded || s.CoyoteTimer > 0) {
		s.Vel.Y = c.cfg.JumpSpeed
		s.Ground = Airborne
		s.CoyoteTimer = 0
		s.JumpBufferTimer = 0
		s.PlatformID = -1
	}

	// --- 5. gravity (airborne only; slope gravity handled on contact) ------
	if s.Ground == Airborne {
		s.Vel.Y -= c.cfg.Gravity * dt
	}

	// --- 6. integrate with collision ---------------------------------------
	// Horizontal motion on a carried platform is relative to the platform;
	// the carry step already added the platform's own displacement.
	dx := s.Vel.X * dt
	if onPlatform {
		dx = (s.Vel.X - platformVx) * dt
	}
	prevPos := s.Pos
	c.resolveInitialOverlaps(world)
	c.moveX(world, dx)
	c.moveY(world, s.Vel.Y*dt)

	// --- 7. contact state update -------------------------------------------
	c.updateContact(world, prevPos)

	// --- 8. terminal fall speed --------------------------------------------
	if s.Vel.Y < -c.cfg.MaxFallSpeed {
		s.Vel.Y = -c.cfg.MaxFallSpeed
	}
}

// resolveInitialOverlaps pushes the character out of any box it overlaps at
// the start of a step (checkpoints, platform carry into walls).
func (c *Controller) resolveInitialOverlaps(world *World) {
	s := &c.state
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	for iter := 0; iter < 4; iter++ {
		pushed := false
		for _, b := range world.BoxColliders() {
			if charOverlapsBox(s.Pos, hw, h, b) {
				s.Pos = s.Pos.Add(depenetrate(s.Pos, hw, h, b))
				pushed = true
			}
		}
		if !pushed {
			return
		}
	}
}

// moveX sweeps horizontally against boxes, with automatic step-up logic.
func (c *Controller) moveX(world *World, dx float64) {
	s := &c.state
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	remaining := dx
	for iter := 0; iter < 4 && absFloat(remaining) > 1e-12; iter++ {
		var earliest boxHit
		found := false
		for _, b := range world.BoxColliders() {
			hit, ok := sweepBox(s.Pos, hw, h, Vec2{X: remaining}, b)
			if !ok {
				continue
			}
			if !found || hit.t < earliest.t {
				earliest = hit
				found = true
			}
		}
		if !found {
			s.Pos.X += remaining
			return
		}
		if earliest.t == 0 {
			s.Pos = s.Pos.Add(depenetrate(s.Pos, hw, h, earliest.box))
			continue
		}
		stepH := earliest.box.Max.Y - s.Pos.Y
		if s.Ground == Grounded && earliest.normal.X != 0 && stepH > 0 && stepH <= c.cfg.MaxStepHeight+c.cfg.SkinWidth {
			if c.tryStep(world, remaining, earliest.box.Max.Y) {
				return
			}
		}
		// Solid wall: stop at contact, keep vertical velocity (wall slide).
		s.Pos.X += remaining * earliest.t
		s.Vel.X = 0
		return
	}
}

// tryStep attempts the lift-forward-place maneuver onto surfaceY. It returns
// false (without mutating state) if the maneuver is blocked or has no support.
func (c *Controller) tryStep(world *World, dx float64, surfaceY float64) bool {
	s := &c.state
	stepH := surfaceY - s.Pos.Y
	if stepH <= 0 || stepH > c.cfg.MaxStepHeight+c.cfg.SkinWidth {
		return false
	}

	// 1. Lift: nothing may block the upward path and head must be clear.
	if c.boxBlocked(s.Pos, Vec2{0, stepH}, world) {
		return false
	}
	lifted := s.Pos.Add(Vec2{0, stepH})
	if c.anyOverlap(lifted, world) {
		return false
	}

	// 2. Forward at the lifted height.
	if c.boxBlocked(lifted, Vec2{X: dx}, world) {
		return false
	}
	target := lifted.Add(Vec2{X: dx})
	if c.anyOverlap(target, world) {
		return false
	}

	// 3. Place down: must find support within stepH + skin.
	support, ok := c.sweepSupport(target, stepH+c.cfg.SkinWidth*2, world)
	if !ok {
		return false
	}
	if support < target.Y-(stepH+c.cfg.SkinWidth*2) {
		return false
	}

	s.Pos = Vec2{X: target.X, Y: support}
	s.Vel.Y = 0
	return true
}

// boxBlocked reports whether sweeping the character from pos by delta hits a box.
func (c *Controller) boxBlocked(pos Vec2, delta Vec2, world *World) bool {
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	for _, b := range world.BoxColliders() {
		if _, ok := sweepBox(pos, hw, h, delta, b); ok {
			return true
		}
	}
	return false
}

// anyOverlap reports whether the character at pos overlaps any box.
func (c *Controller) anyOverlap(pos Vec2, world *World) bool {
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	for _, b := range world.BoxColliders() {
		if charOverlapsBox(pos, hw, h, b) {
			return true
		}
	}
	return false
}

// sweepSupport sweeps the character down by dist and returns the highest
// support surface found (box top or ramp).
func (c *Controller) sweepSupport(pos Vec2, dist float64, world *World) (float64, bool) {
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	skin := c.cfg.SkinWidth
	delta := Vec2{0, -dist}

	bestY := math.Inf(-1)
	found := false
	for _, b := range world.BoxColliders() {
		hit, ok := sweepBox(pos, hw, h, delta, b)
		if !ok {
			continue
		}
		surface := pos.Y - dist*hit.t
		if hit.normal.Y > 0 && surface > bestY {
			bestY = surface
			found = true
		}
	}
	if cross := rampCrossing(pos, pos.Add(delta), hw, world.RampColliders(), skin); cross.found {
		if cross.surfaceY > bestY {
			bestY = cross.surfaceY
			found = true
		}
	}
	return bestY, found
}

// moveY sweeps vertically against boxes; ramps are resolved in updateContact.
func (c *Controller) moveY(world *World, dy float64) {
	s := &c.state
	hw, h := c.cfg.CharacterHalfWidth, c.cfg.CharacterHeight
	delta := Vec2{0, dy}
	for iter := 0; iter < 3; iter++ {
		var earliest boxHit
		found := false
		for _, b := range world.BoxColliders() {
			hit, ok := sweepBox(s.Pos, hw, h, delta, b)
			if !ok {
				continue
			}
			if !found || hit.t < earliest.t {
				earliest = hit
				found = true
			}
		}
		if !found {
			s.Pos.Y += dy
			return
		}
		if earliest.t == 0 {
			s.Pos = s.Pos.Add(depenetrate(s.Pos, hw, h, earliest.box))
			continue
		}
		s.Pos.Y += dy * earliest.t
		s.Vel.Y = 0
		return
	}
}

// updateContact determines the contact state after movement, snaps to the
// surface, projects velocity onto the surface tangent, and applies slope
// gravity for sliding on over-steep slopes.
func (c *Controller) updateContact(world *World, prevPos Vec2) {
	s := &c.state
	hw := c.cfg.CharacterHalfWidth
	skin := c.cfg.SkinWidth
	probe := skin * 2

	contact := probeGround(s.Pos, hw, world.BoxColliders(), world.RampColliders(), probe, skin, c.cfg.MaxStepHeight, s.Ground == Grounded)
	cross := rampCrossing(prevPos, s.Pos, hw, world.RampColliders(), skin)
	if cross.found && (!contact.found ||
		absFloat(cross.surfaceY-s.Pos.Y) < absFloat(contact.surfaceY-s.Pos.Y)) {
		contact = cross
	}

	if !contact.found {
		s.Ground = Airborne
		s.PlatformID = -1
		s.GroundNormal = Vec2{X: 0, Y: 1}
		s.SlopeAngleDeg = 0
		return
	}

	s.GroundNormal = contact.normal
	s.SlopeAngleDeg = slopeAngleDeg(contact.normal)
	s.PlatformID = contact.platform
	s.Pos.Y = contact.surfaceY

	n := contact.normal
	// Project velocity onto the surface tangent (removes into-surface motion).
	s.Vel = s.Vel.Sub(n.Scale(s.Vel.Dot(n)))

	// Hysteresis band around the slope threshold prevents contact-state
	// flapping when the computed angle hovers near the boundary. Hysteresis
	// only applies when already in contact; a fresh landing commits to the
	// raw threshold.
	const slopeHysteresis = 0.5 // degrees
	upper := c.cfg.MaxSlopeAngleDeg + slopeHysteresis
	lower := c.cfg.MaxSlopeAngleDeg - slopeHysteresis
	wasContacting := s.Ground == Grounded || s.Ground == Sliding
	if wasContacting {
		if s.SlopeAngleDeg > upper {
			s.Ground = Sliding
		} else if s.SlopeAngleDeg <= lower {
			s.Ground = Grounded
		}
		// In the band the previous contact state is kept.
	} else {
		// Fresh contact: commit to the walkable threshold with a small
		// epsilon so geometrically exact slopes are not misclassified by
		// floating-point error.
		const slopeEpsilon = 0.1 // degrees
		if s.SlopeAngleDeg <= c.cfg.MaxSlopeAngleDeg+slopeEpsilon {
			s.Ground = Grounded
		} else {
			s.Ground = Sliding
		}
	}
	if s.Ground == Sliding {
		t := n.Perp()
		if t.Y > 0 {
			t = t.Neg()
		}
		s.Vel = s.Vel.Add(t.Scale(c.cfg.Gravity * absFloat(n.X) * c.cfg.FixedDt))
	}
}

func clampFloat(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
