package character

import "math"

// boxHit is the result of a swept AABB vs AABB test.
type boxHit struct {
	t      float64 // 0..1 along the sweep; 0 means initial overlap
	normal Vec2
	box    *BoxCollider
}

// sweepBox tests the character box (feet at pos, half-width hw, height h)
// moving by delta against a single box. Returns the earliest hit.
func sweepBox(pos Vec2, hw, h float64, delta Vec2, b *BoxCollider) (boxHit, bool) {
	cMin := Vec2{pos.X - hw, pos.Y}
	cMax := Vec2{pos.X + hw, pos.Y + h}

	// Initial overlap -> depenetration hit at t=0.
	if cMin.X < b.Max.X && cMax.X > b.Min.X &&
		cMin.Y < b.Max.Y && cMax.Y > b.Min.Y {
		return boxHit{t: 0, normal: Vec2{}, box: b}, true
	}

	var tEnterX, tEnterY float64 = -math.MaxFloat64, -math.MaxFloat64
	var tExitX, tExitY float64 = math.MaxFloat64, math.MaxFloat64
	hasX, hasY := true, true

	if delta.X == 0 {
		if cMax.X <= b.Min.X || cMin.X >= b.Max.X {
			hasX = false
		}
	} else {
		t1 := (b.Min.X - cMax.X) / delta.X
		t2 := (b.Max.X - cMin.X) / delta.X
		tEnterX = math.Min(t1, t2)
		tExitX = math.Max(t1, t2)
	}
	if delta.Y == 0 {
		if cMax.Y <= b.Min.Y || cMin.Y >= b.Max.Y {
			hasY = false
		}
	} else {
		t1 := (b.Min.Y - cMax.Y) / delta.Y
		t2 := (b.Max.Y - cMin.Y) / delta.Y
		tEnterY = math.Min(t1, t2)
		tExitY = math.Max(t1, t2)
	}

	if !hasX || !hasY {
		return boxHit{}, false
	}

	tEnter := math.Max(tEnterX, tEnterY)
	tExit := math.Min(tExitX, tExitY)
	if tEnter > tExit || tEnter < 0 || tEnter > 1 {
		return boxHit{}, false
	}

	n := Vec2{}
	if tEnterX > tEnterY {
		if delta.X > 0 {
			n = Vec2{-1, 0}
		} else {
			n = Vec2{1, 0}
		}
	} else {
		if delta.Y > 0 {
			n = Vec2{0, -1}
		} else {
			n = Vec2{0, 1}
		}
	}
	return boxHit{t: tEnter, normal: n, box: b}, true
}

// depenetrate returns the smallest push-out vector resolving an initial
// overlap between the character box and box b.
func depenetrate(pos Vec2, hw, h float64, b *BoxCollider) Vec2 {
	cMin := Vec2{pos.X - hw, pos.Y}
	cMax := Vec2{pos.X + hw, pos.Y + h}
	overX := math.Min(cMax.X-b.Min.X, b.Max.X-cMin.X)
	overY := math.Min(cMax.Y-b.Min.Y, b.Max.Y-cMin.Y)
	if overX < overY {
		if pos.X < (b.Min.X+b.Max.X)/2 {
			return Vec2{-overX, 0}
		}
		return Vec2{overX, 0}
	}
	if pos.Y < (b.Min.Y+b.Max.Y)/2 {
		return Vec2{0, -overY}
	}
	return Vec2{0, overY}
}

// groundContact describes a detected walkable/contact surface under the character.
type groundContact struct {
	found    bool
	surfaceY float64
	normal   Vec2
	platform int
}

// probeGround finds the nearest contact surface (box top or ramp) at the
// character's feet. probeDist is how far above a surface the feet may be and
// still be considered grounded; skin is the allowed penetration tolerance.
// When wasGrounded is true, a ramp rising into the feet by up to maxStep is
// treated as contact (walking up a slope never falls through).
func probeGround(pos Vec2, hw float64, boxes []*BoxCollider, ramps []*RampCollider, probeDist, skin, maxStep float64, wasGrounded bool) groundContact {
	best := groundContact{platform: -1}
	bestY := -math.MaxFloat64

	for _, b := range boxes {
		// Horizontal overlap with small tolerance for edge cases.
		if pos.X+hw < b.Min.X-skin || pos.X-hw > b.Max.X+skin {
			continue
		}
		top := b.Max.Y
		if pos.Y >= top-skin && pos.Y <= top+probeDist {
			if top > bestY {
				bestY = top
				best = groundContact{found: true, surfaceY: top, normal: Vec2{0, 1}, platform: b.PlatformID}
			}
		}
	}

	for _, r := range ramps {
		info := r.RampAt(pos.X)
		if !info.Found {
			continue
		}
		// Must be within the ramp's horizontal extent (with half-width slack).
		lo := math.Min(r.A.X, r.B.X) - hw - skin
		hi := math.Max(r.A.X, r.B.X) + hw + skin
		if pos.X < lo || pos.X > hi {
			continue
		}
		if pos.Y <= info.SurfaceY+probeDist && pos.Y >= info.SurfaceY-skin {
			if info.SurfaceY > bestY {
				bestY = info.SurfaceY
				best = groundContact{found: true, surfaceY: info.SurfaceY, normal: info.Normal, platform: -1}
			}
			continue
		}
		// Walking onto a rising slope: feet are below the surface by up to
		// maxStep while already grounded -> snap up instead of falling through.
		if wasGrounded && info.SurfaceY > pos.Y && info.SurfaceY-pos.Y <= maxStep {
			if info.SurfaceY > bestY {
				bestY = info.SurfaceY
				best = groundContact{found: true, surfaceY: info.SurfaceY, normal: info.Normal, platform: -1}
			}
		}
	}

	return best
}

// rampCrossing checks whether the feet crossed a ramp surface between the
// previous and current step (prevFeetY >= surface - skin, feet now at or
// below the surface). This prevents tunneling through thin ramps at high speed.
func rampCrossing(prevPos, pos Vec2, hw float64, ramps []*RampCollider, skin float64) groundContact {
	best := groundContact{platform: -1}
	bestDist := math.MaxFloat64

	for _, r := range ramps {
		info := r.RampAt(pos.X)
		if !info.Found {
			continue
		}
		lo := math.Min(r.A.X, r.B.X) - hw - skin
		hi := math.Max(r.A.X, r.B.X) + hw + skin
		if pos.X < lo || pos.X > hi {
			continue
		}
		if prevPos.Y >= info.SurfaceY-skin && pos.Y <= info.SurfaceY+skin {
			d := absFloat(pos.Y - info.SurfaceY)
			if d < bestDist {
				bestDist = d
				best = groundContact{found: true, surfaceY: info.SurfaceY, normal: info.Normal, platform: -1}
			}
		}
	}
	return best
}

// slopeAngleDeg returns the angle in degrees between the surface normal and up.
func slopeAngleDeg(normal Vec2) float64 {
	d := normal.Dot(Vec2{0, 1})
	if d > 1 {
		d = 1
	} else if d < -1 {
		d = -1
	}
	return math.Acos(d) * 180.0 / math.Pi
}
