package controller

// Box is a static, axis-aligned solid collider (walls, floors, ceilings).
type Box struct {
	Min, Max Vec2
}

// Contains reports whether p is strictly inside the box.
func (b Box) Contains(p Vec2) bool {
	return p.X > b.Min.X && p.X < b.Max.X && p.Y > b.Min.Y && p.Y < b.Max.Y
}

// Slope is a one-sided walkable surface from A to B (A.X must be < B.X).
// It only interacts with the character from above, as ground.
type Slope struct {
	A, B Vec2
}

// HeightAt returns the surface height at x, and whether x is within the
// slope's horizontal span.
func (s Slope) HeightAt(x float64) (float64, bool) {
	if x < s.A.X || x > s.B.X {
		return 0, false
	}
	t := (x - s.A.X) / (s.B.X - s.A.X)
	return s.A.Y + t*(s.B.Y-s.A.Y), true
}

// Normal returns the upward-pointing surface normal.
func (s Slope) Normal() Vec2 {
	d := s.B.Sub(s.A)
	n := Vec2{X: -d.Y, Y: d.X}.Normalize()
	if n.Y < 0 {
		n = n.Mul(-1)
	}
	return n
}

// Platform is a kinematic moving carrier. Its collision shape is the
// axis-aligned box centered at Center; Angle/AngularVel only affect how
// standing characters are carried (rigid transform about Center).
type Platform struct {
	ID         int
	Center     Vec2
	Half       Vec2
	Angle      float64
	Vel        Vec2
	AngularVel float64 // radians per second

	lastDelta  Vec2
	lastDAngle float64
}

// Box returns the platform's current collision AABB.
func (p *Platform) Box() Box {
	return Box{Min: p.Center.Sub(p.Half), Max: p.Center.Add(p.Half)}
}

// CarryDelta returns the rigid displacement the platform applied to a point
// at pos during the last Step: rotation about Center, then translation.
func (p *Platform) CarryDelta(pos Vec2) Vec2 {
	rel := pos.Sub(p.Center)
	rotated := rel.Rot(p.lastDAngle)
	return rotated.Sub(rel).Add(p.lastDelta)
}

// World holds all colliders. Slices are iterated in insertion order only,
// keeping the simulation deterministic.
type World struct {
	Boxes     []Box
	Slopes    []Slope
	Platforms []*Platform
}

// Step advances all platforms by dt and records their deltas.
func (w *World) Step(dt float64) {
	for _, p := range w.Platforms {
		p.lastDelta = p.Vel.Mul(dt)
		p.lastDAngle = p.AngularVel * dt
		p.Center = p.Center.Add(p.lastDelta)
		p.Angle += p.lastDAngle
	}
}

// PlatformByID returns the platform with the given id, or nil.
func (w *World) PlatformByID(id int) *Platform {
	for _, p := range w.Platforms {
		if p.ID == id {
			return p
		}
	}
	return nil
}
