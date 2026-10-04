package character

import (
	"math"
	"sort"
)

// BoxCollider is an axis-aligned solid box. Min is the bottom-left corner.
type BoxCollider struct {
	ID         int
	Min        Vec2
	Max        Vec2
	PlatformID int // -1 for static geometry, otherwise the carrying platform ID
}

// RampCollider is a one-sided (top-surface) slope segment from A to B.
// The collideable surface faces upward; approaching from below passes through.
type RampCollider struct {
	ID int
	A  Vec2
	B  Vec2
}

// Platform is a moving axis-aligned box that carries standing characters.
// It rotates about Pos (its center). Translation and rotation are sampled
// between fixed steps so carry motion is deterministic and never flings.
type Platform struct {
	ID        int
	Pos       Vec2 // pivot / center
	HalfW     float64
	HalfH     float64
	Angle     float64 // radians
	LinVel    Vec2    // units/second
	AngVel    float64 // radians/second
	PrevPos   Vec2
	PrevAngle float64
}

// AABB returns the current axis-aligned box of the platform.
func (p *Platform) AABB() (Vec2, Vec2) {
	return Vec2{p.Pos.X - p.HalfW, p.Pos.Y - p.HalfH},
		Vec2{p.Pos.X + p.HalfW, p.Pos.Y + p.HalfH}
}

// VelAt returns the world velocity of the platform surface at world point r.
func (p *Platform) VelAt(worldPoint Vec2) Vec2 {
	r := worldPoint.Sub(p.Pos)
	// v = LinVel + AngVel x r  (2D: perp(r) * omega)
	return p.LinVel.Add(r.Perp().Scale(p.AngVel))
}

// Step advances the platform by one fixed step and records the previous
// transform for carry delta computation.
func (p *Platform) Step(dt float64) {
	p.PrevPos = p.Pos
	p.PrevAngle = p.Angle
	p.Pos = p.Pos.Add(p.LinVel.Scale(dt))
	p.Angle += p.AngVel * dt
}

// CarryDelta returns the world displacement of a character standing on the
// platform between the previous and the current step: the platform's
// translation plus the rotation about its pivot.
func (p *Platform) CarryDelta(feet Vec2) Vec2 {
	dAngle := p.Angle - p.PrevAngle
	cosA := math.Cos(dAngle)
	sinA := math.Sin(dAngle)
	r := feet.Sub(p.PrevPos)
	// Rotate r by dAngle about the pivot.
	rot := Vec2{
		X: r.X*cosA - r.Y*sinA,
		Y: r.X*sinA + r.Y*cosA,
	}
	oldWorld := p.PrevPos.Add(r)
	newWorld := p.Pos.Add(rot)
	return newWorld.Sub(oldWorld)
}

// World holds all colliders and platforms. Colliders are kept sorted by ID
// so iteration order (and therefore resolution order) is deterministic.
type World struct {
	boxes     []*BoxCollider
	ramps     []*RampCollider
	platforms []*Platform
}

// NewWorld creates an empty world.
func NewWorld() *World { return &World{} }

// AddBox adds a static or platform-carried box and returns it.
func (w *World) AddBox(id int, min, max Vec2, platformID int) *BoxCollider {
	b := &BoxCollider{ID: id, Min: min, Max: max, PlatformID: platformID}
	w.boxes = append(w.boxes, b)
	sort.Slice(w.boxes, func(i, j int) bool { return w.boxes[i].ID < w.boxes[j].ID })
	return b
}

// AddRamp adds a slope segment and returns it.
func (w *World) AddRamp(id int, a, b Vec2) *RampCollider {
	r := &RampCollider{ID: id, A: a, B: b}
	w.ramps = append(w.ramps, r)
	sort.Slice(w.ramps, func(i, j int) bool { return w.ramps[i].ID < w.ramps[j].ID })
	return r
}

// AddPlatform adds a moving platform and a synced collision box for it.
func (w *World) AddPlatform(id int, center Vec2, halfW, halfH float64) *Platform {
	p := &Platform{ID: id, Pos: center, HalfW: halfW, HalfH: halfH, PrevPos: center}
	w.platforms = append(w.platforms, p)
	sort.Slice(w.platforms, func(i, j int) bool { return w.platforms[i].ID < w.platforms[j].ID })
	w.boxes = append(w.boxes, &BoxCollider{
		ID:         id,
		PlatformID: id,
		Min:        Vec2{center.X - halfW, center.Y - halfH},
		Max:        Vec2{center.X + halfW, center.Y + halfH},
	})
	sort.Slice(w.boxes, func(i, j int) bool { return w.boxes[i].ID < w.boxes[j].ID })
	return p
}

// syncPlatformBoxes updates platform-attached colliders to the current
// platform transform so collision tests never see stale geometry.
func (w *World) syncPlatformBoxes() {
	for _, b := range w.boxes {
		if b.PlatformID < 0 {
			continue
		}
		p := w.PlatformByID(b.PlatformID)
		if p == nil {
			continue
		}
		b.Min = Vec2{p.Pos.X - p.HalfW, p.Pos.Y - p.HalfH}
		b.Max = Vec2{p.Pos.X + p.HalfW, p.Pos.Y + p.HalfH}
	}
}

// PlatformByID returns the platform with the given ID, or nil.
func (w *World) PlatformByID(id int) *Platform {
	for _, p := range w.platforms {
		if p.ID == id {
			return p
		}
	}
	return nil
}

// Step advances every platform by one fixed step and syncs platform
// colliders. Call this before Controller.Step so carry deltas cover the
// current step.
func (w *World) Step(dt float64) {
	for _, p := range w.platforms {
		p.Step(dt)
	}
	w.syncPlatformBoxes()
}

// BoxColliders returns the sorted static box list.
func (w *World) BoxColliders() []*BoxCollider { return w.boxes }

// RampColliders returns the sorted ramp list.
func (w *World) RampColliders() []*RampCollider { return w.ramps }

// Platforms returns the sorted platform list.
func (w *World) Platforms() []*Platform { return w.platforms }

// RampInfo describes the surface under a point on a ramp.
type RampInfo struct {
	SurfaceY float64
	Normal   Vec2
	Slope    float64 // dy/dx of the ramp direction (positive = rising right)
	Found    bool
}

// RampAt returns the ramp surface information at world x, if x is within the
// ramp's horizontal extent.
func (r *RampCollider) RampAt(x float64) RampInfo {
	dx := r.B.X - r.A.X
	if dx == 0 {
		return RampInfo{Found: false}
	}
	t := (x - r.A.X) / dx
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	y := r.A.Y + (r.B.Y-r.A.Y)*t
	dir := Vec2{r.B.X - r.A.X, r.B.Y - r.A.Y}
	n := dir.Perp()
	if n.Y < 0 {
		n = n.Neg()
	}
	n = n.Normalized()
	return RampInfo{
		SurfaceY: y,
		Normal:   n,
		Slope:    (r.B.Y - r.A.Y) / dx,
		Found:    true,
	}
}

// overlapsBox reports whether the character box (feet at pos, half-width hw,
// height h) overlaps the given axis-aligned box.
func charOverlapsBox(pos Vec2, hw, h float64, b *BoxCollider) bool {
	cMin := Vec2{pos.X - hw, pos.Y}
	cMax := Vec2{pos.X + hw, pos.Y + h}
	return cMin.X < b.Max.X && cMax.X > b.Min.X &&
		cMin.Y < b.Max.Y && cMax.Y > b.Min.Y
}
