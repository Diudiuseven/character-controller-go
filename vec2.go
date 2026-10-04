package character

import "math"

// Vec2 is a deterministic 2D vector. All operations use float64 arithmetic
// (IEEE 754 round-to-nearest), which is deterministic across platforms.
type Vec2 struct {
	X float64
	Y float64
}

func (v Vec2) Add(o Vec2) Vec2      { return Vec2{v.X + o.X, v.Y + o.Y} }
func (v Vec2) Sub(o Vec2) Vec2      { return Vec2{v.X - o.X, v.Y - o.Y} }
func (v Vec2) Scale(s float64) Vec2 { return Vec2{v.X * s, v.Y * s} }
func (v Vec2) Neg() Vec2            { return Vec2{-v.X, -v.Y} }
func (v Vec2) Dot(o Vec2) float64   { return v.X*o.X + v.Y*o.Y }
func (v Vec2) Cross(o Vec2) float64 { return v.X*o.Y - v.Y*o.X }

// Perp returns the counter-clockwise perpendicular vector (-Y, X).
func (v Vec2) Perp() Vec2 { return Vec2{-v.Y, v.X} }

func (v Vec2) Length() float64 { return math.Sqrt(v.X*v.X + v.Y*v.Y) }

func (v Vec2) Normalized() Vec2 {
	l := v.Length()
	if l == 0 {
		return Vec2{}
	}
	return Vec2{v.X / l, v.Y / l}
}

// Lerp moves from v toward o by factor t (clamped to [0,1] by caller convention).
func (v Vec2) Lerp(o Vec2, t float64) Vec2 {
	return Vec2{v.X + (o.X-v.X)*t, v.Y + (o.Y-v.Y)*t}
}

// Min returns the component-wise minimum.
func MinVec(a, b Vec2) Vec2 {
	if a.X < b.X {
		b.X = a.X
	}
	if a.Y < b.Y {
		b.Y = a.Y
	}
	return b
}

// Max returns the component-wise maximum.
func MaxVec(a, b Vec2) Vec2 {
	if a.X > b.X {
		b.X = a.X
	}
	if a.Y > b.Y {
		b.Y = a.Y
	}
	return b
}

func absFloat(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
