package controller

import "math"

// Vec2 is a 2D vector. All math uses float64 in a fixed operation order so
// that results are deterministic across runs on the same platform.
type Vec2 struct {
	X, Y float64
}

func V(x, y float64) Vec2 { return Vec2{X: x, Y: y} }

func (a Vec2) Add(b Vec2) Vec2    { return Vec2{X: a.X + b.X, Y: a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2    { return Vec2{X: a.X - b.X, Y: a.Y - b.Y} }
func (a Vec2) Mul(s float64) Vec2 { return Vec2{X: a.X * s, Y: a.Y * s} }
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }
func (a Vec2) Len() float64       { return math.Hypot(a.X, a.Y) }

func (a Vec2) Normalize() Vec2 {
	l := a.Len()
	if l == 0 {
		return Vec2{}
	}
	return a.Mul(1 / l)
}

// Perp returns the vector rotated 90 degrees counter-clockwise.
func (a Vec2) Perp() Vec2 { return Vec2{X: -a.Y, Y: a.X} }

// Rot rotates the vector by angle radians (counter-clockwise).
func (a Vec2) Rot(angle float64) Vec2 {
	c, s := math.Cos(angle), math.Sin(angle)
	return Vec2{X: a.X*c - a.Y*s, Y: a.X*s + a.Y*c}
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
