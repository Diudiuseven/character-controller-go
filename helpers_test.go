package character

import (
	"math"
	"testing"
)

// testWorld returns a world with a large floor at y=1.
func testWorld() *World {
	w := NewWorld()
	w.AddBox(0, Vec2{-50, 0}, Vec2{50, 1}, -1)
	return w
}

func approxEqual(a, b float64, tol float64) bool {
	return math.Abs(a-b) <= tol
}

func vecApproxEqual(a, b Vec2, tol float64) bool {
	return approxEqual(a.X, b.X, tol) && approxEqual(a.Y, b.Y, tol)
}

// runSteps advances the controller for n steps with the given input.
func runSteps(c *Controller, w *World, n int, in Input) {
	for i := 0; i < n; i++ {
		c.Step(w, in)
	}
}

// runSequence advances the controller with a per-step input provider.
func runSequence(c *Controller, w *World, n int, at func(i int) Input) {
	for i := 0; i < n; i++ {
		c.Step(w, at(i))
	}
}

func assertStateEqual(t *testing.T, a, b State) {
	t.Helper()
	if !vecApproxEqual(a.Pos, b.Pos, 1e-9) {
		t.Fatalf("pos mismatch: %v vs %v", a.Pos, b.Pos)
	}
	if !vecApproxEqual(a.Vel, b.Vel, 1e-9) {
		t.Fatalf("vel mismatch: %v vs %v", a.Vel, b.Vel)
	}
	if a.Ground != b.Ground {
		t.Fatalf("ground mismatch: %d vs %d", a.Ground, b.Ground)
	}
	if a.PlatformID != b.PlatformID {
		t.Fatalf("platform mismatch: %d vs %d", a.PlatformID, b.PlatformID)
	}
	if a.StepCount != b.StepCount {
		t.Fatalf("step count mismatch: %d vs %d", a.StepCount, b.StepCount)
	}
}
