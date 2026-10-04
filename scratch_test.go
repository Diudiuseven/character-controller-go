package character

import (
	"math"
	"testing"
)

func TestScratchSlopeWalkUp(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{-2, 0}, Vec2{30, 1}, -1) // long ground
	w.AddRamp(1, Vec2{0, 1}, Vec2{6, 1 + 6*math.Tan(30*math.Pi/180)})
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{-1, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 30; i++ {
		c.Step(w, Input{MoveX: 1})
	}
	s := c.State()
	t.Logf("pos=%v ground=%d slope=%.2f", s.Pos, s.Ground, s.SlopeAngleDeg)
	if s.Ground != Grounded {
		t.Fatalf("expected grounded on slope, got %d", s.Ground)
	}
	if s.Pos.Y < 2 {
		t.Fatalf("expected to climb slope, y=%v", s.Pos.Y)
	}
}

func TestScratchSteepSlopeSlide(t *testing.T) {
	w := NewWorld()
	w.AddRamp(1, Vec2{0, 5}, Vec2{4, 0}) // 51 deg steep, down to the right
	w.AddBox(0, Vec2{-2, 0}, Vec2{30, 0.01}, -1)
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{2, 2.5}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	slid := false
	for i := 0; i < 60; i++ {
		c.Step(w, Input{})
		if c.State().Ground == Sliding {
			slid = true
		}
	}
	s := c.State()
	t.Logf("pos=%v ground=%d slid=%v", s.Pos, s.Ground, slid)
	if !slid {
		t.Fatalf("expected to enter Sliding state on steep slope")
	}
	if s.Pos.X <= 2 {
		t.Fatalf("expected to slide down toward +x, x=%v", s.Pos.X)
	}
}

func TestScratchStepUp(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{2, 1}, Vec2{20, 1.6}, -1) // 0.6 step (max)
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{1.5, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		c.Step(w, Input{MoveX: 1})
	}
	s := c.State()
	t.Logf("pos=%v ground=%d", s.Pos, s.Ground)
	if s.Pos.Y < 1.5 {
		t.Fatalf("expected to step up onto box, y=%v", s.Pos.Y)
	}
}

func TestScratchStepTooHigh(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{2, 1}, Vec2{3, 1.7}, -1) // 0.7 > max 0.6
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{1.5, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		c.Step(w, Input{MoveX: 1})
	}
	s := c.State()
	t.Logf("pos=%v ground=%d", s.Pos, s.Ground)
	if s.Pos.Y > 1.5 {
		t.Fatalf("should NOT step up 0.7, y=%v", s.Pos.Y)
	}
}

func TestScratchPlatformCarry(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	p := w.AddPlatform(2, Vec2{5, 2}, 1, 0.5)
	p.LinVel = Vec2{3, 0}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 2, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{})
	}
	s := c.State()
	t.Logf("pos=%v ground=%d plat=%d", s.Pos, s.Ground, s.PlatformID)
	if s.Ground != Grounded || s.PlatformID != 2 {
		t.Fatalf("expected to stay on platform, got ground=%d plat=%d", s.Ground, s.PlatformID)
	}
	platformX := 5 + 3.0*1.0
	if absFloat(s.Pos.X-platformX) > 0.1 {
		t.Fatalf("expected to be carried with platform (x=%.2f), got x=%v", platformX, s.Pos.X)
	}
}

func TestScratchPlatformRotate(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(2, Vec2{5, 3}, 1, 0.5)
	p.AngVel = 0.3 // slow enough to stay in contact
	c, _ := NewController(DefaultConfig())
	// character at center-top of platform
	c.SetState(State{Pos: Vec2{5, 3.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 2, GroundNormal: Vec2{0, 1}})
	startX := c.State().Pos.X
	for i := 0; i < 20; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{})
	}
	s := c.State()
	t.Logf("pos=%v ground=%d", s.Pos, s.Ground)
	if s.Ground != Grounded {
		t.Fatalf("expected to stay grounded on rotating platform, got %d", s.Ground)
	}
	if s.Pos.X >= startX {
		t.Fatalf("expected rotation to carry character left, x=%v", s.Pos.X)
	}
}

func TestScratchPlatformVanish(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	p := w.AddPlatform(2, Vec2{5, 2}, 1, 0.5)
	p.LinVel = Vec2{5, 0}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{2, 0}, Ground: Grounded, PlatformID: 2, GroundNormal: Vec2{0, 1}})
	w.Step(DefaultConfig().FixedDt)
	c.Step(w, Input{})
	before := c.State()
	// remove platform
	w.platforms = nil
	w.boxes = w.boxes[:1]
	c.Step(w, Input{})
	after := c.State()
	t.Logf("before=%v after=%v", before.Vel, after.Vel)
	if after.PlatformID != -1 {
		t.Fatalf("expected platform id cleared, got %d", after.PlatformID)
	}
	if after.Vel.X <= 0.35*before.Vel.X {
		t.Fatalf("momentum should be preserved on vanish (no instant brake), got %v -> %v", before.Vel.X, after.Vel.X)
	}
	if after.Vel.X > before.Vel.X+0.01 {
		t.Fatalf("velocity must not increase on vanish (no fling), got %v -> %v", before.Vel.X, after.Vel.X)
	}
}
