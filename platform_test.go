package character

import (
	"testing"
)

// TestPlatformCarryTranslation: a standing character moves with a translating
// platform without extra input displacement.
func TestPlatformCarryTranslation(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(1, Vec2{5, 2}, 1, 0.5)
	p.LinVel = Vec2{4, 0}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{})
	}
	s := c.State()
	expectedX := 5 + 4.0*1.0
	if s.Ground != Grounded || s.PlatformID != 1 {
		t.Fatalf("expected to stay grounded on platform, got ground=%d plat=%d", s.Ground, s.PlatformID)
	}
	if !approxEqual(s.Pos.X, expectedX, 0.05) {
		t.Fatalf("expected carried to x≈%.2f, got %.2f", expectedX, s.Pos.X)
	}
}

// TestPlatformCarryRotation: rotation about the pivot carries the character
// along the platform's arc.
func TestPlatformCarryRotation(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(1, Vec2{5, 3}, 1, 0.5)
	p.AngVel = 0.4
	c, _ := NewController(DefaultConfig())
	// character at center-top, r=(0, 0.5)
	c.SetState(State{Pos: Vec2{5, 3.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	startX := c.State().Pos.X
	for i := 0; i < 20; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{})
	}
	s := c.State()
	if s.Ground != Grounded {
		t.Fatalf("expected to stay grounded on rotating platform, got %d", s.Ground)
	}
	if s.Pos.X >= startX {
		t.Fatalf("expected rotation to carry character left, got x=%.3f", s.Pos.X)
	}
}

// TestPlatformAccelerationNoFling: a platform that accelerates drags the
// character along without a sudden velocity spike.
func TestPlatformAccelerationNoFling(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(1, Vec2{5, 2}, 1, 0.5)
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	// First step: platform at rest.
	w.Step(DefaultConfig().FixedDt)
	c.Step(w, Input{})
	// Suddenly set platform velocity.
	p.LinVel = Vec2{8, 0}
	w.Step(DefaultConfig().FixedDt)
	c.Step(w, Input{})
	s := c.State()
	// Character should be near platform speed, not spiked beyond it.
	if s.Vel.X > 8.5 {
		t.Fatalf("velocity spiked beyond platform speed: %.2f", s.Vel.X)
	}
	if s.Pos.X < 5 {
		t.Fatalf("character should be carried forward, got x=%.2f", s.Pos.X)
	}
}

// TestPlatformVanishNoFling: when the platform disappears the character keeps
// its momentum and becomes airborne; no velocity is added or removed.
func TestPlatformVanishNoFling(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	p := w.AddPlatform(1, Vec2{5, 2}, 1, 0.5)
	p.LinVel = Vec2{5, 0}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{3, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	w.Step(DefaultConfig().FixedDt)
	c.Step(w, Input{})
	before := c.State()
	// Remove the platform.
	w.platforms = nil
	w.boxes = w.boxes[:1]
	c.Step(w, Input{})
	after := c.State()
	if after.PlatformID != -1 {
		t.Fatalf("expected platform id cleared, got %d", after.PlatformID)
	}
	if after.Ground != Airborne {
		t.Fatalf("expected airborne after vanish, got %d", after.Ground)
	}
	if after.Vel.X > before.Vel.X+0.01 {
		t.Fatalf("velocity must not increase (no fling): %.2f -> %.2f", before.Vel.X, after.Vel.X)
	}
}

// TestPlatformVerticalCarry: a platform moving down carries the character
// down with it (no separation).
func TestPlatformVerticalCarry(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(1, Vec2{5, 5}, 1, 0.5)
	p.LinVel = Vec2{0, -3}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 5.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	startY := c.State().Pos.Y
	for i := 0; i < 30; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{})
	}
	s := c.State()
	if s.Ground != Grounded {
		t.Fatalf("expected to stay grounded on downward platform, got %d", s.Ground)
	}
	if s.Pos.Y >= startY {
		t.Fatalf("expected to be carried down, y: %.2f -> %.2f", startY, s.Pos.Y)
	}
}

// TestPlatformInputRelative: while on a moving platform, input moves the
// character relative to the platform.
func TestPlatformInputRelative(t *testing.T) {
	w := NewWorld()
	p := w.AddPlatform(1, Vec2{5, 2}, 2, 0.5)
	p.LinVel = Vec2{4, 0}
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{5, 2.5}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: 1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		w.Step(DefaultConfig().FixedDt)
		c.Step(w, Input{MoveX: 1})
	}
	s := c.State()
	platformX := 5 + 4.0*1.0
	if s.Pos.X <= platformX+1 {
		t.Fatalf("expected input to move relative to platform, platform at %.2f, char at %.2f", platformX, s.Pos.X)
	}
}
