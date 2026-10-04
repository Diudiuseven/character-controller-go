package character

import (
	"testing"
)

// TestPhysicsGravity: airborne character accelerates downward.
func TestPhysicsGravity(t *testing.T) {
	w := NewWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 10}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 10, Input{})
	s := c.State()
	expectedY := 10 - 0.5*cfg.Gravity*float64(10*10)*cfg.FixedDt*cfg.FixedDt
	if !approxEqual(s.Pos.Y, expectedY, 0.5) {
		t.Fatalf("gravity: expected y≈%.2f, got %.2f", expectedY, s.Pos.Y)
	}
	if s.Vel.Y >= 0 {
		t.Fatalf("expected downward velocity, got %.2f", s.Vel.Y)
	}
}

// TestPhysicsTerminalVelocity: fall speed never exceeds MaxFallSpeed.
func TestPhysicsTerminalVelocity(t *testing.T) {
	w := NewWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 1000}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 600, Input{})
	s := c.State()
	if s.Vel.Y < -cfg.MaxFallSpeed-1e-9 {
		t.Fatalf("terminal velocity exceeded: %.2f < %.2f", s.Vel.Y, -cfg.MaxFallSpeed)
	}
	if s.Vel.Y > -cfg.MaxFallSpeed+1.0 {
		t.Fatalf("expected to reach terminal velocity, got %.2f", s.Vel.Y)
	}
}

// TestPhysicsGroundFriction: grounded character with no input decelerates.
func TestPhysicsGroundFriction(t *testing.T) {
	w := testWorld()
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{10, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 120, Input{})
	s := c.State()
	if s.Vel.X > 0.05 {
		t.Fatalf("ground friction should stop the character, vx=%.4f", s.Vel.X)
	}
}

// TestPhysicsAirFrictionWeak: airborne horizontal momentum is preserved with
// no input; only the configured (weak) air drag decays it.
func TestPhysicsAirFrictionWeak(t *testing.T) {
	w := NewWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 10}, Vel: Vec2{10, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 30, Input{})
	s := c.State()
	// Drag model: vx *= (1 - drag*dt) each step.
	expected := 10.0
	for i := 0; i < 30; i++ {
		expected *= (1 - cfg.AirDrag*cfg.FixedDt)
	}
	if !approxEqual(s.Vel.X, expected, 0.01) {
		t.Fatalf("air drag: expected vx=%.3f, got %.3f", expected, s.Vel.X)
	}
	if s.Vel.X <= 0 {
		t.Fatalf("air drag must not stop or reverse, vx=%.3f", s.Vel.X)
	}
}

// TestPhysicsMoveSpeedClamp: grounded character accelerates toward MaxMoveSpeed.
func TestPhysicsMoveSpeedClamp(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 120, Input{MoveX: 1})
	s := c.State()
	if s.Vel.X > cfg.MaxMoveSpeed+0.01 {
		t.Fatalf("move speed should clamp to %.2f, got %.2f", cfg.MaxMoveSpeed, s.Vel.X)
	}
	if s.Vel.X < cfg.MaxMoveSpeed-0.5 {
		t.Fatalf("should reach near max speed, got %.2f", s.Vel.X)
	}
}

// TestPhysicsExtremeFrictionZero: zero friction does not reverse velocity.
func TestPhysicsExtremeFrictionZero(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	cfg.GroundFriction = 0
	c, err := NewController(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{5, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 60, Input{})
	s := c.State()
	if s.Vel.X < -1e-9 {
		t.Fatalf("zero friction must not reverse velocity, vx=%.4f", s.Vel.X)
	}
}

// TestPhysicsExtremeDragHuge: huge drag damps motion without reversing.
func TestPhysicsExtremeDragHuge(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	cfg.AirDrag = 1000
	c, err := NewController(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{5, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 10, Input{})
	s := c.State()
	if s.Vel.X < -1e-9 {
		t.Fatalf("huge drag must not reverse velocity, vx=%.4f", s.Vel.X)
	}
}

// TestPhysicsExtremeGravity: very strong gravity still lands and stays grounded.
func TestPhysicsExtremeGravity(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	cfg.Gravity = 500
	c, err := NewController(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.SetState(State{Pos: Vec2{0, 5}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 30, Input{})
	s := c.State()
	if s.Ground != Grounded {
		t.Fatalf("expected to land under strong gravity, got %d", s.Ground)
	}
	if s.Pos.Y != 1 {
		t.Fatalf("expected feet at y=1, got %.2f", s.Pos.Y)
	}
}

// TestPhysicsNoReverseOnLand: landing does not bounce or reverse vertical velocity.
func TestPhysicsNoReverseOnLand(t *testing.T) {
	w := testWorld()
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{0, 5}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	prevY := 5.0
	for i := 0; i < 60; i++ {
		c.Step(w, Input{})
		s := c.State()
		if s.Pos.Y > prevY+1e-9 {
			t.Fatalf("position must never move upward when falling, step %d: %.4f -> %.4f", i, prevY, s.Pos.Y)
		}
		prevY = s.Pos.Y
	}
}
