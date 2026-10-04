package character

import (
	"testing"
)

// TestCoyoteTime: jumping shortly after leaving a ledge still works.
func TestCoyoteTime(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	// Walk off the ledge (no ground ahead, but test world has floor everywhere;
	// instead simulate by setting airborne with a small coyote timer).
	c.state.Ground = Airborne
	c.state.CoyoteTimer = cfg.CoyoteTime
	c.Step(w, Input{JumpPressed: true})
	s := c.State()
	if s.Vel.Y <= 0 {
		t.Fatalf("coyote jump should grant upward velocity, got %.2f", s.Vel.Y)
	}
}

// TestCoyoteTimeExpired: jumping after the coyote window does nothing.
func TestCoyoteTimeExpired(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 10}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 60, Input{})
	// After landing, walk off and wait out coyote time.
	c.state.Ground = Airborne
	c.state.CoyoteTimer = 0
	c.state.Pos = Vec2{0, 5}
	c.state.Vel = Vec2{0, 0}
	c.Step(w, Input{JumpPressed: true})
	s := c.State()
	if s.Vel.Y > 0 {
		t.Fatalf("jump after coyote expiry should not fire, got vy=%.2f", s.Vel.Y)
	}
}

// TestJumpBuffer: a jump pressed slightly before landing fires on landing.
func TestJumpBuffer(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 5}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	// Press jump while still airborne (within buffer window before landing).
	for i := 0; i < 20; i++ {
		in := Input{}
		if i == 5 {
			in.JumpPressed = true
		}
		c.Step(w, in)
	}
	s := c.State()
	// After buffered jump, character should have left the ground again.
	if s.Ground == Grounded && s.Pos.Y <= 1.01 {
		t.Fatalf("buffered jump should have fired, pos=%.2f ground=%d", s.Pos.Y, s.Ground)
	}
}

// TestJumpApex: jump reaches a deterministic apex and returns.
func TestJumpApex(t *testing.T) {
	w := testWorld()
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	c.Step(w, Input{JumpPressed: true})
	maxY := 1.0
	for i := 0; i < 60; i++ {
		c.Step(w, Input{})
		if c.State().Pos.Y > maxY {
			maxY = c.State().Pos.Y
		}
	}
	expectedApex := 1.0 + cfg.JumpSpeed*cfg.JumpSpeed/(2*cfg.Gravity)
	if !approxEqual(maxY, expectedApex, 0.2) {
		t.Fatalf("jump apex: expected ≈%.2f, got %.2f", expectedApex, maxY)
	}
}

// TestSerializationRoundTrip: state serializes to JSON and restores exactly.
func TestSerializationRoundTrip(t *testing.T) {
	s := State{
		Pos:             Vec2{1.5, 2.5},
		Vel:             Vec2{-3.25, 4.0},
		Ground:          Sliding,
		GroundNormal:    Vec2{0.707, 0.707},
		SlopeAngleDeg:   45,
		PlatformID:      7,
		CoyoteTimer:     0.05,
		JumpBufferTimer: 0.08,
		StepCount:       42,
	}
	data, err := MarshalState(s)
	if err != nil {
		t.Fatal(err)
	}
	restored, err := UnmarshalState(data)
	if err != nil {
		t.Fatal(err)
	}
	assertStateEqual(t, s, restored)
}

// TestSerializationCheckpointContinues: a checkpoint continues identically.
func TestSerializationCheckpointContinues(t *testing.T) {
	w := testWorld()
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 50; i++ {
		c.Step(w, Input{MoveX: 1})
	}
	data, _ := MarshalState(c.State())
	restored, _ := UnmarshalState(data)
	c2, _ := NewController(DefaultConfig())
	c2.SetState(restored)
	assertStateEqual(t, c.State(), c2.State())
}
