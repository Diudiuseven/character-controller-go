package controller

import (
	"math"
	"testing"
)

func TestTerminalVelocityClamp(t *testing.T) {
	cfg := DefaultConfig()
	cfg.TerminalVelocity = 10
	cfg.JumpSpeed = 5
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 1000)))
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 600; i++ {
		s.Step(Input{})
		if s.Char.Vel.Y < -cfg.TerminalVelocity {
			t.Fatalf("fell past terminal velocity: %v", s.Char.Vel.Y)
		}
		if i > 100 && s.Char.Vel.Y != -cfg.TerminalVelocity {
			t.Fatalf("expected exactly terminal velocity, got %v", s.Char.Vel.Y)
		}
	}
}

func TestGroundAccelAndFriction(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	// Accelerate right: speed must approach but never exceed MaxRunSpeed.
	for i := 0; i < 120; i++ {
		s.Step(Input{Move: 1})
		if s.Char.Vel.X > cfg.MaxRunSpeed {
			t.Fatalf("exceeded max run speed: %v", s.Char.Vel.X)
		}
	}
	if s.Char.Vel.X != cfg.MaxRunSpeed {
		t.Fatalf("expected to reach max run speed, got %v", s.Char.Vel.X)
	}
	// Release input: friction decelerates to exactly zero, never negative.
	for i := 0; i < 120; i++ {
		s.Step(Input{})
		if s.Char.Vel.X < 0 {
			t.Fatalf("friction reversed velocity: %v", s.Char.Vel.X)
		}
	}
	if s.Char.Vel.X != 0 {
		t.Fatalf("expected full stop, got %v", s.Char.Vel.X)
	}
}

func TestAirControlWeakerThanGround(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	s.Step(Input{Jump: true})
	s.Step(Input{Move: 1})
	airSpeed := s.Char.Vel.X
	// One airborne step of accel must be smaller than one grounded step.
	s2, _ := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	runSteps(s2, 10, Input{})
	s2.Step(Input{Move: 1})
	groundSpeed := s2.Char.Vel.X
	if !(airSpeed < groundSpeed) {
		t.Fatalf("air accel %v not weaker than ground accel %v", airSpeed, groundSpeed)
	}
}

func TestExtremeParametersDoNotDiverge(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Gravity = 500
	cfg.TerminalVelocity = 200
	cfg.GroundFriction = 10000
	cfg.GroundAccel = 10000
	cfg.MaxRunSpeed = 5
	cfg.Dt = 0.1
	if err := cfg.Validate(); err != nil {
		t.Fatal(err)
	}
	w := &World{Boxes: []Box{{Min: V(-5000, -1), Max: V(5000, 0)}}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 5)))
	if err != nil {
		t.Fatal(err)
	}
	inputs := []Input{{Move: 1}, {Move: -1}, {}, {Jump: true}, {Move: 1}}
	for i := 0; i < 2000; i++ {
		s.Step(inputs[i%len(inputs)])
		if math.IsNaN(s.Char.Pos.X) || math.IsNaN(s.Char.Pos.Y) ||
			math.IsInf(s.Char.Vel.X, 0) || math.IsInf(s.Char.Vel.Y, 0) {
			t.Fatalf("step %d diverged: pos=%v vel=%v", i, s.Char.Pos, s.Char.Vel)
		}
		if s.Char.Pos.Y < -1e-9 {
			t.Fatalf("step %d sank below floor: %v", i, s.Char.Pos.Y)
		}
	}
}

func TestCoyoteTimeAndJumpBuffer(t *testing.T) {
	cfg := DefaultConfig()
	cfg.CoyoteTime = 0.12
	cfg.JumpBufferTime = 0.12
	// Coyote: walk off a ledge, jump a few steps later, still jumps.
	w := &World{Boxes: []Box{{Min: V(-50, -1), Max: V(0, 0)}}}
	s, err := NewSim(cfg, w, NewCharacter(V(-1, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	// Run right until we leave the ledge at x=0.
	for i := 0; i < 200 && s.Char.State != StateAirborne; i++ {
		s.Step(Input{Move: 1})
	}
	if s.Char.State != StateAirborne {
		t.Fatalf("expected to be airborne after leaving ledge")
	}
	s.Step(Input{}) // 1-2 steps past the edge: still inside coyote window
	s.Step(Input{Jump: true})
	if s.Char.Vel.Y <= 0 {
		t.Fatalf("coyote jump did not fire, vy=%v", s.Char.Vel.Y)
	}

	// Buffer: press jump just before landing, it fires on touchdown.
	s2, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s2, 10, Input{})
	s2.Step(Input{Jump: true})
	// Fall until close to the ground but definitely still airborne.
	for !(s2.Char.Vel.Y < 0 && s2.Char.Pos.Y < 0.3) {
		s2.Step(Input{})
	}
	if s2.Char.State != StateAirborne {
		t.Fatalf("setup: expected airborne before buffered press")
	}
	s2.Step(Input{Jump: true}) // buffered while still airborne
	for i := 0; i < 30; i++ {
		s2.Step(Input{})
		if s2.Char.State == StateAirborne {
			return // jumped again right after landing: buffer worked
		}
	}
	t.Fatalf("buffered jump did not fire after landing")
}

func TestInputClampedAndNaNSafe(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	runSteps(s, 120, Input{Move: 100}) // way beyond [-1,1]
	if s.Char.Vel.X > cfg.MaxRunSpeed {
		t.Fatalf("unclamped input exceeded max speed: %v", s.Char.Vel.X)
	}
	nan := math.NaN()
	s.Step(Input{Move: nan})
	if math.IsNaN(s.Char.Pos.X) || math.IsNaN(s.Char.Vel.X) {
		t.Fatalf("NaN input poisoned the state")
	}
}
