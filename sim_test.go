package controller

import "testing"

// flatWorld returns a world with a single floor box at y=0 spanning x in
// [-50, 50].
func flatWorld() *World {
	return &World{
		Boxes: []Box{{Min: V(-50, -1), Max: V(50, 0)}},
	}
}

func runSteps(s *Sim, n int, in Input) {
	for i := 0; i < n; i++ {
		s.Step(in)
	}
}

func TestFallsAndLandsOnFloor(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 2)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 120, Input{})
	if s.Char.State != StateGrounded {
		t.Fatalf("expected grounded, got %v", s.Char.State)
	}
	if s.Char.Pos.Y != 0 {
		t.Fatalf("expected feet at y=0, got %v", s.Char.Pos.Y)
	}
	if s.Char.Vel.Y != 0 {
		t.Fatalf("expected zero vertical velocity, got %v", s.Char.Vel.Y)
	}
	// Stays put with no input.
	for i := 0; i < 60; i++ {
		s.Step(Input{})
		if s.Char.Pos.Y != 0 || s.Char.Pos.X != 0 {
			t.Fatalf("character drifted to %v", s.Char.Pos)
		}
	}
}

func TestDeterministicSameInputs(t *testing.T) {
	cfg := DefaultConfig()
	inputs := make([]Input, 600)
	for i := range inputs {
		if i%7 == 0 {
			inputs[i].Move = 1
		} else if i%5 == 0 {
			inputs[i].Move = -1
		}
		if i%50 == 10 {
			inputs[i].Jump = true
		}
	}
	run := func() []Vec2 {
		s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0.5)))
		if err != nil {
			t.Fatal(err)
		}
		traj := make([]Vec2, 0, len(inputs))
		for _, in := range inputs {
			s.Step(in)
			traj = append(traj, s.Char.Pos)
		}
		return traj
	}
	a := run()
	b := run()
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("step %d diverged: %v vs %v", i, a[i], b[i])
		}
	}
}

func TestJumpApexAndLand(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 30, Input{}) // settle
	s.Step(Input{Jump: true})
	if s.Char.State != StateAirborne {
		t.Fatalf("expected airborne after jump")
	}
	maxY := 0.0
	for i := 0; i < 200 && s.Char.State == StateAirborne; i++ {
		s.Step(Input{})
		if s.Char.Pos.Y > maxY {
			maxY = s.Char.Pos.Y
		}
	}
	// v^2/(2g) = 144/60 = 2.4 expected apex (approx, discrete steps).
	if maxY < 2.2 || maxY > 2.6 {
		t.Fatalf("unexpected jump apex %v", maxY)
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("expected to land, got %v", s.Char.State)
	}
}
