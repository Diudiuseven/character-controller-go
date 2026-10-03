package controller

import (
	"math"
	"testing"
)

func TestWallSlideNoJitter(t *testing.T) {
	cfg := DefaultConfig()
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(50, 0)}, // floor
		{Min: V(3, 0), Max: V(4, 5)},     // wall
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	prevX := 0.0
	for i := 0; i < 300; i++ {
		s.Step(Input{Move: 1})
		// Never penetrate the wall face (half width = 0.3).
		if s.Char.Pos.X > 3-0.3+1e-9 {
			t.Fatalf("step %d: penetrated wall, x=%v", i, s.Char.Pos.X)
		}
		if s.Char.Pos.X < prevX {
			t.Fatalf("step %d: position oscillated backward %v < %v", i, s.Char.Pos.X, prevX)
		}
		prevX = s.Char.Pos.X
	}
	// Pressed against the wall, stays grounded and stable.
	if s.Char.State != StateGrounded {
		t.Fatalf("expected grounded at wall, got %v", s.Char.State)
	}
	if math.Abs(s.Char.Pos.X-(3-0.3)) > 1e-9 {
		t.Fatalf("expected rest at wall face, got %v", s.Char.Pos.X)
	}
}

func TestAirborneWallSlide(t *testing.T) {
	cfg := DefaultConfig()
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(50, 0)},
		{Min: V(2, 0), Max: V(3, 50)},
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	s.Step(Input{Jump: true, Move: 1})
	// While pushed against the wall in the air, the character must slide
	// smoothly: vertical motion keeps going, no sticking.
	landed := false
	for i := 0; i < 300; i++ {
		s.Step(Input{Move: 1})
		if s.Char.Pos.X > 2-0.3+1e-9 {
			t.Fatalf("step %d: penetrated wall", i)
		}
		if s.Char.State == StateGrounded {
			landed = true
			break
		}
	}
	if !landed {
		t.Fatalf("never landed")
	}
}

func TestFloorSeamNoSnag(t *testing.T) {
	cfg := DefaultConfig()
	// Two floor boxes meeting at x=0 with identical tops.
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(0, 0)},
		{Min: V(0, -1), Max: V(50, 0)},
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(-2, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	prevY := s.Char.Pos.Y
	for i := 0; i < 200; i++ {
		s.Step(Input{Move: 1})
		if s.Char.Pos.Y != prevY {
			t.Fatalf("step %d: vertical snag at seam, y=%v x=%v", i, s.Char.Pos.Y, s.Char.Pos.X)
		}
		if i > 10 && s.Char.Vel.X != cfg.MaxRunSpeed {
			t.Fatalf("step %d: speed dropped at seam: %v", i, s.Char.Vel.X)
		}
	}
	if s.Char.Pos.X < 2 {
		t.Fatalf("did not cross the seam, x=%v", s.Char.Pos.X)
	}
}

func TestStepUpStairs(t *testing.T) {
	cfg := DefaultConfig()
	// Staircase of 5 steps, each 0.3 high (<= MaxStepHeight 0.35).
	w := &World{Boxes: []Box{{Min: V(-500, -1), Max: V(500, 0)}}}
	for i := 0; i < 5; i++ {
		x0 := 2.0 + float64(i)*0.8
		top := 0.3 * float64(i+1)
		w.Boxes = append(w.Boxes, Box{Min: V(x0, 0), Max: V(x0+0.8, top)})
	}
	// Upper floor continuing from the top step (last step ends at x=6.0).
	w.Boxes = append(w.Boxes, Box{Min: V(6.0, 0), Max: V(500, 1.5)})
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	runSteps(s, 600, Input{Move: 1})
	// Must have climbed to the top step (1.5 high) and passed it.
	if s.Char.Pos.Y != 1.5 {
		t.Fatalf("did not climb stairs, y=%v x=%v", s.Char.Pos.Y, s.Char.Pos.X)
	}
	if s.Char.Pos.X < 6 {
		t.Fatalf("did not traverse stairs, x=%v", s.Char.Pos.X)
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("expected grounded on upper floor, got %v", s.Char.State)
	}
}

func TestStepTooHighBlocks(t *testing.T) {
	cfg := DefaultConfig()
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(50, 0)},
		{Min: V(2, 0), Max: V(3, 0.5)}, // 0.5 > MaxStepHeight 0.35
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	runSteps(s, 120, Input{Move: 1})
	if s.Char.Pos.X > 2-0.3+1e-9 {
		t.Fatalf("climbed a too-high step, x=%v y=%v", s.Char.Pos.X, s.Char.Pos.Y)
	}
	if s.Char.Pos.Y != 0 {
		t.Fatalf("unexpected lift, y=%v", s.Char.Pos.Y)
	}
}

func TestCeilingBump(t *testing.T) {
	cfg := DefaultConfig()
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(50, 0)},
		{Min: V(-50, 2.2), Max: V(50, 3)}, // low ceiling
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	s.Step(Input{Jump: true})
	for i := 0; i < 300; i++ {
		s.Step(Input{})
		// Head at Pos.Y+1.7 must never enter the ceiling.
		if s.Char.Pos.Y+1.7 > 2.2+1e-9 {
			t.Fatalf("step %d: head penetrated ceiling, y=%v", i, s.Char.Pos.Y)
		}
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("expected to land again, got %v", s.Char.State)
	}
}

func TestCornerNoStick(t *testing.T) {
	cfg := DefaultConfig()
	// Wall meeting floor: jumping into the corner must not stick.
	w := &World{Boxes: []Box{
		{Min: V(-50, -1), Max: V(50, 0)},
		{Min: V(2, 0), Max: V(3, 50)},
	}}
	s, err := NewSim(cfg, w, NewCharacter(V(0, 0)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	s.Step(Input{Jump: true, Move: 1})
	ys := []float64{}
	for i := 0; i < 200 && s.Char.State != StateGrounded; i++ {
		s.Step(Input{Move: 1})
		ys = append(ys, s.Char.Pos.Y)
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("stuck in corner, ys=%v", ys)
	}
}
