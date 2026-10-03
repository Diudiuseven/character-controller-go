package controller

import (
	"testing"
)

// slopeWorld builds: flat floor up to x=2, slope (2,0)->(7,rise), plateau
// from x=7 at height rise.
func slopeWorld(rise float64) *World {
	return &World{
		Boxes: []Box{
			{Min: V(-50, -1), Max: V(2, 0)},
			{Min: V(7, -1), Max: V(50, rise)},
		},
		Slopes: []Slope{{A: V(2, 0), B: V(7, rise)}},
	}
}

func TestClimbWalkableSlope(t *testing.T) {
	cfg := DefaultConfig()                                      // max slope 50 deg
	s, err := NewSim(cfg, slopeWorld(3), NewCharacter(V(0, 0))) // 31 deg
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	everAirborne := false
	for i := 0; i < 400; i++ {
		s.Step(Input{Move: 1})
		if s.Char.State == StateAirborne {
			everAirborne = true
		}
	}
	if everAirborne {
		t.Fatalf("went airborne while climbing a walkable slope")
	}
	if s.Char.Pos.Y != 3 {
		t.Fatalf("expected to reach plateau at y=3, got %v (x=%v)", s.Char.Pos.Y, s.Char.Pos.X)
	}
	if s.Char.Pos.X < 8 {
		t.Fatalf("did not cross the slope, x=%v", s.Char.Pos.X)
	}
}

func TestDescendSlopeStaysGlued(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, slopeWorld(3), NewCharacter(V(8, 3)))
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	airborneSteps := 0
	prevY := s.Char.Pos.Y
	for i := 0; i < 300; i++ {
		s.Step(Input{Move: -1})
		if s.Char.State == StateAirborne {
			airborneSteps++
		}
		if s.Char.Pos.Y > prevY+1e-9 {
			t.Fatalf("step %d: bounced uphill while descending", i)
		}
		prevY = s.Char.Pos.Y
	}
	if airborneSteps > 0 {
		t.Fatalf("went airborne %d steps while descending", airborneSteps)
	}
	if s.Char.Pos.X > 0 {
		t.Fatalf("did not descend to flat ground, pos=%v", s.Char.Pos)
	}
}

func TestSteepSlopeBlocksClimbing(t *testing.T) {
	cfg := DefaultConfig()                                      // max 50 + hyst 2
	s, err := NewSim(cfg, slopeWorld(8), NewCharacter(V(0, 0))) // 58 deg
	if err != nil {
		t.Fatal(err)
	}
	runSteps(s, 10, Input{})
	for i := 0; i < 300; i++ {
		s.Step(Input{Move: 1})
		if s.Char.Pos.Y > 0.5 {
			t.Fatalf("step %d: climbed a steep slope to y=%v", i, s.Char.Pos.Y)
		}
	}
	if s.Char.Pos.X > 2.01 {
		t.Fatalf("walked past steep slope base, x=%v", s.Char.Pos.X)
	}
}

func TestSteepSlopeSlidesDown(t *testing.T) {
	cfg := DefaultConfig()
	s, err := NewSim(cfg, slopeWorld(8), NewCharacter(V(4.5, 6)))
	if err != nil {
		t.Fatal(err)
	}
	// Drop onto the middle of the steep slope.
	sawSliding := false
	for i := 0; i < 600; i++ {
		s.Step(Input{})
		if s.Char.State == StateSliding {
			sawSliding = true
		}
		if s.Char.State == StateGrounded && s.Char.Pos.Y == 0 {
			break
		}
	}
	if !sawSliding {
		t.Fatalf("never entered sliding state, state=%v pos=%v", s.Char.State, s.Char.Pos)
	}
	// Must end up at the base on flat ground, not stuck on the slope.
	if s.Char.State != StateGrounded || s.Char.Pos.Y != 0 {
		t.Fatalf("did not reach flat ground: state=%v pos=%v", s.Char.State, s.Char.Pos)
	}
	if s.Char.Pos.X > 2 {
		t.Fatalf("slid past the slope base, x=%v", s.Char.Pos.X)
	}
}

func TestSlopeHysteresisNoJitter(t *testing.T) {
	cfg := DefaultConfig() // max 50, hysteresis 2 -> band [48,52]
	// Slope at 51 degrees: inside the band. rise = 5*tan(51deg).
	rise := 5 * 1.234897156535051 // tan(51°)
	s, err := NewSim(cfg, slopeWorld(rise), NewCharacter(V(4.5, rise/2+0.5)))
	if err != nil {
		t.Fatal(err)
	}
	// Drop onto the slope; whatever state it settles in must be stable.
	runSteps(s, 120, Input{})
	settled := s.Char.State
	for i := 0; i < 300; i++ {
		s.Step(Input{})
		if s.Char.State != settled {
			t.Fatalf("step %d: state jittered %v -> %v near critical angle",
				i, settled, s.Char.State)
		}
	}
}

func TestSlideContinuesAcrossBandThenStops(t *testing.T) {
	cfg := DefaultConfig()
	// Three segments: 60 deg, then 51 deg (inside hysteresis band), then
	// 40 deg (walkable). A character sliding from the steep part must keep
	// sliding across the band segment and stop on the walkable part.
	tan60, tan51, tan40 := 1.7320508075688772, 1.234897156535051, 0.8390996311772799
	w := &World{
		Boxes: []Box{{Min: V(-50, -10), Max: V(50, -3*tan51-3*tan40)}},
		Slopes: []Slope{
			{A: V(0, 3*tan60), B: V(3, 0)},
			{A: V(3, 0), B: V(6, -3*tan51)},
			{A: V(6, -3*tan51), B: V(9, -3*tan51-3*tan40)},
		},
	}
	s, err := NewSim(cfg, w, NewCharacter(V(1.5, 3*tan60+2)))
	if err != nil {
		t.Fatal(err)
	}
	states := []CharState{}
	for i := 0; i < 600; i++ {
		s.Step(Input{})
		states = append(states, s.Char.State)
		if s.Char.State == StateGrounded {
			break
		}
	}
	// Must have been sliding, and must end grounded on the 40 deg segment
	// (x beyond 6) rather than jittering in the band.
	sawSlide := false
	for _, st := range states {
		if st == StateSliding {
			sawSlide = true
		}
	}
	if !sawSlide {
		t.Fatalf("never slid")
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("did not settle on walkable segment, state=%v pos=%v", s.Char.State, s.Char.Pos)
	}
	if s.Char.Pos.X < 6 {
		t.Fatalf("stopped inside the hysteresis band at x=%v", s.Char.Pos.X)
	}
}
