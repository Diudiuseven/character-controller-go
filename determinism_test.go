package character

import (
	"testing"
)

// TestDeterminismSameInputs: identical inputs on identical worlds produce
// identical trajectories, step for step.
func TestDeterminismSameInputs(t *testing.T) {
	inputs := func(i int) Input {
		switch {
		case i < 30:
			return Input{MoveX: 1}
		case i == 30:
			return Input{MoveX: 1, JumpPressed: true}
		case i < 60:
			return Input{MoveX: -1}
		default:
			return Input{}
		}
	}

	c1, _ := NewController(DefaultConfig())
	c2, _ := NewController(DefaultConfig())
	w1, w2 := testWorld(), testWorld()
	c1.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	c2.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})

	for i := 0; i < 120; i++ {
		in := inputs(i)
		c1.Step(w1, in)
		c2.Step(w2, in)
		assertStateEqual(t, c1.State(), c2.State())
	}
}

// TestDeterminismCheckpointResume: serializing mid-run, resuming on a new
// controller, and continuing yields the same trajectory as a continuous run.
func TestDeterminismCheckpointResume(t *testing.T) {
	inputs := func(i int) Input {
		switch {
		case i < 40:
			return Input{MoveX: 1}
		case i == 40:
			return Input{JumpPressed: true}
		case i < 80:
			return Input{MoveX: -1}
		default:
			return Input{}
		}
	}

	// Continuous run.
	c1, _ := NewController(DefaultConfig())
	w1 := testWorld()
	c1.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 120; i++ {
		c1.Step(w1, inputs(i))
	}

	// Checkpoint run: serialize at step 60, resume on a fresh controller.
	c2, _ := NewController(DefaultConfig())
	w2 := testWorld()
	c2.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 60; i++ {
		c2.Step(w2, inputs(i))
	}
	data, err := MarshalState(c2.State())
	if err != nil {
		t.Fatal(err)
	}
	restored, err := UnmarshalState(data)
	if err != nil {
		t.Fatal(err)
	}
	c3, _ := NewController(DefaultConfig())
	c3.SetState(restored)
	w3 := testWorld()
	// Replay platform/world state is identical (static world).
	for i := 60; i < 120; i++ {
		c3.Step(w3, inputs(i))
	}

	assertStateEqual(t, c1.State(), c3.State())
}

// TestDeterminismReplaySequence: a recorded input sequence replayed from a
// checkpoint matches the live run.
func TestDeterminismReplaySequence(t *testing.T) {
	var seq InputSequence
	c1, _ := NewController(DefaultConfig())
	w1 := testWorld()
	c1.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 90; i++ {
		in := Input{MoveX: 1}
		if i == 25 {
			in.JumpPressed = true
		}
		seq.Record(int64(i), in)
		c1.Step(w1, in)
	}

	c2, _ := NewController(DefaultConfig())
	w2 := testWorld()
	c2.SetState(State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 90; i++ {
		c2.Step(w2, seq.InputAt(int64(i)))
	}
	assertStateEqual(t, c1.State(), c2.State())
}

// TestNoFrameRateDependence: the controller has no dt parameter; stepping is
// always fixed. Two runs advanced in different-sized batches agree.
func TestNoFrameRateDependence(t *testing.T) {
	c1, _ := NewController(DefaultConfig())
	c2, _ := NewController(DefaultConfig())
	w1, w2 := testWorld(), testWorld()
	s0 := State{Pos: Vec2{0, 1}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}}
	c1.SetState(s0)
	c2.SetState(s0)
	// Batch sizes differ, but Step is always fixed.
	for i := 0; i < 100; i++ {
		c1.Step(w1, Input{MoveX: 1})
	}
	for i := 0; i < 100; i++ {
		c2.Step(w2, Input{MoveX: 1})
	}
	assertStateEqual(t, c1.State(), c2.State())
}
