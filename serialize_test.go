package controller

import "testing"

// busyWorld has a floor, stairs, a slope and a moving platform so checkpoint
// tests exercise all state at once.
func busyWorld() *World {
	return &World{
		Boxes: []Box{
			{Min: V(-50, -1), Max: V(50, 0)},
			{Min: V(5, 0), Max: V(6, 0.3)},
		},
		Slopes: []Slope{{A: V(8, 0), B: V(13, 2)}},
		Platforms: []*Platform{
			{ID: 7, Center: V(-5, 1), Half: V(2, 0.5), Vel: V(1, 0.3), AngularVel: 0.1},
		},
	}
}

func scriptedInput(i int) Input {
	in := Input{}
	switch {
	case i%9 < 4:
		in.Move = 1
	case i%9 < 6:
		in.Move = -1
	}
	if i%23 == 5 || i%17 == 3 {
		in.Jump = true
	}
	return in
}

func TestCheckpointResumeBitExact(t *testing.T) {
	cfg := DefaultConfig()
	const total = 800
	const cut = 350

	// Continuous run.
	cont, err := NewSim(cfg, busyWorld(), NewCharacter(V(0, 0.5)))
	if err != nil {
		t.Fatal(err)
	}
	traj := make([]Vec2, total)
	for i := 0; i < total; i++ {
		cont.Step(scriptedInput(i))
		traj[i] = cont.Char.Pos
	}

	// Checkpointed run: simulate to `cut`, snapshot, restore into a fresh
	// sim, continue with the same inputs.
	first, err := NewSim(cfg, busyWorld(), NewCharacter(V(0, 0.5)))
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < cut; i++ {
		first.Step(scriptedInput(i))
	}
	snap, err := first.Snapshot()
	if err != nil {
		t.Fatal(err)
	}

	resumed, err := NewSim(cfg, busyWorld(), NewCharacter(V(0, 0.5)))
	if err != nil {
		t.Fatal(err)
	}
	if err := resumed.Restore(snap); err != nil {
		t.Fatal(err)
	}
	if resumed.StepCount != uint64(cut) {
		t.Fatalf("step count not restored: %d", resumed.StepCount)
	}
	for i := cut; i < total; i++ {
		resumed.Step(scriptedInput(i))
		if resumed.Char.Pos != traj[i] {
			t.Fatalf("step %d diverged after restore: got %v want %v",
				i, resumed.Char.Pos, traj[i])
		}
	}
}

func TestSnapshotIncludesPlatforms(t *testing.T) {
	cfg := DefaultConfig()
	mk := func() *Sim {
		s, err := NewSim(cfg, busyWorld(), NewCharacter(V(0, 0.5)))
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	a := mk()
	runSteps(a, 100, Input{Move: 1})
	snap, err := a.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	b := mk()
	if err := b.Restore(snap); err != nil {
		t.Fatal(err)
	}
	pa := a.World.PlatformByID(7)
	pb := b.World.PlatformByID(7)
	if pa.Center != pb.Center || pa.Angle != pb.Angle {
		t.Fatalf("platform state not restored: %v/%v vs %v/%v",
			pa.Center, pa.Angle, pb.Center, pb.Angle)
	}
	// Continue both: identical.
	for i := 0; i < 200; i++ {
		a.Step(scriptedInput(i))
		b.Step(scriptedInput(i))
		if a.Char.Pos != b.Char.Pos {
			t.Fatalf("step %d diverged", i)
		}
	}
}

func TestRestoreRejectsUnknownPlatform(t *testing.T) {
	cfg := DefaultConfig()
	a, err := NewSim(cfg, busyWorld(), NewCharacter(V(0, 0.5)))
	if err != nil {
		t.Fatal(err)
	}
	snap, err := a.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	// Restore into a world without the platform.
	b, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0.5)))
	if err != nil {
		t.Fatal(err)
	}
	if err := b.Restore(snap); err == nil {
		t.Fatalf("expected error restoring snapshot with missing platform")
	}
}
