package controller

import (
	"math"
	"testing"
)

// platformWorld: static floor far below, one moving platform.
func platformWorld(plat *Platform) *World {
	return &World{
		Boxes:     []Box{{Min: V(-500, -51), Max: V(500, -50)}},
		Platforms: []*Platform{plat},
	}
}

func dropOntoPlatform(t *testing.T, cfg Config, w *World, x float64) *Sim {
	t.Helper()
	s, err := NewSim(cfg, w, NewCharacter(V(x, 3)))
	if err != nil {
		t.Fatal(err)
	}
	// Fall until standing on the platform.
	for i := 0; i < 600; i++ {
		s.Step(Input{})
		if s.Char.State == StateGrounded && s.Char.GroundPlatID >= 0 {
			return s
		}
	}
	t.Fatalf("never landed on platform: state=%v pos=%v", s.Char.State, s.Char.Pos)
	return nil
}

func TestPlatformCarriesCharacter(t *testing.T) {
	cfg := DefaultConfig()
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(3, 0.5), Vel: V(2, 0)}
	s := dropOntoPlatform(t, cfg, platformWorld(p), 0)
	startX := s.Char.Pos.X
	startPlatX := p.Center.X
	runSteps(s, 60, Input{})
	// Character must move exactly with the platform.
	dxChar := s.Char.Pos.X - startX
	dxPlat := p.Center.X - startPlatX
	if math.Abs(dxChar-dxPlat) > 1e-9 {
		t.Fatalf("carry mismatch: char moved %v, platform moved %v", dxChar, dxPlat)
	}
	if s.Char.State != StateGrounded || s.Char.GroundPlatID != 1 {
		t.Fatalf("lost platform contact: state=%v plat=%d", s.Char.State, s.Char.GroundPlatID)
	}
	// No extra displacement: feet stay on the platform surface.
	if s.Char.Pos.Y != p.Center.Y+p.Half.Y {
		t.Fatalf("feet left platform surface: y=%v top=%v", s.Char.Pos.Y, p.Center.Y+p.Half.Y)
	}
}

func TestPlatformVerticalCarry(t *testing.T) {
	cfg := DefaultConfig()
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(3, 0.5), Vel: V(0, 1.5)}
	s := dropOntoPlatform(t, cfg, platformWorld(p), 0)
	y0 := s.Char.Pos.Y
	runSteps(s, 60, Input{})
	want := y0 + 1.5*60*cfg.Dt
	if math.Abs(s.Char.Pos.Y-want) > 1e-9 {
		t.Fatalf("vertical carry mismatch: got %v want %v", s.Char.Pos.Y, want)
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("lost contact on rising platform")
	}
}

func TestPlatformRotationCarriesCharacter(t *testing.T) {
	cfg := DefaultConfig()
	// Slow rotation, small radius: the character is carried by the rigid
	// rotation, then constrained back onto the platform's top surface
	// (the collision shape is an AABB; see README contact semantics).
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(5, 0.5), AngularVel: 0.2}
	s := dropOntoPlatform(t, cfg, platformWorld(p), 2) // stand off-center
	prevX := s.Char.Pos.X
	for i := 0; i < 120; i++ {
		s.Step(Input{})
		if s.Char.State != StateGrounded || s.Char.GroundPlatID != 1 {
			t.Fatalf("step %d: lost footing on rotating platform", i)
		}
		if s.Char.Pos.Y != p.Center.Y+p.Half.Y {
			t.Fatalf("step %d: left platform surface, y=%v", i, s.Char.Pos.Y)
		}
		// CCW rotation carries a point at +x toward the center (x shrinks).
		if s.Char.Pos.X >= prevX {
			t.Fatalf("step %d: rotation did not carry character inward", i)
		}
		prevX = s.Char.Pos.X
	}
}

func TestPlatformSpeedChangeNoFling(t *testing.T) {
	cfg := DefaultConfig()
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(30, 0.5), Vel: V(5, 0)}
	s := dropOntoPlatform(t, cfg, platformWorld(p), 0)
	runSteps(s, 30, Input{})
	// Sudden stop: character keeps only its own velocity (zero), no fling.
	p.Vel = V(0, 0)
	runSteps(s, 30, Input{})
	if math.Abs(s.Char.Vel.X) > 1e-9 {
		t.Fatalf("platform stop flung character: vx=%v", s.Char.Vel.X)
	}
	// Sudden reversal.
	p.Vel = V(-5, 0)
	x0 := s.Char.Pos.X
	runSteps(s, 30, Input{})
	if s.Char.Pos.X >= x0 {
		t.Fatalf("character not carried by reversed platform")
	}
	if s.Char.State != StateGrounded {
		t.Fatalf("lost contact after reversal")
	}
}

func TestPlatformVanishDropsCharacter(t *testing.T) {
	cfg := DefaultConfig()
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(3, 0.5), Vel: V(4, 2)}
	w := platformWorld(p)
	s := dropOntoPlatform(t, cfg, w, 0)
	runSteps(s, 30, Input{})
	// Remove the platform: character must fall straight from its current
	// spot with no inherited platform velocity.
	w.Platforms = nil
	vxBefore := s.Char.Vel.X
	s.Step(Input{})
	if s.Char.State == StateGrounded && s.Char.GroundPlatID >= 0 {
		t.Fatalf("still attached to vanished platform")
	}
	if s.Char.Vel.X != vxBefore {
		t.Fatalf("vanishing platform injected velocity: vx %v -> %v", vxBefore, s.Char.Vel.X)
	}
	// Falls to the lower floor and lands.
	for i := 0; i < 600 && s.Char.State != StateGrounded; i++ {
		s.Step(Input{})
	}
	if s.Char.Pos.Y != -50 {
		t.Fatalf("expected to land on lower floor, got y=%v", s.Char.Pos.Y)
	}
}

func TestWalkOffPlatformEdge(t *testing.T) {
	cfg := DefaultConfig()
	p := &Platform{ID: 1, Center: V(0, 0), Half: V(2, 0.5)}
	s := dropOntoPlatform(t, cfg, platformWorld(p), 0)
	// Walk right off the platform edge; must fall and land on lower floor.
	for i := 0; i < 600; i++ {
		s.Step(Input{Move: 1})
		if s.Char.State == StateGrounded && s.Char.GroundPlatID == -1 {
			break
		}
	}
	if s.Char.Pos.Y != -50 {
		t.Fatalf("expected to land on lower floor, got y=%v", s.Char.Pos.Y)
	}
}
