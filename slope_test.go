package character

import (
	"math"
	"testing"
)

func degToRad(d float64) float64 { return d * math.Pi / 180 }

// rampWorld: a 30-degree climbable ramp followed by a 60-degree slide.
func slopeWorld() *World {
	w := NewWorld()
	w.AddBox(0, Vec2{-4, 0}, Vec2{0, 1}, -1)
	w.AddRamp(1, Vec2{0, 1}, Vec2{6, 1 + 6*math.Tan(degToRad(30))})
	w.AddRamp(2, Vec2{6, 1 + 6*math.Tan(degToRad(30))}, Vec2{9, 1 + 6*math.Tan(degToRad(30)) - 3*math.Tan(degToRad(60))})
	w.AddBox(3, Vec2{9, 0}, Vec2{20, 1}, -1)
	return w
}

// TestSlopeWalkUp: character climbs a walkable slope and stays grounded.
func TestSlopeWalkUp(t *testing.T) {
	w := slopeWorld()
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{-2, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 30, Input{MoveX: 1})
	s := c.State()
	if s.Ground != Grounded {
		t.Fatalf("expected grounded on climbable slope, got %d", s.Ground)
	}
	if s.Pos.Y < 2.0 {
		t.Fatalf("expected to climb above y=2.0, got %.2f", s.Pos.Y)
	}
}

// TestSlopeCriticalAngleNoFlap: at the walkable threshold the contact state
// must not flap between grounded and sliding.
func TestSlopeCriticalAngleNoFlap(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{-4, 0}, Vec2{0, 1}, -1)
	// Ramp at exactly the default threshold (45 degrees).
	w.AddRamp(1, Vec2{0, 1}, Vec2{4, 5})
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{-2, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	prev := Grounded
	flaps := 0
	for i := 0; i < 120; i++ {
		c.Step(w, Input{MoveX: 1})
		s := c.State()
		if s.Ground != prev {
			flaps++
		}
		prev = s.Ground
	}
	if flaps > 2 {
		t.Fatalf("contact state flapped %d times at critical angle", flaps)
	}
}

// TestSlopeSteepSlides: on a slope steeper than the threshold the character
// enters the Sliding state and accelerates downhill.
func TestSlopeSteepSlides(t *testing.T) {
	w := NewWorld()
	w.AddRamp(1, Vec2{0, 6}, Vec2{4, 0}) // ~56 degrees
	w.AddBox(0, Vec2{-4, 0}, Vec2{20, 0.01}, -1)
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{2, 3}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	slid := false
	for i := 0; i < 120; i++ {
		c.Step(w, Input{})
		if c.State().Ground == Sliding {
			slid = true
		}
	}
	s := c.State()
	if !slid {
		t.Fatalf("expected sliding on steep slope")
	}
	if s.Pos.X <= 2 {
		t.Fatalf("expected to slide downhill toward +x, got x=%.2f", s.Pos.X)
	}
}

// TestSlopeSlideStopsAtBottom: sliding onto flat ground transitions to grounded.
func TestSlopeSlideStopsAtBottom(t *testing.T) {
	w := NewWorld()
	w.AddRamp(1, Vec2{0, 6}, Vec2{4, 0})
	w.AddBox(0, Vec2{-4, 0}, Vec2{20, 0.01}, -1)
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{2, 3}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 200, Input{})
	s := c.State()
	if s.Ground != Grounded {
		t.Fatalf("expected grounded at bottom, got %d", s.Ground)
	}
	if s.Pos.Y != 0.01 {
		t.Fatalf("expected to rest on ground y=0.01, got %.4f", s.Pos.Y)
	}
}

// TestStepUpMaxHeight: a step at the configured maximum is climbed.
func TestStepUpMaxHeight(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{2, 1}, Vec2{20, 1.6}, -1) // exactly 0.6
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{1, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 60, Input{MoveX: 1})
	s := c.State()
	if s.Pos.Y < 1.55 {
		t.Fatalf("expected to step up to y≈1.6, got %.2f", s.Pos.Y)
	}
}

// TestStepAboveMaxHeight: a step taller than the maximum becomes a wall.
func TestStepAboveMaxHeight(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{2, 1}, Vec2{3, 1.7}, -1) // 0.7 > 0.6
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{1, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 60, Input{MoveX: 1})
	s := c.State()
	if s.Pos.Y > 1.5 {
		t.Fatalf("should not step up 0.7, got y=%.2f", s.Pos.Y)
	}
	if s.Pos.X > 2.5 {
		t.Fatalf("should be blocked by wall, got x=%.2f", s.Pos.X)
	}
}

// TestStepConfigurable: raising MaxStepHeight allows climbing taller steps.
func TestStepConfigurable(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{2, 1}, Vec2{20, 2.1}, -1) // 1.1 step
	cfg := DefaultConfig()
	cfg.MaxStepHeight = 1.2
	c, err := NewController(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.SetState(State{Pos: Vec2{1, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	runSteps(c, w, 60, Input{MoveX: 1})
	s := c.State()
	if s.Pos.Y < 2.0 {
		t.Fatalf("expected to step up 1.1 with MaxStepHeight=1.2, got y=%.2f", s.Pos.Y)
	}
}

// TestNoPenetration: character never ends a step inside solid geometry.
func TestNoPenetration(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{5, 1}, Vec2{6, 5}, -1) // wall
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	for i := 0; i < 200; i++ {
		c.Step(w, Input{MoveX: 1})
		s := c.State()
		for _, b := range w.BoxColliders() {
			if charOverlapsBox(s.Pos, c.Config().CharacterHalfWidth, c.Config().CharacterHeight, b) {
				t.Fatalf("step %d: character overlaps box %d at pos=%v", i, b.ID, s.Pos)
			}
		}
	}
}

// TestWallSlide: falling along a wall keeps contact and slides smoothly.
func TestWallSlide(t *testing.T) {
	w := NewWorld()
	w.AddBox(0, Vec2{0, 0}, Vec2{30, 1}, -1)
	w.AddBox(1, Vec2{5, 1}, Vec2{6, 8}, -1) // tall wall
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{4.4, 8}, Vel: Vec2{2, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	prevY := 8.0
	slid := false
	for i := 0; i < 120; i++ {
		c.Step(w, Input{MoveX: 1})
		s := c.State()
		if s.Pos.Y < prevY-0.01 && s.Pos.X >= 4.9 {
			slid = true
		}
		prevY = s.Pos.Y
	}
	if !slid {
		t.Fatalf("expected to slide down along the wall")
	}
}

// TestNoJitterOnGround: standing still on flat ground produces no position jitter.
func TestNoJitterOnGround(t *testing.T) {
	w := testWorld()
	c, _ := NewController(DefaultConfig())
	c.SetState(State{Pos: Vec2{0, 1}, Vel: Vec2{0, 0}, Ground: Grounded, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	ys := map[float64]int{}
	for i := 0; i < 120; i++ {
		c.Step(w, Input{})
		ys[c.State().Pos.Y]++
	}
	if len(ys) != 1 {
		t.Fatalf("expected no vertical jitter, got %d distinct y values", len(ys))
	}
}

// TestSlopeCriticalLanding: landing on a slope exactly at the threshold
// commits to grounded (no airborne hover / jitter).
func TestSlopeCriticalLanding(t *testing.T) {
	w := NewWorld()
	w.AddRamp(1, Vec2{0, 0}, Vec2{4, 4}) // 45 degrees
	cfg := DefaultConfig()
	c, _ := NewController(cfg)
	c.SetState(State{Pos: Vec2{2, 5}, Vel: Vec2{0, 0}, Ground: Airborne, PlatformID: -1, GroundNormal: Vec2{0, 1}})
	grounded := false
	for i := 0; i < 120; i++ {
		c.Step(w, Input{})
		if c.State().Ground == Grounded {
			grounded = true
		}
	}
	if !grounded {
		t.Fatalf("expected to land on 45-degree slope, final state=%d", c.State().Ground)
	}
}
