package controller

import (
	"encoding/json"
	"errors"
	"fmt"
)

// Sim ties config, world and character together into a deterministic,
// fixed-step simulation. Calling Step with the same input sequence from the
// same state always produces bit-identical trajectories.
type Sim struct {
	Cfg       Config
	World     *World
	Char      *Character
	StepCount uint64
}

// NewSim validates the config and returns a ready simulation.
func NewSim(cfg Config, w *World, ch *Character) (*Sim, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	if w == nil {
		return nil, errors.New("sim: world must not be nil")
	}
	if ch == nil {
		return nil, errors.New("sim: character must not be nil")
	}
	return &Sim{Cfg: cfg, World: w, Char: ch}, nil
}

// Step advances the whole simulation by exactly one fixed step: platforms
// move first, then the character (so a standing character is carried by the
// platform's just-applied delta).
func (s *Sim) Step(in Input) {
	s.World.Step(s.Cfg.Dt)
	s.Char.Step(s.World, &s.Cfg, in)
	s.StepCount++
}

// snapshot is the serialized form of all dynamic state.
type snapshot struct {
	StepCount uint64         `json:"step_count"`
	Char      charSnapshot   `json:"char"`
	Platforms []platSnapshot `json:"platforms"`
}

type charSnapshot struct {
	Pos          Vec2      `json:"pos"`
	Vel          Vec2      `json:"vel"`
	State        CharState `json:"state"`
	GroundNormal Vec2      `json:"ground_normal"`
	GroundPlatID int       `json:"ground_plat_id"`
	CoyoteTimer  float64   `json:"coyote_timer"`
	JumpBufTimer float64   `json:"jump_buf_timer"`
	SlideLatched bool      `json:"slide_latched"`
}

type platSnapshot struct {
	ID     int     `json:"id"`
	Center Vec2    `json:"center"`
	Angle  float64 `json:"angle"`
}

// Snapshot serializes the full dynamic state (character, platforms, step
// counter) so the simulation can be resumed bit-exactly later.
func (s *Sim) Snapshot() ([]byte, error) {
	snap := snapshot{
		StepCount: s.StepCount,
		Char: charSnapshot{
			Pos:          s.Char.Pos,
			Vel:          s.Char.Vel,
			State:        s.Char.State,
			GroundNormal: s.Char.GroundNormal,
			GroundPlatID: s.Char.GroundPlatID,
			CoyoteTimer:  s.Char.coyoteTimer,
			JumpBufTimer: s.Char.jumpBufTimer,
			SlideLatched: s.Char.slideLatched,
		},
	}
	for _, p := range s.World.Platforms {
		snap.Platforms = append(snap.Platforms, platSnapshot{
			ID:     p.ID,
			Center: p.Center,
			Angle:  p.Angle,
		})
	}
	return json.Marshal(snap)
}

// Restore loads a snapshot produced by Snapshot. The world geometry (boxes,
// slopes, platform shapes and ids) must match the snapshot's sim.
func (s *Sim) Restore(data []byte) error {
	var snap snapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return fmt.Errorf("sim: restore: %w", err)
	}
	s.StepCount = snap.StepCount
	c := s.Char
	c.Pos = snap.Char.Pos
	c.Vel = snap.Char.Vel
	c.State = snap.Char.State
	c.GroundNormal = snap.Char.GroundNormal
	c.GroundPlatID = snap.Char.GroundPlatID
	c.coyoteTimer = snap.Char.CoyoteTimer
	c.jumpBufTimer = snap.Char.JumpBufTimer
	c.slideLatched = snap.Char.SlideLatched
	for _, ps := range snap.Platforms {
		p := s.World.PlatformByID(ps.ID)
		if p == nil {
			return fmt.Errorf("sim: restore: platform id %d not found", ps.ID)
		}
		p.Center = ps.Center
		p.Angle = ps.Angle
	}
	return nil
}
