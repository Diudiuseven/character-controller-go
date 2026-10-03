package controller

import "fmt"

// Config holds every tunable parameter of the controller. All durations are
// in seconds, distances in world units, angles in degrees.
type Config struct {
	Dt                 float64 // fixed simulation step, seconds
	Gravity            float64 // downward acceleration magnitude (>0)
	MaxRunSpeed        float64 // horizontal speed cap produced by input
	GroundAccel        float64 // horizontal acceleration on ground
	AirAccel           float64 // horizontal acceleration in air
	GroundFriction     float64 // horizontal deceleration on ground with no input
	AirDrag            float64 // fractional horizontal drag per second in air [0,1)
	JumpSpeed          float64 // initial upward jump velocity
	TerminalVelocity   float64 // max downward fall speed (>0)
	MaxSlopeAngleDeg   float64 // steepest walkable slope angle
	SlopeHysteresisDeg float64 // hysteresis band around MaxSlopeAngleDeg
	MaxStepHeight      float64 // max ledge height the character can step up
	GroundSnapDistance float64 // max distance to snap down onto ground
	CoyoteTime         float64 // grace window to still jump after leaving ground
	JumpBufferTime     float64 // window to buffer a jump press before landing
	Width              float64 // character AABB width
	Height             float64 // character AABB height
}

// ConfigError identifies a single invalid parameter.
type ConfigError struct {
	Field string
	Msg   string
}

func (e *ConfigError) Error() string {
	return fmt.Sprintf("config: %s: %s", e.Field, e.Msg)
}

// DefaultConfig returns a sane, valid configuration.
func DefaultConfig() Config {
	return Config{
		Dt:                 1.0 / 60.0,
		Gravity:            30,
		MaxRunSpeed:        6,
		GroundAccel:        60,
		AirAccel:           30,
		GroundFriction:     50,
		AirDrag:            0.05,
		JumpSpeed:          12,
		TerminalVelocity:   40,
		MaxSlopeAngleDeg:   50,
		SlopeHysteresisDeg: 2,
		MaxStepHeight:      0.35,
		GroundSnapDistance: 0.08,
		CoyoteTime:         0.1,
		JumpBufferTime:     0.1,
		Width:              0.6,
		Height:             1.7,
	}
}

// Validate checks every parameter and rejects illegal or conflicting values.
// The returned error names the offending field.
func (c Config) Validate() error {
	bad := func(field, msg string) error { return &ConfigError{Field: field, Msg: msg} }

	if !(c.Dt > 0) || c.Dt > 0.1 {
		return bad("Dt", "must be in (0, 0.1] seconds")
	}
	if !(c.Gravity > 0) {
		return bad("Gravity", "must be > 0 (magnitude, applied downward)")
	}
	if !(c.MaxRunSpeed > 0) {
		return bad("MaxRunSpeed", "must be > 0")
	}
	if !(c.GroundAccel >= 0) {
		return bad("GroundAccel", "must be >= 0")
	}
	if !(c.AirAccel >= 0) {
		return bad("AirAccel", "must be >= 0")
	}
	if !(c.GroundFriction >= 0) {
		return bad("GroundFriction", "must be >= 0")
	}
	if !(c.AirDrag >= 0) || c.AirDrag >= 1 {
		return bad("AirDrag", "must be in [0, 1)")
	}
	if !(c.JumpSpeed > 0) {
		return bad("JumpSpeed", "must be > 0")
	}
	if !(c.TerminalVelocity > 0) {
		return bad("TerminalVelocity", "must be > 0")
	}
	if c.JumpSpeed > c.TerminalVelocity {
		return bad("JumpSpeed", "must not exceed TerminalVelocity")
	}
	if !(c.MaxSlopeAngleDeg > 0) || c.MaxSlopeAngleDeg >= 90 {
		return bad("MaxSlopeAngleDeg", "must be in (0, 90)")
	}
	if !(c.SlopeHysteresisDeg >= 0) {
		return bad("SlopeHysteresisDeg", "must be >= 0")
	}
	if c.SlopeHysteresisDeg >= c.MaxSlopeAngleDeg {
		return bad("SlopeHysteresisDeg", "must be smaller than MaxSlopeAngleDeg")
	}
	if c.MaxSlopeAngleDeg+c.SlopeHysteresisDeg >= 90 {
		return bad("SlopeHysteresisDeg", "MaxSlopeAngleDeg+SlopeHysteresisDeg must be < 90")
	}
	if !(c.MaxStepHeight >= 0) {
		return bad("MaxStepHeight", "must be >= 0")
	}
	if !(c.GroundSnapDistance >= 0) {
		return bad("GroundSnapDistance", "must be >= 0")
	}
	if !(c.CoyoteTime >= 0) {
		return bad("CoyoteTime", "must be >= 0")
	}
	if !(c.JumpBufferTime >= 0) {
		return bad("JumpBufferTime", "must be >= 0")
	}
	if !(c.Width > 0) {
		return bad("Width", "must be > 0")
	}
	if !(c.Height > 0) {
		return bad("Height", "must be > 0")
	}
	if c.MaxStepHeight >= c.Height {
		return bad("MaxStepHeight", "must be smaller than Height")
	}
	// Stability: the worst-case horizontal displacement per step must stay
	// below the step-up probe granularity so tunneling through thin
	// features cannot occur at supported speeds.
	if c.MaxRunSpeed*c.Dt > c.Width {
		return bad("MaxRunSpeed", "MaxRunSpeed*Dt must not exceed Width (tunneling guard)")
	}
	return nil
}
