package character

import (
	"errors"
	"fmt"
)

// Config holds all tunable parameters of the controller.
// Use DefaultConfig for a safe starting point and Validate before use.
type Config struct {
	// FixedDt is the fixed simulation timestep in seconds. The controller
	// always advances by exactly this amount; frame rate never affects motion.
	FixedDt float64

	// Gravity is the downward acceleration in units/second^2 (positive).
	Gravity float64
	// MaxFallSpeed is the terminal downward velocity in units/second.
	MaxFallSpeed float64
	// MaxMoveSpeed is the target horizontal ground speed in units/second.
	MaxMoveSpeed float64

	// GroundAccel is the lerp rate toward target speed while grounded (1/s).
	GroundAccel float64
	// AirAccel is the lerp rate toward target speed while airborne (1/s).
	AirAccel float64
	// GroundFriction is the extra braking rate when grounded with no input (1/s).
	GroundFriction float64
	// AirDrag is the exponential horizontal damping rate applied in all states (1/s).
	AirDrag float64

	// JumpSpeed is the initial upward velocity granted on jump (units/s).
	JumpSpeed float64

	// MaxSlopeAngleDeg is the steepest slope (degrees from horizontal) the
	// character can stand and walk on. Steeper slopes cause sliding.
	MaxSlopeAngleDeg float64
	// MaxStepHeight is the tallest obstacle (units) that will be automatically
	// stepped over instead of becoming a wall.
	MaxStepHeight float64

	// SkinWidth is the collision skin / contact tolerance in world units.
	SkinWidth float64
	// CoyoteTime is the grace window (seconds) after leaving a ledge during
	// which jumping is still allowed.
	CoyoteTime float64
	// JumpBufferTime is the window (seconds) a jump press is buffered before landing.
	JumpBufferTime float64

	// CharacterHalfWidth is half the character box width.
	CharacterHalfWidth float64
	// CharacterHeight is the character box height.
	CharacterHeight float64
}

// DefaultConfig returns a configuration with safe, tested values.
func DefaultConfig() Config {
	return Config{
		FixedDt:            1.0 / 60.0,
		Gravity:            28.0,
		MaxFallSpeed:       32.0,
		MaxMoveSpeed:       8.0,
		GroundAccel:        80.0,
		AirAccel:           30.0,
		GroundFriction:     10.0,
		AirDrag:            0.5,
		JumpSpeed:          11.0,
		MaxSlopeAngleDeg:   45.0,
		MaxStepHeight:      0.6,
		SkinWidth:          0.05,
		CoyoteTime:         0.10,
		JumpBufferTime:     0.10,
		CharacterHalfWidth: 0.3,
		CharacterHeight:    1.8,
	}
}

// ConfigError is a parameter validation error that names the offending field.
type ConfigError struct {
	Param   string
	Message string
}

func (e *ConfigError) Error() string {
	return fmt.Sprintf("config parameter %q: %s", e.Param, e.Message)
}

func cfgErr(param, msg string) error {
	return &ConfigError{Param: param, Message: msg}
}

// Validate checks every parameter and returns all errors found, each naming
// the offending parameter. It never panics.
func (c *Config) Validate() error {
	var errs []error

	if !(c.FixedDt > 0) {
		errs = append(errs, cfgErr("FixedDt", "must be > 0"))
	} else if c.FixedDt > 1.0/30.0 {
		errs = append(errs, cfgErr("FixedDt", "must be <= 1/30 s (30 Hz); larger steps risk tunneling and unstable contact"))
	}
	if !(c.Gravity > 0) {
		errs = append(errs, cfgErr("Gravity", "must be > 0"))
	}
	if !(c.MaxFallSpeed > 0) {
		errs = append(errs, cfgErr("MaxFallSpeed", "must be > 0"))
	}
	if !(c.MaxMoveSpeed > 0) {
		errs = append(errs, cfgErr("MaxMoveSpeed", "must be > 0"))
	}
	if c.GroundAccel < 0 {
		errs = append(errs, cfgErr("GroundAccel", "must be >= 0"))
	}
	if c.AirAccel < 0 {
		errs = append(errs, cfgErr("AirAccel", "must be >= 0"))
	}
	if c.GroundFriction < 0 {
		errs = append(errs, cfgErr("GroundFriction", "must be >= 0"))
	}
	if c.AirDrag < 0 {
		errs = append(errs, cfgErr("AirDrag", "must be >= 0"))
	}
	if !(c.JumpSpeed > 0) {
		errs = append(errs, cfgErr("JumpSpeed", "must be > 0"))
	}
	if c.MaxSlopeAngleDeg < 0 || c.MaxSlopeAngleDeg > 89 {
		errs = append(errs, cfgErr("MaxSlopeAngleDeg", "must be in [0, 89]"))
	}
	if c.MaxStepHeight < 0 {
		errs = append(errs, cfgErr("MaxStepHeight", "must be >= 0"))
	}
	if !(c.SkinWidth > 0) {
		errs = append(errs, cfgErr("SkinWidth", "must be > 0"))
	}
	if c.CoyoteTime < 0 {
		errs = append(errs, cfgErr("CoyoteTime", "must be >= 0"))
	}
	if c.JumpBufferTime < 0 {
		errs = append(errs, cfgErr("JumpBufferTime", "must be >= 0"))
	}
	if !(c.CharacterHalfWidth > 0) {
		errs = append(errs, cfgErr("CharacterHalfWidth", "must be > 0"))
	}
	if !(c.CharacterHeight > c.CharacterHalfWidth) {
		errs = append(errs, cfgErr("CharacterHeight", "must be > CharacterHalfWidth"))
	}

	// Cross-parameter checks: the step-up maneuver must have head clearance.
	if c.CharacterHeight > c.CharacterHalfWidth && c.MaxStepHeight > 0 {
		if c.MaxStepHeight+c.SkinWidth >= c.CharacterHeight {
			errs = append(errs, cfgErr("MaxStepHeight",
				"must satisfy MaxStepHeight + SkinWidth < CharacterHeight so the character can stand on top of stepped obstacles"))
		}
	}
	// Terminal fall speed must be large enough to matter and small enough that
	// swept collision never tunnels through the skin in one step.
	if c.FixedDt > 0 && c.MaxFallSpeed > 0 {
		if c.MaxFallSpeed*c.FixedDt > c.CharacterHeight {
			errs = append(errs, cfgErr("MaxFallSpeed",
				"MaxFallSpeed*FixedDt must not exceed CharacterHeight to avoid tunneling"))
		}
	}
	// Jump must be reachable-ish relative to gravity (informational range).
	if c.Gravity > 0 && c.JumpSpeed > 0 {
		if c.JumpSpeed*c.JumpSpeed/(2*c.Gravity) < c.SkinWidth {
			errs = append(errs, cfgErr("JumpSpeed",
				"jump apex is below SkinWidth; jump would be unnoticeable"))
		}
	}

	return errors.Join(errs...)
}
