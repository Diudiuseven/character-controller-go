package character

import (
	"errors"
	"testing"
)

// TestConfigValidDefaults: the default configuration validates.
func TestConfigValidDefaults(t *testing.T) {
	cfg := DefaultConfig()
	if err := cfg.Validate(); err != nil {
		t.Fatalf("default config should be valid: %v", err)
	}
}

// TestConfigInvalidParams: each invalid parameter is rejected and the error
// names the offending field.
func TestConfigInvalidParams(t *testing.T) {
	cases := []struct {
		name string
		mod  func(*Config)
	}{
		{"FixedDt zero", func(c *Config) { c.FixedDt = 0 }},
		{"FixedDt too large", func(c *Config) { c.FixedDt = 0.1 }},
		{"Gravity zero", func(c *Config) { c.Gravity = 0 }},
		{"Gravity negative", func(c *Config) { c.Gravity = -1 }},
		{"MaxFallSpeed zero", func(c *Config) { c.MaxFallSpeed = 0 }},
		{"MaxMoveSpeed zero", func(c *Config) { c.MaxMoveSpeed = 0 }},
		{"GroundAccel negative", func(c *Config) { c.GroundAccel = -1 }},
		{"AirAccel negative", func(c *Config) { c.AirAccel = -1 }},
		{"GroundFriction negative", func(c *Config) { c.GroundFriction = -1 }},
		{"AirDrag negative", func(c *Config) { c.AirDrag = -1 }},
		{"JumpSpeed zero", func(c *Config) { c.JumpSpeed = 0 }},
		{"MaxSlopeAngleDeg negative", func(c *Config) { c.MaxSlopeAngleDeg = -1 }},
		{"MaxSlopeAngleDeg too high", func(c *Config) { c.MaxSlopeAngleDeg = 90 }},
		{"MaxStepHeight negative", func(c *Config) { c.MaxStepHeight = -0.1 }},
		{"SkinWidth zero", func(c *Config) { c.SkinWidth = 0 }},
		{"CoyoteTime negative", func(c *Config) { c.CoyoteTime = -0.1 }},
		{"JumpBufferTime negative", func(c *Config) { c.JumpBufferTime = -0.1 }},
		{"CharacterHalfWidth zero", func(c *Config) { c.CharacterHalfWidth = 0 }},
		{"CharacterHeight too small", func(c *Config) { c.CharacterHeight = 0.1 }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cfg := DefaultConfig()
			tc.mod(&cfg)
			err := cfg.Validate()
			if err == nil {
				t.Fatalf("expected validation error for %s", tc.name)
			}
			var ce *ConfigError
			if !errors.As(err, &ce) {
				t.Fatalf("expected ConfigError, got %T: %v", err, err)
			}
		})
	}
}

// TestConfigCrossParam: conflicting cross-parameter configurations are rejected.
func TestConfigCrossParam(t *testing.T) {
	cfg := DefaultConfig()
	cfg.MaxStepHeight = cfg.CharacterHeight // step taller than character
	if err := cfg.Validate(); err == nil {
		t.Fatalf("expected rejection when MaxStepHeight >= CharacterHeight")
	}
}

// TestConfigNewControllerRejects: NewController returns an error for invalid config.
func TestConfigNewControllerRejects(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Gravity = -5
	if _, err := NewController(cfg); err == nil {
		t.Fatalf("NewController should reject invalid config")
	}
}

// TestConfigErrorsNameParam: joined errors name each offending parameter.
func TestConfigErrorsNameParam(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Gravity = 0
	cfg.JumpSpeed = 0
	err := cfg.Validate()
	if err == nil {
		t.Fatal("expected errors")
	}
	msg := err.Error()
	if !contains(msg, "Gravity") || !contains(msg, "JumpSpeed") {
		t.Fatalf("errors should name parameters, got: %s", msg)
	}
}

func contains(s, sub string) bool {
	return len(s) >= len(sub) && (s == sub || indexOf(s, sub) >= 0)
}

func indexOf(s, sub string) int {
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return i
		}
	}
	return -1
}
