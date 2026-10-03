package controller

import (
	"strings"
	"testing"
)

func TestDefaultConfigValid(t *testing.T) {
	if err := DefaultConfig().Validate(); err != nil {
		t.Fatalf("default config invalid: %v", err)
	}
}

func TestConfigValidationNamesField(t *testing.T) {
	good := DefaultConfig()
	cases := []struct {
		name  string
		mut   func(*Config)
		field string
	}{
		{"zero dt", func(c *Config) { c.Dt = 0 }, "Dt"},
		{"huge dt", func(c *Config) { c.Dt = 0.5 }, "Dt"},
		{"negative gravity", func(c *Config) { c.Gravity = -1 }, "Gravity"},
		{"zero run speed", func(c *Config) { c.MaxRunSpeed = 0 }, "MaxRunSpeed"},
		{"negative friction", func(c *Config) { c.GroundFriction = -1 }, "GroundFriction"},
		{"drag too large", func(c *Config) { c.AirDrag = 1 }, "AirDrag"},
		{"jump exceeds terminal", func(c *Config) { c.JumpSpeed = 100 }, "JumpSpeed"},
		{"slope angle zero", func(c *Config) { c.MaxSlopeAngleDeg = 0 }, "MaxSlopeAngleDeg"},
		{"slope angle 90", func(c *Config) { c.MaxSlopeAngleDeg = 90 }, "MaxSlopeAngleDeg"},
		{"hysteresis exceeds slope", func(c *Config) { c.SlopeHysteresisDeg = 60 }, "SlopeHysteresisDeg"},
		{"negative step", func(c *Config) { c.MaxStepHeight = -0.1 }, "MaxStepHeight"},
		{"step taller than character", func(c *Config) { c.MaxStepHeight = 2 }, "MaxStepHeight"},
		{"negative coyote", func(c *Config) { c.CoyoteTime = -1 }, "CoyoteTime"},
		{"zero width", func(c *Config) { c.Width = 0 }, "Width"},
		{"zero height", func(c *Config) { c.Height = 0 }, "Height"},
		{"tunneling speed", func(c *Config) { c.MaxRunSpeed = 100 }, "MaxRunSpeed"},
	}
	for _, tc := range cases {
		c := good
		tc.mut(&c)
		err := c.Validate()
		if err == nil {
			t.Errorf("%s: expected error", tc.name)
			continue
		}
		ce, ok := err.(*ConfigError)
		if !ok {
			t.Errorf("%s: error is not *ConfigError: %T", tc.name, err)
			continue
		}
		if ce.Field != tc.field {
			t.Errorf("%s: error names field %q, want %q", tc.name, ce.Field, tc.field)
		}
		if !strings.Contains(err.Error(), tc.field) {
			t.Errorf("%s: error message %q does not name field", tc.name, err.Error())
		}
	}
}

func TestNewSimRejectsInvalidConfig(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Gravity = -5
	if _, err := NewSim(cfg, flatWorld(), NewCharacter(V(0, 0))); err == nil {
		t.Fatalf("expected NewSim to reject invalid config")
	}
	if _, err := NewSim(DefaultConfig(), nil, NewCharacter(V(0, 0))); err == nil {
		t.Fatalf("expected NewSim to reject nil world")
	}
}
