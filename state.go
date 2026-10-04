package character

import "encoding/json"

// GroundState describes the character's contact state.
type GroundState int

const (
	// Airborne: no contact with walkable ground.
	Airborne GroundState = iota
	// Grounded: supported by a walkable surface (flat or slope <= threshold).
	Grounded
	// Sliding: in contact with a slope steeper than the walkable threshold.
	Sliding
)

// State is the complete, serializable character simulation state.
// All fields are value types; no maps or slices, so serialization is stable.
type State struct {
	Pos Vec2 `json:"pos"`
	Vel Vec2 `json:"vel"`

	// Ground is the current contact state.
	Ground GroundState `json:"ground"`
	// GroundNormal is the surface normal of the current/last contact.
	GroundNormal Vec2 `json:"ground_normal"`
	// SlopeAngleDeg is the angle between the ground normal and the up axis.
	SlopeAngleDeg float64 `json:"slope_angle_deg"`

	// PlatformID is the platform currently being carried (-1 if none).
	PlatformID int `json:"platform_id"`

	// CoyoteTimer counts down grace time after leaving a ledge.
	CoyoteTimer float64 `json:"coyote_timer"`
	// JumpBufferTimer counts down buffered jump presses.
	JumpBufferTimer float64 `json:"jump_buffer_timer"`

	// StepCount is the number of fixed steps simulated (for replay alignment).
	StepCount int64 `json:"step_count"`
}

// Input is the per-step player input.
type Input struct {
	// MoveX is the horizontal input in [-1, 1].
	MoveX float64 `json:"move_x"`
	// JumpPressed is true on the step the jump button is first pressed.
	JumpPressed bool `json:"jump_pressed"`
}

// DefaultState returns a state with no velocity and no contact.
func DefaultState() State {
	return State{
		Ground:       Airborne,
		GroundNormal: Vec2{X: 0, Y: 1},
		PlatformID:   -1,
	}
}

// Clone returns a deep copy of the state.
func (s State) Clone() State { return s }

// MarshalState serializes the state to JSON.
func MarshalState(s State) ([]byte, error) {
	return json.Marshal(s)
}

// UnmarshalState restores a state from JSON produced by MarshalState.
func UnmarshalState(data []byte) (State, error) {
	var s State
	if err := json.Unmarshal(data, &s); err != nil {
		return State{}, err
	}
	if s.PlatformID == 0 && s.Ground == Airborne && s.Pos == (Vec2{}) {
		// leave as-is; zero values are valid defaults
	}
	return s, nil
}

// RecordedStep pairs an input with the step index it applies to.
type RecordedStep struct {
	Step  int64 `json:"step"`
	Input Input `json:"input"`
}

// InputSequence is a replay recording: the exact inputs for each fixed step.
type InputSequence struct {
	Steps []RecordedStep `json:"steps"`
}

// Record appends an input for the given step index.
func (seq *InputSequence) Record(step int64, in Input) {
	seq.Steps = append(seq.Steps, RecordedStep{Step: step, Input: in})
}

// MarshalSequence serializes the input sequence.
func MarshalSequence(seq InputSequence) ([]byte, error) {
	return json.Marshal(seq)
}

// UnmarshalSequence restores an input sequence.
func UnmarshalSequence(data []byte) (InputSequence, error) {
	var seq InputSequence
	err := json.Unmarshal(data, &seq)
	return seq, err
}

// InputAt returns the input recorded for step index n, or a zero Input.
func (seq InputSequence) InputAt(n int64) Input {
	// Linear search is fine for test-sized recordings; steps are appended in order.
	for i := range seq.Steps {
		if seq.Steps[i].Step == n {
			return seq.Steps[i].Input
		}
	}
	return Input{}
}
