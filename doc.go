// Package character implements a fixed-step character movement controller
// with deterministic, explainable motion.
//
// # Time model
//
// The simulation advances in exact fixed steps (Config.FixedDt). The
// controller exposes no delta-time argument: every call to Controller.Step
// advances the world by exactly one fixed step, so identical inputs on an
// identical world always produce an identical trajectory — regardless of
// frame rate, scheduling, or machine. The host game loop is expected to
// accumulate real time and call Step once per accumulated fixed step;
// replaying a recorded InputSequence yields bit-identical results.
//
// All arithmetic uses float64 (IEEE 754 round-to-nearest). No randomness,
// wall-clock reads, or map iteration order influence the result. Colliders
// and platforms are kept sorted by ID so resolution order is deterministic.
//
// # Coordinate system
//
// Units are world units. Positions denote the character's feet (bottom
// center). The character is an axis-aligned box of width
// 2*CharacterHalfWidth and height CharacterHeight.
//
// # Contact semantics
//
// The character is in one of three contact states:
//
//   - Grounded: supported by a walkable surface (flat ground or a slope at
//     or below MaxSlopeAngleDeg). Ground friction brakes motion when no
//     direction is held; the character sticks to slopes and follows moving
//     platforms.
//   - Sliding: in contact with a steeper slope. The character cannot jump,
//     has no ground friction, and accelerates downhill along the surface
//     tangent until reaching a walkable surface or the terminal fall speed.
//   - Airborne: no contact. Horizontal momentum is preserved; only held
//     directions add (weak) air control.
//
// Contact detection uses swept AABB tests against solid boxes and one-sided
// top-surface ramp segments. The SkinWidth parameter is the contact
// tolerance: surfaces within SkinWidth are treated as touching, and a small
// probe range above a surface keeps the character snapped to ground while
// moving. A hysteresis band of +/-0.5 degrees around the slope threshold
// prevents the contact state from flapping when the computed angle hovers
// near the boundary.
//
// # Slopes and steps
//
// Slopes are ramp segments. A slope whose angle from horizontal is at or
// below MaxSlopeAngleDeg is walkable; steeper slopes cause sliding. Walking
// onto a rising slope whose rise per step is within MaxStepHeight snaps the
// character up to the surface instead of penetrating.
//
// Steps are axis-aligned obstacles. When grounded and blocked by an obstacle
// whose top is within MaxStepHeight above the feet, the controller performs
// a lift-forward-place maneuver: it raises the character, moves it forward,
// and sets it down on the obstacle top, provided head clearance and support
// exist. Taller obstacles become walls; the character stops and slides along
// them smoothly rather than sticking.
//
// # Moving platforms
//
// Platforms are moving axis-aligned boxes. The host calls World.Step(dt)
// before Controller.Step so each platform records its previous transform.
// A grounded character standing on a platform is carried by the platform's
// full delta transform — translation plus rotation about the platform pivot
// — applied as a position change before physics. Because carry is a direct
// transform delta (not integrated velocity), the character is never flung
// by platform acceleration, turning, or disappearance. If a platform
// vanishes mid-step the character becomes airborne and preserves its
// momentum exactly. While on a platform, character input moves the character
// relative to the platform.
//
// # Jump and ground grace
//
// Jump presses are buffered for JumpBufferTime: a press shortly before
// landing fires on landing. After leaving a ledge the character remains
// jumpable for CoyoteTime. Both windows are configurable; setting either to
// zero disables the corresponding grace.
//
// # Serialization and replay
//
// State is a plain value struct (no maps or slices) and serializes to JSON
// via MarshalState/UnmarshalState. A checkpoint can be restored on a fresh
// controller and continued; the resulting trajectory is identical to a
// continuous run. Inputs can be recorded in an InputSequence for replay.
//
// # Configuration
//
// All parameters live in Config. DefaultConfig provides safe values.
// Config.Validate checks every parameter and returns all errors, each named
// with the offending parameter (e.g. "gravity: must be > 0"). Cross-parameter
// conflicts (such as a step taller than the character) are also rejected.
// NewController returns an error for any invalid configuration.
package character
