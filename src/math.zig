//! Handles: how callers name what the world owns.
pub const emath = @import("eggenvector");

/// A handle to a body or collider: its slot and the generation it was added in, so a handle to
/// a removed one never matches whatever reuses its slot. Each `of` makes a distinct type, so a
/// body handle can't be passed as a collider's. Packed so handles compare with `==`.
pub fn Handle(comptime of: enum { body, collider }) type {
    return packed struct(u64) {
        index: u32,
        generation: u32,

        pub const target = of;
    };
}
