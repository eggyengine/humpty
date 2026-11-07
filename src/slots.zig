//! Slot storage behind generational handles: removing frees a slot for reuse, and bumps its
//! generation so old handles to it go stale instead of naming whatever moves in.
const std = @import("std");

/// Storage of `Item`s named by `Handle` (a `math.Handle`).
pub fn Slots(comptime Item: type, comptime Handle: type) type {
    return struct {
        entries: std.ArrayList(Entry) = .empty,
        free: std.ArrayList(u32) = .empty,

        const Self = @This();

        pub const Entry = struct {
            generation: u32 = 0,
            item: ?Item = null,
        };

        pub fn deinit(self: *Self, gpa: std.mem.Allocator) void {
            self.entries.deinit(gpa);
            self.free.deinit(gpa);
        }

        pub fn add(self: *Self, gpa: std.mem.Allocator, item: Item) !Handle {
            // Room for every slot on the free list, so `remove` never allocates.
            try self.free.ensureTotalCapacity(gpa, self.entries.items.len + 1);
            if (self.free.pop()) |index| {
                const entry = &self.entries.items[index];
                entry.item = item;
                return .{ .index = index, .generation = entry.generation };
            }
            const index: u32 = @intCast(self.entries.items.len);
            try self.entries.append(gpa, .{ .item = item });
            return .{ .index = index, .generation = 0 };
        }

        /// Frees `handle`'s slot. Returns false when it was already stale.
        pub fn remove(self: *Self, handle: Handle) bool {
            const entry = self.entryOf(handle) orelse return false;
            entry.item = null;
            entry.generation +%= 1;
            self.free.appendAssumeCapacity(handle.index);
            return true;
        }

        /// The item `handle` names, or null when it is stale. Valid until the next `add`.
        pub fn get(self: *const Self, handle: Handle) ?*Item {
            const entry = self.entryOf(handle) orelse return null;
            return &entry.item.?;
        }

        /// The handle naming the live item in slot `index`.
        pub fn handleAt(self: *const Self, index: usize) Handle {
            return .{ .index = @intCast(index), .generation = self.entries.items[index].generation };
        }

        fn entryOf(self: *const Self, handle: Handle) ?*Entry {
            if (handle.index >= self.entries.items.len) return null;
            const entry = &self.entries.items[handle.index];
            if (entry.generation != handle.generation or entry.item == null) return null;
            return entry;
        }
    };
}

test "a removed slot's handle goes stale when the slot is reused" {
    const Handle = packed struct(u64) { index: u32, generation: u32 };
    const gpa = std.testing.allocator;
    var slots: Slots(u8, Handle) = .{};
    defer slots.deinit(gpa);

    const first = try slots.add(gpa, 1);
    try std.testing.expect(slots.remove(first));
    const second = try slots.add(gpa, 2);
    try std.testing.expectEqual(first.index, second.index);
    try std.testing.expect(slots.get(first) == null);
    try std.testing.expectEqual(@as(u8, 2), slots.get(second).?.*);
    try std.testing.expect(!slots.remove(first));
}
