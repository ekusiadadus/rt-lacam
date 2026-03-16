//! Generic ring-buffer double-ended queue.
//!
//! Provides O(1) amortized push/pop at both ends.
//! Grows by 2x when capacity is exhausted.

const std = @import("std");
const Allocator = std.mem.Allocator;

pub fn Deque(comptime T: type) type {
    return struct {
        const Self = @This();

        items: []T,
        head: usize,
        len: usize,
        allocator: Allocator,

        pub fn init(allocator: Allocator) Self {
            return .{
                .items = &[_]T{},
                .head = 0,
                .len = 0,
                .allocator = allocator,
            };
        }

        pub fn deinit(self: *Self) void {
            if (self.items.len > 0) {
                self.allocator.free(self.items);
            }
            self.* = undefined;
        }

        pub fn pushBack(self: *Self, item: T) !void {
            try self.ensureCapacity();
            const idx = (self.head + self.len) % self.items.len;
            self.items[idx] = item;
            self.len += 1;
        }

        pub fn pushFront(self: *Self, item: T) !void {
            try self.ensureCapacity();
            self.head = if (self.head == 0) self.items.len - 1 else self.head - 1;
            self.items[self.head] = item;
            self.len += 1;
        }

        pub fn popFront(self: *Self) ?T {
            if (self.len == 0) return null;
            const item = self.items[self.head];
            self.head = (self.head + 1) % self.items.len;
            self.len -= 1;
            return item;
        }

        pub fn popBack(self: *Self) ?T {
            if (self.len == 0) return null;
            self.len -= 1;
            const idx = (self.head + self.len) % self.items.len;
            return self.items[idx];
        }

        pub fn peekFront(self: *const Self) ?T {
            if (self.len == 0) return null;
            return self.items[self.head];
        }

        pub fn peekBack(self: *const Self) ?T {
            if (self.len == 0) return null;
            const idx = (self.head + self.len - 1) % self.items.len;
            return self.items[idx];
        }

        pub fn isEmpty(self: *const Self) bool {
            return self.len == 0;
        }

        fn ensureCapacity(self: *Self) !void {
            if (self.len < self.items.len) return;

            const new_cap = if (self.items.len == 0) @as(usize, 16) else self.items.len * 2;
            const new_items = try self.allocator.alloc(T, new_cap);

            // Copy old items in order
            for (0..self.len) |i| {
                new_items[i] = self.items[(self.head + i) % self.items.len];
            }

            if (self.items.len > 0) {
                self.allocator.free(self.items);
            }
            self.items = new_items;
            self.head = 0;
        }
    };
}

// --- Tests ---

const testing = std.testing;

test "deque: push_back and pop_front (FIFO)" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try d.pushBack(1);
    try d.pushBack(2);
    try d.pushBack(3);

    try testing.expectEqual(3, d.len);
    try testing.expectEqual(1, d.popFront().?);
    try testing.expectEqual(2, d.popFront().?);
    try testing.expectEqual(3, d.popFront().?);
    try testing.expectEqual(null, d.popFront());
}

test "deque: push_front and pop_back (FIFO reversed)" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try d.pushFront(1);
    try d.pushFront(2);
    try d.pushFront(3);

    try testing.expectEqual(3, d.len);
    try testing.expectEqual(1, d.popBack().?);
    try testing.expectEqual(2, d.popBack().?);
    try testing.expectEqual(3, d.popBack().?);
}

test "deque: push_front and pop_front (LIFO)" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try d.pushFront(1);
    try d.pushFront(2);
    try d.pushFront(3);

    try testing.expectEqual(3, d.popFront().?);
    try testing.expectEqual(2, d.popFront().?);
    try testing.expectEqual(1, d.popFront().?);
}

test "deque: grow beyond initial capacity" {
    var d = Deque(u32).init(testing.allocator);
    defer d.deinit();

    for (0..100) |i| {
        try d.pushBack(@intCast(i));
    }
    try testing.expectEqual(100, d.len);

    for (0..100) |i| {
        try testing.expectEqual(@as(u32, @intCast(i)), d.popFront().?);
    }
}

test "deque: wrap-around correctness" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    // Fill and drain to move head forward
    for (0..10) |_| {
        try d.pushBack(42);
    }
    for (0..10) |_| {
        _ = d.popFront();
    }

    // Now push/pop with wrapped head
    try d.pushBack(1);
    try d.pushBack(2);
    try d.pushFront(0);

    try testing.expectEqual(0, d.popFront().?);
    try testing.expectEqual(1, d.popFront().?);
    try testing.expectEqual(2, d.popFront().?);
}

test "deque: peek" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try testing.expectEqual(null, d.peekFront());
    try testing.expectEqual(null, d.peekBack());

    try d.pushBack(10);
    try d.pushBack(20);

    try testing.expectEqual(10, d.peekFront().?);
    try testing.expectEqual(20, d.peekBack().?);
    try testing.expectEqual(2, d.len); // peek doesn't remove
}

test "deque: isEmpty" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try testing.expect(d.isEmpty());
    try d.pushBack(1);
    try testing.expect(!d.isEmpty());
    _ = d.popFront();
    try testing.expect(d.isEmpty());
}

// --- Edge case tests ---

test "deque: empty pop returns null" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try testing.expectEqual(null, d.popFront());
    try testing.expectEqual(null, d.popBack());
    try testing.expectEqual(null, d.popFront());
}

test "deque: single element push/pop all directions" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    // pushBack + popFront
    try d.pushBack(1);
    try testing.expectEqual(1, d.popFront().?);
    try testing.expect(d.isEmpty());

    // pushBack + popBack
    try d.pushBack(2);
    try testing.expectEqual(2, d.popBack().?);
    try testing.expect(d.isEmpty());

    // pushFront + popFront
    try d.pushFront(3);
    try testing.expectEqual(3, d.popFront().?);
    try testing.expect(d.isEmpty());

    // pushFront + popBack
    try d.pushFront(4);
    try testing.expectEqual(4, d.popBack().?);
    try testing.expect(d.isEmpty());
}

test "deque: alternating push/pop stress" {
    var d = Deque(u32).init(testing.allocator);
    defer d.deinit();

    // Interleave pushFront/pushBack with pops to stress wrap-around
    for (0..200) |i| {
        if (i % 3 == 0) {
            try d.pushFront(@intCast(i));
        } else {
            try d.pushBack(@intCast(i));
        }
        if (i % 5 == 0 and !d.isEmpty()) {
            _ = d.popFront();
        }
        if (i % 7 == 0 and !d.isEmpty()) {
            _ = d.popBack();
        }
    }

    // Drain completely
    while (!d.isEmpty()) {
        _ = d.popFront();
    }
    try testing.expect(d.isEmpty());
}

test "deque: peek on single element" {
    var d = Deque(i32).init(testing.allocator);
    defer d.deinit();

    try d.pushBack(42);
    try testing.expectEqual(42, d.peekFront().?);
    try testing.expectEqual(42, d.peekBack().?);
    try testing.expectEqual(1, d.len);
}

test "deque: multiple grow cycles" {
    var d = Deque(u32).init(testing.allocator);
    defer d.deinit();

    // Initial capacity is 16, grow to 16 -> 32 -> 64 -> 128
    for (0..100) |i| {
        try d.pushFront(@intCast(i));
    }
    // Verify LIFO order
    for (0..100) |i| {
        const expected: u32 = @intCast(99 - i);
        try testing.expectEqual(expected, d.popFront().?);
    }
}
