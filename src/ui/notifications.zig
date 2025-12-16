const std = @import("std");
const zgui = @import("zgui");

const log = std.log.scoped(.ui);

const MaxNotifications = 5;
const Width = 256.0;
const Padding = 16.0;
const ExpireTime = 4000; // ms
const FadeOutTime = 1000; // ms

const Notification = struct {
    title: []const u8,
    text: []const u8,
    time: i64,

    pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
        allocator.free(self.title);
        allocator.free(self.text);
    }

    pub fn compare(_: void, self: @This(), other: @This()) std.math.Order {
        return std.math.order(other.time, self.time);
    }
};

notifications: std.ArrayList(Notification) = .empty,
_mutex: std.Thread.Mutex = .{},
_allocator: std.mem.Allocator,

pub fn init(allocator: std.mem.Allocator) @This() {
    return .{ ._allocator = allocator };
}

pub fn deinit(self: *@This()) void {
    self._mutex.lock();
    defer self._mutex.unlock();

    for (self.notifications.items) |*notification| notification.deinit(self._allocator);
    self.notifications.deinit();
}

pub fn push(self: *@This(), comptime title_fmt: []const u8, title_args: anytype, comptime text_fmt: []const u8, text_args: anytype) void {
    push_impl(self, title_fmt, title_args, text_fmt, text_args) catch |err| {
        log.err("Failed to push notification: {t}", .{err});
    };
}

fn push_impl(self: *@This(), comptime title_fmt: []const u8, title_args: anytype, comptime text_fmt: []const u8, text_args: anytype) !void {
    self._mutex.lock();
    defer self._mutex.unlock();

    while (self.notifications.items.len >= MaxNotifications)
        self.notifications.pop().?.deinit(self._allocator);

    const title = try std.fmt.allocPrint(self._allocator, title_fmt, title_args);
    errdefer self._allocator.free(title);
    const text = try std.fmt.allocPrint(self._allocator, text_fmt, text_args);
    errdefer self._allocator.free(text);
    try self.notifications.insert(self._allocator, 0, .{ .title = title, .text = text, .time = std.time.milliTimestamp() });
}

const ImguiWidgetIDs = arr: {
    var a: [MaxNotifications][:0]const u8 = @splat(undefined);
    for (&a, 0..) |*n, i| n.* = std.fmt.comptimePrint("Notification {d}", .{i + 1});
    break :arr a;
};

pub fn draw(self: *@This()) void {
    self._mutex.lock();
    defer self._mutex.unlock();

    const time = std.time.milliTimestamp();
    const window_size = zgui.io.getDisplaySize();
    while (self.notifications.items.len > 0 and time - self.notifications.getLast().time > ExpireTime)
        self.notifications.pop().?.deinit(self._allocator);

    var y: f32 = Padding;
    for (self.notifications.items, 0..) |notification, idx| {
        zgui.setNextWindowSize(.{ .w = Width, .h = 0.0, .cond = .always });
        zgui.setNextWindowPos(.{
            .x = window_size[0] - Padding,
            .y = window_size[1] - y,
            .cond = .always,
            .pivot_x = 1.0,
            .pivot_y = 1.0,
        });
        zgui.pushStyleVar1f(.{ .idx = .alpha, .v = @min(1.0, @as(f32, @floatFromInt(ExpireTime - (time - notification.time))) / FadeOutTime) });
        defer zgui.popStyleVar(.{ .count = 1 });
        if (zgui.begin(ImguiWidgetIDs[idx], .{ .flags = .{
            .always_auto_resize = true,
            .no_title_bar = true,
            .no_move = true,
            .no_resize = true,
            .no_docking = true,
            .no_mouse_inputs = true,
            .no_nav_inputs = true,
            .no_nav_focus = true,
            .no_focus_on_appearing = true,
            .no_scrollbar = true,
            .no_collapse = true,
            .no_saved_settings = true,
        } })) {
            if (notification.title.len > 0)
                zgui.text("{s}", .{notification.title});
            if (notification.text.len > 0)
                zgui.textWrapped("{s}", .{notification.text});
            y += Padding + zgui.getWindowHeight();
        }
        zgui.end();
    }
}
