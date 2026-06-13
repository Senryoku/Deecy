const std = @import("std");
const zglfw = @import("zglfw");

const termcolor = @import("termcolor");
const Deecy = @import("../deecy.zig");
const HostPaths = @import("dreamcast").HostPaths;

const log = std.log.scoped(.deecy);

pub const Modifiers = struct {
    shift: bool = false,
    control: bool = false,
    alt: bool = false,
    super: bool = false,

    pub fn to_glfw(self: @This()) zglfw.Mods {
        return .{
            .shift = self.shift,
            .control = self.control,
            .alt = self.alt,
            .super = self.super,
        };
    }

    pub fn from_glfw(mods: zglfw.Mods) @This() {
        return .{
            .shift = mods.shift,
            .control = mods.control,
            .alt = mods.alt,
            .super = mods.super,
        };
    }

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        if (self.control) try writer.print("Control + ", .{});
        if (self.alt) try writer.print("Alt + ", .{});
        if (self.shift) try writer.print("Shift + ", .{});
        if (self.super) try writer.print("Super + ", .{});
    }
};

pub const Key = union(enum) {
    controller: zglfw.Gamepad.Button,
    keyboard: struct {
        key: zglfw.Key,
        mods: Modifiers = .{},
    },

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        switch (self) {
            .controller => |controller| try writer.print("{s}", .{@import("helpers").title_case_enum(controller)}),
            .keyboard => |keyboard| try writer.print("{f}{s}", .{ keyboard.mods, @import("helpers").title_case_enum(keyboard.key) }),
        }
    }
};

pub const Action = struct {
    /// Milliseconds
    const InitialHoldTime = 300;
    const RepeatInterval = 100;
    pub const Name = enum {
        @"Debug UI",
        @"Load State 1",
        @"Load State 2",
        @"Load State 3",
        @"Load State 4",
        @"Next VBlank In",
        @"Save State 1",
        @"Save State 2",
        @"Save State 3",
        @"Save State 4",
        Screenshot,
        @"Start/Pause",
        @"Toggle Fullscreen",
        @"Toggle Realtime",
        @"Toggle UI",
        @"Start Launcher",
        @"Rewind - Previous Snapshot",
        @"Rewind - Next Snapshot",
        @"Rewind - Confirm",
        @"Rewind - Cancel",
    };
    name: Name,
    callback: *const fn (*Deecy) void,
    allow_repeat: bool = false,

    /// In milliseconds
    hold_start: ?i64 = null,
    last_repeat: ?i64 = null,

    pub fn call(self: *@This(), deecy: *Deecy) void {
        self.hold_start = null;
        self.last_repeat = null;
        self.callback(deecy);
    }

    pub fn on_hold(self: *@This(), deecy: *Deecy) void {
        if (self.allow_repeat) {
            const now = std.Io.Timestamp.now(deecy.io, .awake).toMilliseconds();
            if (self.hold_start) |hold_start| {
                if (now < hold_start) {
                    self.hold_start = null;
                    return;
                }
                if (now - hold_start > InitialHoldTime) {
                    if (self.last_repeat) |last_repeat| {
                        if (now < last_repeat) {
                            self.last_repeat = null;
                            return;
                        }
                        if (now - last_repeat > RepeatInterval) {
                            self.last_repeat = now;
                            self.callback(deecy);
                        }
                    } else {
                        self.last_repeat = now;
                        self.callback(deecy);
                    }
                } else {
                    self.last_repeat = null;
                }
            } else {
                self.hold_start = now;
            }
        }
    }
};

shortcuts: std.AutoHashMap(Key, Action),

pub fn init(allocator: std.mem.Allocator, io: std.Io) !@This() {
    var self = @This(){
        .shortcuts = .init(allocator),
    };
    self.deserialize(allocator, io) catch |err| {
        switch (err) {
            error.FileNotFound => try self.load_default_shortcuts(),
            else => return err,
        }
    };
    return self;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator, io: std.Io) void {
    self.serialize(allocator, io) catch |err| log.err("Failed to serialize shortcuts: {t}", .{err});
    self.shortcuts.deinit();
}

fn app_ptr(self: *@This()) *Deecy {
    return @alignCast(@fieldParentPtr("shortcuts", self));
}

pub fn on_press(self: *@This(), key: Key) void {
    if (self.shortcuts.getPtr(key)) |shortcut|
        shortcut.call(self.app_ptr());
}

/// Bypasses internal repeat timer. Intended to be used with the OS repeat logic.
/// Do not mix on_repeat and on_hold with the same type of input device.
pub fn on_repeat(self: *@This(), key: Key) void {
    if (self.shortcuts.getPtr(key)) |shortcut| {
        if (shortcut.allow_repeat)
            shortcut.call(self.app_ptr());
    }
}

/// Intended to be used with input without OS repeat logic (e.g. controller).
/// Do not mix on_repeat and on_hold with the same type of input device.
pub fn on_hold(self: *@This(), key: Key) void {
    if (self.shortcuts.getPtr(key)) |shortcut|
        shortcut.on_hold(self.app_ptr());
}

pub fn put(self: *@This(), key: Key, action: Action.Name) !void {
    try self.shortcuts.put(key, get_action(action));
}

pub fn remove(self: *@This(), key: Key) void {
    _ = self.shortcuts.remove(key);
}

const Actions = actions_table: {
    var table: [@typeInfo(Action.Name).@"enum".fields.len]Action = undefined;
    for ([_]Action{
        // zig fmt: off
        .{ .name = .Screenshot,                    .callback = Deecy.save_screenshot   },
        .{ .name = .@"Toggle UI",                  .callback = Deecy.toggle_ui         },
        .{ .name = .@"Start/Pause",                .callback = Deecy.start_pause       },
        .{ .name = .@"Debug UI",                   .callback = Deecy.toggle_debug_ui   },
        .{ .name = .@"Toggle Fullscreen",          .callback = Deecy.toggle_fullscreen },
        .{ .name = .@"Toggle Realtime",            .callback = Deecy.toggle_realtime   },
        .{ .name = .@"Save State 1",               .callback = Deecy.save_state_idx(0) },
        .{ .name = .@"Save State 2",               .callback = Deecy.save_state_idx(1) },
        .{ .name = .@"Save State 3",               .callback = Deecy.save_state_idx(2) },
        .{ .name = .@"Save State 4",               .callback = Deecy.save_state_idx(3) },
        .{ .name = .@"Load State 1",               .callback = Deecy.load_state_idx(0) },
        .{ .name = .@"Load State 2",               .callback = Deecy.load_state_idx(1) },
        .{ .name = .@"Load State 3",               .callback = Deecy.load_state_idx(2) },
        .{ .name = .@"Load State 4",               .callback = Deecy.load_state_idx(3) },
        .{ .name = .@"Start Launcher",             .callback = Deecy.start_launcher    },
        .{ .name = .@"Rewind - Confirm",           .callback = Deecy.rewind_confirm    },
        .{ .name = .@"Rewind - Cancel",            .callback = Deecy.rewind_cancel     },
        .{ .name = .@"Rewind - Previous Snapshot", .callback = Deecy.previous_snapshot, .allow_repeat = true },
        .{ .name = .@"Rewind - Next Snapshot",     .callback = Deecy.next_snapshot,     .allow_repeat = true },
        .{ .name = .@"Next VBlank In",             .callback = Deecy.next_vblankin,     .allow_repeat = true  },
        // zig fmt: on
    }) |entry| {
        table[@intFromEnum(entry.name)] = entry;
    }
    break :actions_table table;
};

const SerializedShortcut = struct {
    action: Action.Name,
    key: Key,
};

fn serialize(self: @This(), allocator: std.mem.Allocator, io: std.Io) !void {
    if (self.shortcuts.count() == 0) return;

    var list: std.ArrayList(SerializedShortcut) = .empty;
    defer list.deinit(allocator);
    var it = self.shortcuts.iterator();
    while (it.next()) |entry| {
        try list.append(allocator, .{ .key = entry.key_ptr.*, .action = entry.value_ptr.name });
    }

    const config_path = try get_config_path(allocator);
    defer allocator.free(config_path);

    var config_file = try std.Io.Dir.cwd().createFile(io, config_path, .{});
    defer config_file.close(io);

    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = config_file.writer(io, buffer);
    try std.zon.stringify.serialize(list.items, .{}, &writer.interface);
    try writer.end();
}

fn deserialize(self: *@This(), allocator: std.mem.Allocator, io: std.Io) !void {
    const config_path = try get_config_path(allocator);
    defer allocator.free(config_path);

    const data = try std.Io.Dir.cwd().readFileAllocOptions(io, config_path, allocator, .limited(32 * 1024 * 1024), .@"8", 0);
    defer allocator.free(data);

    var diagnostics: std.zon.parse.Diagnostics = .{};
    defer diagnostics.deinit(allocator);
    const zon = std.zon.parse.fromSliceAlloc([]const SerializedShortcut, allocator, data, &diagnostics, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
        log.err(termcolor.red("Failed to parse shortcuts file: {t}."), .{err});
        log.err("{f}", .{diagnostics});
        return err;
    };
    defer std.zon.parse.free(allocator, zon);
    self.shortcuts.clearRetainingCapacity();
    for (zon) |shortcut|
        try self.shortcuts.put(shortcut.key, get_action(shortcut.action));
    // FIXME: If new default shortcuts are added by an update, they won't appear if the user already have a config file...
}

fn get_config_path(allocator: std.mem.Allocator) ![]const u8 {
    return try std.fs.path.join(allocator, &[_][]const u8{ HostPaths.get_userdata_path(), "shortcuts.zon" });
}

fn get_action(name: Action.Name) Action {
    return Actions[@intFromEnum(name)];
}

pub fn load_default_shortcuts(self: *@This()) !void {
    self.shortcuts.clearRetainingCapacity();
    inline for ([_]struct { zglfw.Key, Action.Name }{
        .{ .space, .@"Start/Pause" },
        .{ .d, .@"Debug UI" },
        .{ .f, .@"Toggle Fullscreen" },
        .{ .l, .@"Toggle Realtime" },
        .{ .n, .@"Next VBlank In" },
        .{ .F1, .@"Save State 1" },
        .{ .F2, .@"Save State 2" },
        .{ .F3, .@"Save State 3" },
        .{ .F4, .@"Save State 4" },
        .{ .F5, .@"Load State 1" },
        .{ .F6, .@"Load State 2" },
        .{ .F7, .@"Load State 3" },
        .{ .F8, .@"Load State 4" },
        .{ .F12, .Screenshot },
    }) |entry|
        try self.shortcuts.put(.{ .keyboard = .{ .key = entry[0] } }, get_action(entry[1]));

    try self.shortcuts.put(.{ .keyboard = .{ .key = .enter, .mods = .{ .alt = true } } }, get_action(.@"Toggle Fullscreen"));
}
