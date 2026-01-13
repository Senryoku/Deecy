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
            .controller => |controller| try writer.print("controller.{s}", .{@tagName(controller)}),
            .keyboard => |keyboard| try writer.print("keyboard.{f}{s}", .{ keyboard.mods, @tagName(keyboard.key) }),
        }
    }
};

pub const Action = struct {
    pub const Name = enum {
        Screenshot,
        ToggleUI,
        StartPause,
        DebugUI,
        ToggleFullscreen,
        ToggleRealtime,
        NextVBlankIn,
        SaveState1,
        SaveState2,
        SaveState3,
        SaveState4,
        LoadState1,
        LoadState2,
        LoadState3,
        LoadState4,
    };
    name: Name,
    callback: *const fn (*Deecy) void,
};

shortcuts: std.AutoHashMap(Key, Action),

pub fn init(allocator: std.mem.Allocator) !@This() {
    var self = @This(){
        .shortcuts = .init(allocator),
    };
    try self.deserialize(allocator);
    return self;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    self.serialize(allocator) catch |err| log.err("Failed to serialize shortcuts: {t}", .{err});
    self.shortcuts.deinit();
}

pub fn on_key(self: *@This(), key: Key) void {
    if (self.shortcuts.get(key)) |shortcut| {
        const deecy: *Deecy = @fieldParentPtr("shortcuts", self);
        shortcut.callback(deecy);
    }
}

const Actions = actions_table: {
    var table: [@typeInfo(Action.Name).@"enum".fields.len]Action = undefined;
    for (.{
        .{ Action.Name.Screenshot, Deecy.save_screenshot },
        .{ Action.Name.ToggleUI, Deecy.toggle_ui },
        .{ Action.Name.StartPause, Deecy.start_pause },
        .{ Action.Name.DebugUI, Deecy.toggle_debug_ui },
        .{ Action.Name.ToggleFullscreen, Deecy.toggle_fullscreen },
        .{ Action.Name.ToggleRealtime, Deecy.toggle_realtime },
        .{ Action.Name.NextVBlankIn, Deecy.next_vblankin },
        .{ Action.Name.SaveState1, Deecy.save_state_idx(0) },
        .{ Action.Name.SaveState2, Deecy.save_state_idx(1) },
        .{ Action.Name.SaveState3, Deecy.save_state_idx(2) },
        .{ Action.Name.SaveState4, Deecy.save_state_idx(3) },
        .{ Action.Name.LoadState1, Deecy.load_state_idx(0) },
        .{ Action.Name.LoadState2, Deecy.load_state_idx(1) },
        .{ Action.Name.LoadState3, Deecy.load_state_idx(2) },
        .{ Action.Name.LoadState4, Deecy.load_state_idx(3) },
    }) |entry| {
        table[@intFromEnum(entry[0])] = .{ .name = entry[0], .callback = entry[1] };
    }
    break :actions_table table;
};

const SerializedShortcut = struct {
    // FIXME: I don't love this, as this means any change to Action.Name will lead to very weird behavior. It might me better to store the full name as a string.
    action: Action.Name,
    key: Key,
};

fn serialize(self: @This(), allocator: std.mem.Allocator) !void {
    if (self.shortcuts.count() == 0) return;

    var list: std.ArrayList(SerializedShortcut) = .empty;
    defer list.deinit(allocator);
    var it = self.shortcuts.iterator();
    while (it.next()) |entry| {
        try list.append(allocator, .{ .key = entry.key_ptr.*, .action = entry.value_ptr.name });
    }

    const config_path = try get_config_path(allocator);
    defer allocator.free(config_path);

    var config_file = try std.fs.cwd().createFile(config_path, .{});
    defer config_file.close();

    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = config_file.writer(buffer);
    try std.zon.stringify.serialize(list.items, .{}, &writer.interface);
    try writer.end();
}

fn deserialize(self: *@This(), allocator: std.mem.Allocator) !void {
    const config_path = try get_config_path(allocator);
    defer allocator.free(config_path);

    var file = std.fs.cwd().openFile(config_path, .{}) catch |err| {
        log.warn("Failed to open shortcuts file: {t}. Loading default configuration.", .{err});
        return self.load_default_shortcuts();
    };
    defer file.close();
    const data = try file.readToEndAllocOptions(allocator, 32 * 1024 * 1024, null, .@"8", 0);
    defer allocator.free(data);

    var diagnostics: std.zon.parse.Diagnostics = .{};
    const zon = std.zon.parse.fromSlice([]const SerializedShortcut, allocator, data, &diagnostics, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
        log.err(termcolor.red("Failed to parse shortcuts file: {t}."), .{err});
        log.err("{f}", .{diagnostics});
        return err;
    };
    defer allocator.free(zon);
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

fn load_default_shortcuts(self: *@This()) !void {
    self.shortcuts.clearRetainingCapacity();
    inline for (.{
        .{ zglfw.Key.escape, Action.Name.ToggleUI },
        .{ zglfw.Key.space, Action.Name.StartPause },
        .{ zglfw.Key.d, Action.Name.DebugUI },
        .{ zglfw.Key.f, Action.Name.ToggleFullscreen },
        .{ zglfw.Key.l, Action.Name.ToggleRealtime },
        .{ zglfw.Key.n, Action.Name.NextVBlankIn },
        .{ zglfw.Key.F1, Action.Name.SaveState1 },
        .{ zglfw.Key.F2, Action.Name.SaveState2 },
        .{ zglfw.Key.F3, Action.Name.SaveState3 },
        .{ zglfw.Key.F4, Action.Name.SaveState4 },
        .{ zglfw.Key.F5, Action.Name.LoadState1 },
        .{ zglfw.Key.F6, Action.Name.LoadState2 },
        .{ zglfw.Key.F7, Action.Name.LoadState3 },
        .{ zglfw.Key.F8, Action.Name.LoadState4 },
        .{ zglfw.Key.F12, Action.Name.Screenshot },
    }) |entry|
        try self.shortcuts.put(.{ .keyboard = .{ .key = entry[0] } }, get_action(entry[1]));
}
