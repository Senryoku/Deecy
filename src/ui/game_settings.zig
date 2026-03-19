const std = @import("std");
const zgui = @import("zgui");
const log = std.log.scoped(.game_settings);

const GameFile = @import("../deecy_ui.zig").GameFile;
const Cheats = @import("../cheats.zig");
const Icons = @import("./common.zig").Icons;

selected_file: ?*const GameFile = null,
cheats: std.ArrayList(Cheats.Cheat) = .empty,

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    for (self.cheats.items) |*c| c.deinit(allocator);
    self.cheats.deinit(allocator);
    self.cheats = .empty;

    self.selected_file = null;
}

pub fn setup(self: *@This(), allocator: std.mem.Allocator, entry: *const GameFile) !void {
    std.debug.assert(self.selected_file == null);
    self.selected_file = entry;
    if (entry.product_name != null and entry.product_id != null) {
        if (try Cheats.load(allocator, entry.product_name.?, entry.product_id.?)) |cheats|
            self.cheats = .fromOwnedSlice(cheats);
    }
    // TODO: Load per-game settings for this game.
}

pub fn open(self: *@This()) void {
    _ = self;
    zgui.openPopup("Game Settings", .{});
}

fn close(self: *@This(), allocator: std.mem.Allocator) void {
    if (self.selected_file) |f|
        if (f.product_name != null and f.product_id != null)
            Cheats.save(allocator, f.product_name.?, f.product_id.?, self.cheats.items) catch |err|
                std.log.err("Failed to save cheats: {t}", .{err});
    zgui.closeCurrentPopup();
    self.deinit(allocator);
}

/// Needs to be called on the same stack ID as open()
pub fn draw(self: *@This(), allocator: std.mem.Allocator) !void {
    if (zgui.beginPopupModal("Game Settings", .{ .flags = .{ .always_auto_resize = true } })) {
        if (zgui.beginTabBar("GameSettingsTabBar", .{})) {
            // if (zgui.beginTabItem("Settings", .{})) {
            //     zgui.textUnformatted("TODO: Game Settings");
            //     zgui.endTabItem();
            // }
            if (zgui.beginTabItem("Cheats", .{})) {
                var buffer: [256:0]u8 = undefined;
                var cheat_index_to_delete: ?usize = null;
                if (zgui.beginChild("##Scrollable", .{ .w = 640, .h = 480, .child_flags = .{ .frame_style = true } })) {
                    if (self.cheats.items.len == 0) zgui.textUnformatted("No cheats.");

                    for (self.cheats.items, 0..) |*c, i| {
                        zgui.pushIntId(@intCast(i));
                        defer zgui.popId();

                        _ = zgui.checkbox("##Enabled", .{ .v = &c.enabled });
                        zgui.sameLine(.{});
                        @memset(&buffer, 0);
                        @memcpy(buffer[0..@min(buffer.len, c.name.len)], c.name[0..@min(buffer.len, c.name.len)]);
                        if (zgui.inputText("##Name", .{ .buf = &buffer, .flags = .{} })) {
                            allocator.free(c.name);
                            c.name = try allocator.dupe(u8, buffer[0..std.mem.indexOfScalar(u8, &buffer, 0).?]);
                        }
                        zgui.sameLine(.{});
                        if (zgui.button(Icons.Trash, .{})) cheat_index_to_delete = i;

                        zgui.indent(.{});
                        defer zgui.unindent(.{});

                        var action_index_to_delete: ?usize = null;
                        for (c.actions, 0..) |*a, aidx| {
                            zgui.pushIntId(@intCast(aidx));
                            defer zgui.popId();

                            var action_type = std.meta.activeTag(a.*);
                            zgui.setNextItemWidth(100.0);
                            if (zgui.comboFromEnum("##ActionType", &action_type)) {
                                switch (action_type) {
                                    inline else => |at| a.* = @unionInit(Cheats.Action, @tagName(at), .{}),
                                }
                            }

                            if (a.* == .Condition) {
                                zgui.sameLine(.{});
                                zgui.setNextItemWidth(100.0);
                                _ = zgui.comboFromEnum("##Condition", &a.Condition.condition);
                                zgui.sameLine(.{});
                                zgui.setNextItemWidth(100.0);
                                if (zgui.inputScalar("##Count", u8, .{ .v = &a.Condition.count, .step = 1, .cfmt = "%d", .flags = .{} })) {}
                            }

                            zgui.sameLine(.{});
                            const addr = a.address_ptr();
                            zgui.setNextItemWidth(100.0);
                            if (zgui.inputScalar("##Address", u32, .{ .v = addr, .step = null, .cfmt = "%08X", .flags = .{ .chars_hexadecimal = true } }))
                                addr.* = @intCast(std.math.clamp(addr.*, 0x0C000000, 0x0D000000));
                            zgui.sameLine(.{});
                            var value_type = std.meta.activeTag(a.value_ptr().*);
                            zgui.setNextItemWidth(64.0);
                            if (zgui.comboFromEnum("##Type", &value_type)) {
                                switch (value_type) {
                                    inline else => |vt| a.value_ptr().* = @unionInit(Cheats.Value, @tagName(vt), 0),
                                }
                            }
                            zgui.sameLine(.{});
                            switch (a.value_ptr().*) {
                                inline else => |*v| {
                                    zgui.setNextItemWidth(100.0);
                                    _ = zgui.inputScalar("##Value", @typeInfo(@TypeOf(v)).pointer.child, .{ .v = v, .step = null, .cfmt = "%X", .flags = .{ .chars_hexadecimal = true } });
                                },
                            }
                            zgui.sameLine(.{});
                            if (zgui.button(Icons.Trash, .{})) action_index_to_delete = i;
                        }
                        if (action_index_to_delete) |idx| {
                            var tmp = try allocator.alloc(Cheats.Action, c.actions.len - 1);
                            @memcpy(tmp[0..idx], c.actions[0..idx]);
                            @memcpy(tmp[idx..], c.actions[idx + 1 ..]);
                            allocator.free(c.actions);
                            c.actions = tmp;
                        }
                        if (zgui.button(Icons.CirclePlus ++ " Add Action", .{})) {
                            var tmp = try allocator.alloc(Cheats.Action, c.actions.len + 1);
                            @memcpy(tmp[0..c.actions.len], c.actions);
                            tmp[c.actions.len] = .{ .Write = .{} };
                            allocator.free(c.actions);
                            c.actions = tmp;
                        }
                    }
                }
                zgui.endChild();

                if (cheat_index_to_delete) |idx|
                    self.cheats.orderedRemove(idx).deinit(allocator);

                if (zgui.button(Icons.SquarePlus ++ " New Cheat", .{})) {
                    const new_cheat = Cheats.Cheat{
                        .enabled = true,
                        .name = try allocator.dupe(u8, "New Cheat"),
                        .actions = try allocator.alloc(Cheats.Action, 1),
                    };
                    new_cheat.actions[0] = .{ .Write = .{} };
                    try self.cheats.append(allocator, new_cheat);
                }
                zgui.sameLine(.{});
                if (zgui.button("Import", .{}))
                    zgui.openPopup("Import Cheat", .{});

                if (zgui.beginPopupModal("Import Cheat", .{ .flags = .{ .always_auto_resize = true } })) {
                    try self.import_cheat_popup(allocator);
                    zgui.endPopup();
                }

                zgui.endTabItem();
            }
            zgui.endTabBar();
        }
        if (zgui.button("Save & Close", .{})) self.close(allocator);
        zgui.endPopup();
    }
}

fn import_cheat_popup(self: *@This(), allocator: std.mem.Allocator) !void {
    const static = struct {
        var import_buffer: [256:0]u8 = @splat(0);
        var last_error: anyerror = error.None;
    };
    _ = zgui.inputTextMultiline("##ImportText", .{ .buf = &static.import_buffer, .w = 320, .h = 240, .flags = .{} });
    if (zgui.button("Import##Button", .{})) {
        var reader: std.Io.Reader = .fixed(std.mem.sliceTo(&static.import_buffer, 0));
        if (@import("../codebreaker.zig").parse(allocator, &reader)) |cheats| cancel: {
            defer allocator.free(cheats);
            var actions: std.ArrayList(Cheats.Action) = .empty;
            defer actions.deinit(allocator);
            for (cheats) |c| {
                try actions.append(allocator, switch (c) {
                    .u8 => |v| .{ .Write = .{ .address = 0x0C000000 + v.address, .value = .{ .u8 = v.value } } },
                    .u16 => |v| .{ .Write = .{ .address = 0x0C000000 + v.address, .value = .{ .u16 = v.value } } },
                    .u32 => |v| .{ .Write = .{ .address = 0x0C000000 + v.address, .value = .{ .u32 = v.value } } },
                    .Condition => |v| .{ .Condition = .{ .condition = .from_codebreaker(v.condition), .count = 1, .address = 0x0C000000 + v.address, .value = .{ .u16 = v.value } } },
                    else => {
                        log.err("Unsupported CodeBreaker cheat: {any}", .{c});
                        zgui.openPopup("Import Error", .{});
                        static.last_error = error.UnsupportedType;
                        break :cancel;
                    },
                });
            }
            const new_cheat = Cheats.Cheat{
                .enabled = true,
                .name = try allocator.dupe(u8, "Imported Cheat"),
                .actions = try actions.toOwnedSlice(allocator),
            };
            try self.cheats.append(allocator, new_cheat);
            zgui.closeCurrentPopup();
        } else |err| {
            static.last_error = err;
            log.err("Failed to parse CodeBreaker cheat: {t}", .{err});
            zgui.openPopup("Import Error", .{});
        }
    }
    if (zgui.button("Cancel", .{})) zgui.closeCurrentPopup();

    if (zgui.beginPopupModal("Import Error", .{ .flags = .{ .always_auto_resize = true } })) {
        zgui.text("Failed to import CodeBreaker cheat: {t}", .{static.last_error});
        if (zgui.button("Ok", .{})) zgui.closeCurrentPopup();
        zgui.endPopup();
    }
}
