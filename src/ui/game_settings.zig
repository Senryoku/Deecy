//! Game specific settings (& cheats) edition UI
selected_file: ?*const GameFile = null,
cheats: std.ArrayList(Cheats.Cheat) = .empty,
settings: GameSettings = .{},

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    for (self.cheats.items) |*c| c.deinit(allocator);
    self.cheats.deinit(allocator);
    self.cheats = .empty;
    self.settings = .{};

    self.selected_file = null;
}

pub fn setup(self: *@This(), allocator: std.mem.Allocator, io: std.Io, entry: *const GameFile) !void {
    std.debug.assert(self.selected_file == null);
    self.selected_file = entry;
    if (entry.product_name) |name| {
        if (entry.product_id) |id| {
            if (try Cheats.load(allocator, io, .{ .name = name, .id = id })) |cheats|
                self.cheats = .fromOwnedSlice(cheats);
            self.settings = try GameSettings.load(io, allocator, .{ .name = name, .id = id });
        }
    }
}

pub fn open(self: *@This()) void {
    _ = self;
    zgui.openPopup("Game Settings", .{});
}

fn close(self: *@This(), io: std.Io, allocator: std.mem.Allocator) void {
    if (self.selected_file) |f| {
        if (f.product_name) |name| {
            if (f.product_id) |id| {
                Cheats.save(allocator, io, .{ .name = name, .id = id }, self.cheats.items) catch |err| {
                    std.log.err("Failed to save cheats: {t}", .{err});
                };
                self.settings.save(io, allocator, .{ .name = name, .id = id }) catch |err|
                    std.log.err("Failed to save game settings: {}", .{err});
            }
        }
    }
    zgui.closeCurrentPopup();
    self.deinit(allocator);
}

/// Needs to be called on the same stack ID as open()
pub fn draw(self: *@This(), io: std.Io, allocator: std.mem.Allocator) !void {
    const dropdown_size = 196;
    if (zgui.beginPopupModal("Game Settings", .{ .flags = .{ .always_auto_resize = true } })) {
        defer zgui.endPopup();
        const name = self.selected_file.?.product_name orelse return;
        const id = self.selected_file.?.product_id orelse return;
        zgui.text("Settings for '{s}' ({s})", .{ name, id });
        if (zgui.beginTabBar("Game Settings Tab Bar", .{})) {
            defer zgui.endTabBar();
            if (zgui.beginTabItem("Settings", .{})) {
                defer zgui.endTabItem();
                zgui.setNextItemWidth(dropdown_size);
                _ = zgui.comboFromEnum("Region", &self.settings.region);
                zgui.setNextItemWidth(dropdown_size);
                _ = zgui.comboFromEnum("Video Cable", &self.settings.video_cable);
                zgui.setNextItemWidth(dropdown_size);
                _ = zgui.comboFromEnum("BIOS Emulation", &self.settings.bios_emulation);
                zgui.separatorText("Rendering");
                _ = draw_renderer_game_settings(&self.settings.rendering);
            }
            if (zgui.beginTabItem("Cheats", .{})) {
                defer zgui.endTabItem();
                var buffer: [256:0]u8 = undefined;
                var cheat_index_to_delete: ?usize = null;
                if (zgui.beginChild("##Scrollable", .{ .w = 640, .h = 480, .child_flags = .{ .frame_style = true } })) {
                    if (self.cheats.items.len == 0) zgui.textUnformatted("No cheats.");

                    for (self.cheats.items, 0..) |*c, cheat_idx| {
                        zgui.pushIntId(@intCast(cheat_idx));
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
                        if (zgui.button(Icons.Trash, .{})) cheat_index_to_delete = cheat_idx;

                        zgui.indent(.{});
                        defer zgui.unindent(.{});

                        var action_index_to_delete: ?usize = null;
                        for (c.actions, 0..) |*a, action_index| {
                            zgui.pushIntId(@intCast(action_index));
                            defer zgui.popId();

                            var action_type = std.meta.activeTag(a.*);
                            zgui.setNextItemWidth(110.0);
                            if (zgui.comboFromEnum("##ActionType", &action_type)) {
                                switch (action_type) {
                                    inline else => |at| a.* = @unionInit(Cheats.Action, @tagName(at), .{}),
                                }
                            }
                            if (a.* == .Condition) {
                                zgui.sameLine(.{});
                                zgui.setNextItemWidth(32.0);
                                if (zgui.inputScalar("##Count", u8, .{ .v = &a.Condition.count, .step = null, .cfmt = "%d", .flags = .{} })) {}
                            }

                            zgui.sameLine(.{});
                            const addr = a.address_ptr();
                            zgui.setNextItemWidth(80.0);
                            if (zgui.inputScalar("##Address", u32, .{ .v = addr, .step = null, .cfmt = "%08X", .flags = .{ .chars_hexadecimal = true } }))
                                addr.* = @intCast(std.math.clamp(addr.*, 0x0C000000, 0x0D000000));
                            if (a.* == .Condition) {
                                zgui.sameLine(.{});
                                zgui.setNextItemWidth(50.0);
                                _ = zgui.comboFromEnum("##Condition", &a.Condition.condition);
                            }
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
                                    zgui.setNextItemWidth(80.0);
                                    _ = zgui.inputScalar("##Value", @typeInfo(@TypeOf(v)).pointer.child, .{ .v = v, .step = null, .cfmt = "%X", .flags = .{ .chars_hexadecimal = true } });
                                },
                            }
                            zgui.sameLine(.{});
                            if (zgui.button(Icons.Trash, .{})) action_index_to_delete = action_index;
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
            }
        }
        zgui.separator();
        if (zgui.button("Save & Close", .{})) self.close(io, allocator);
    }
}
pub fn draw_renderer_game_settings(game_settings: *Renderer.GameSettings) bool {
    const dropdown_size = 196;
    var modified = false;
    zgui.setNextItemWidth(dropdown_size);
    modified = zgui.comboFromEnum("Aspect Ratio", &game_settings.aspect_ratio) or modified;
    zgui.setItemTooltip(
        \\ Anything other than 4:3 will require a compatible (or modified) game.
        \\   4:3             Default
        \\   16:9 (Stretch)  Rendered at the normal resolution, but streched horizontally to 16:9. Cheap and accurate. (Anamorphic widescreen)
        \\   16:9            Rendered at an increased horizontal resolution. More expensive and might be less compatible.
    , .{});
    zgui.setNextItemWidth(dropdown_size);
    modified = zgui.comboFromEnum("Scaling Filter", &game_settings.scaling_filter) or modified;
    zgui.separatorText("Compatibility Tweaks");
    modified = zgui.checkbox("Framebuffer Emulation", .{ .v = &game_settings.framebuffer_emulation }) or modified;
    zgui.setItemTooltip("Allow re-use of the result of rendering to the framebuffer.\nSlower, particularly with 'Copy to Guest VRAM' enabled, but necessary for some effects (Static loading screens for example).", .{});
    if (game_settings.framebuffer_emulation and game_settings.copy_to_vram) {
        zgui.sameLine(.{});
        zgui.textUnformattedColored(common.Yellow, Icons.TriangleExclamation);
        zgui.setItemTooltip("'Framebuffer Emulation' and 'Copy to Guest VRAM' are rarely necessary at the same time and can hinder performance.", .{});
    }
    modified = zgui.checkbox("Copy to Guest VRAM", .{ .v = &game_settings.copy_to_vram }) or modified;
    zgui.setItemTooltip("Copy the result of rendering to the guest VRAM.\nSlower, particularly with 'Framebuffer Emulation' enabled, but necessary for some effects.", .{});
    modified = zgui.checkbox("Clamp Sprites UVs", .{ .v = &game_settings.clamp_sprites_uvs }) or modified;
    zgui.setItemTooltip("Avoid some seams around sprites when upscaling.", .{});
    modified = zgui.checkbox("Synchronous Render", .{ .v = &game_settings.synchronous_render }) or modified;
    zgui.setItemTooltip(
        \\ Render synchronously with the guest system.
        \\ Can avoid some synchronization issues at a slight performance cost.
        \\ Try this when you notice corrupted textures, especially during transitions.
    , .{});
    modified = zgui.checkbox("Delayed Render", .{ .v = &game_settings.delay_render }) or modified;
    zgui.setItemTooltip(
        \\ Delay rendering until a frame is actually presented.
        \\ Can prevent flickering or missing pause screens, it should only be enabled when encountering these issues.
        \\ Might require 'Synchronous Render' to be enabled as well."
    , .{});
    return modified;
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

const std = @import("std");
const zgui = @import("zgui");
const log = std.log.scoped(.game_settings);
const common = @import("./common.zig");

const Partial = @import("helpers").Partial;
const GameFile = @import("../deecy_ui.zig").GameFile;
const Cheats = @import("../cheats.zig");
const Icons = @import("./common.zig").Icons;
const Renderer = @import("../renderer.zig");
const GameSettings = @import("../GameSettings.zig");
