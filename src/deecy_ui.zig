const std = @import("std");
const builtin = @import("builtin");
const termcolor = @import("termcolor");
const Once = @import("helpers").Once;

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zgpu = @import("zgpu");

const ui_log = std.log.scoped(.ui);

const nfd = @import("nfd");

const Deecy = @import("deecy.zig");
const HostPaths = Deecy.HostPaths;
const DreamcastModule = @import("dreamcast");
const MapleModule = DreamcastModule.Maple;
const Disc = DreamcastModule.GDROM.Disc;
const PVRFile = @import("pvr_file.zig");

const Notifications = @import("./ui/notifications.zig");
const wait_for = @import("./ui/wait_for_input.zig");

const Self = @This();

const GameFile = struct {
    path: [:0]const u8,
    name: [:0]const u8,
    texture: ?zgpu.TextureHandle,
    view: ?zgpu.TextureViewHandle,

    pub fn free(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
        allocator.free(self.path);
        allocator.free(self.name);
        if (self.texture) |texture| {
            gctx.releaseResource(texture);
            self.texture = null;
        }
        if (self.view) |view| {
            gctx.releaseResource(view);
            self.view = null;
        }
    }

    pub fn sort(_: void, a: @This(), b: @This()) bool {
        return std.ascii.orderIgnoreCase(a.name, b.name) == .lt;
    }
};

last_error: []const u8 = "",

vmu_displays: [4]struct {
    texture: zgpu.TextureHandle = .nil,
    view: zgpu.TextureViewHandle = .nil,
    valid: bool = false, // Is in use?
    display: bool = true, // Should be displayed?
    dirty: bool = false, // GPU texture is outdated?
    data: [48 * 32 / 8]u8 = @splat(255),
} = @splat(.{}),

binary_loaded: bool = false, // Indicates if we're running a raw binary loaded directly in RAM (not from a disc) (FIXME: Used to avoid drawing the game library when pausing without a disc. Clunky.)
disc_files: std.ArrayList(GameFile) = .empty,
disc_files_mutex: std.Thread.Mutex = .{}, // Used during disc_files population (then assumed to be constant outside of refresh_games)

notifications: Notifications,

deecy: *Deecy,
allocator: std.mem.Allocator,

pub fn create(allocator: std.mem.Allocator, d: *Deecy) !*@This() {
    var r = try allocator.create(@This());
    r.* = .{
        .notifications = .init(allocator),
        .deecy = d,
        .allocator = allocator,
    };
    for (0..4) |p|
        r.create_vmu_texture(@intCast(p));
    return r;
}

pub fn destroy(self: *@This()) void {
    for (self.disc_files.items) |*entry| entry.free(self.allocator, self.deecy.gctx);
    self.disc_files.deinit(self.allocator);

    for (self.vmu_displays) |texture| {
        self.deecy.gctx.releaseResource(texture.texture);
        self.deecy.gctx.releaseResource(texture.view);
    }

    self.allocator.destroy(self);
}

fn create_vmu_texture(self: *@This(), controller: u8) void {
    const tex = self.deecy.gctx.createTexture(.{
        .usage = .{ .texture_binding = true, .copy_dst = true },
        .size = .{
            .width = 48,
            .height = 32,
            .depth_or_array_layers = 1,
        },
        .format = .bgra8_unorm,
        .mip_level_count = 1,
    });
    const view = self.deecy.gctx.createTextureView(tex, .{});
    self.vmu_displays[controller] = .{ .texture = tex, .view = view };
    self.upload_vmu_texture(controller);
}

pub fn update_vmu_screen(self: *@This(), data: [*]const u8, controller: u8) void {
    @memcpy(&self.vmu_displays[controller].data, data[0 .. 48 * 32 / 8]);
    self.vmu_displays[controller].dirty = true;
}

pub fn vmu_screen_callback(comptime port_idx: u8) type {
    std.debug.assert(port_idx < 4);
    return struct {
        pub fn callback(self: *Self, data: [*]const u8) void {
            self.update_vmu_screen(data, port_idx);
        }
    };
}

/// Locks gctx_queue_mutex
pub fn upload_vmu_texture(self: *@This(), controller: u8) void {
    const colors = [2][3]u8{ // bgr
        .{ 152, 135, 92 }, // "white"
        .{ 104, 43, 40 }, // "black"
    };

    const tex = &self.vmu_displays[controller];
    var pixels: [4 * 48 * 32]u8 = undefined;
    for (0..32) |r| {
        const row = tex.data[6 * (31 - r) .. 6 * ((31 - r) + 1)];
        for (0..6) |c| {
            var byte = row[5 - c];
            for (0..8) |b| {
                const color: [3]u8 = colors[byte & 0x1];
                const idx = 4 * (48 * r + (8 * c + b));
                pixels[idx + 0] = color[0];
                pixels[idx + 1] = color[1];
                pixels[idx + 2] = color[2];
                pixels[idx + 3] = 255;
                byte >>= 1;
            }
        }
    }
    self.deecy.gctx_queue_mutex.lock();
    defer self.deecy.gctx_queue_mutex.unlock();
    self.deecy.gctx.queue.writeTexture(
        .{ .texture = self.deecy.gctx.lookupResource(tex.texture).? },
        .{ .bytes_per_row = 4 * 48, .rows_per_image = 32 },
        .{ .width = 48, .height = 32 },
        u8,
        &pixels,
    );
    tex.dirty = false;
}

pub fn draw_vmus(self: *@This(), editable: bool) void {
    if ((editable and self.deecy.config.display_debug_ui) or (!editable and !self.deecy.config.display_vmus)) return;

    zgui.setNextWindowSize(.{ .w = 4 * 48, .h = 2 * (4 * 32 + 8), .cond = .first_use_ever });
    zgui.setNextWindowPos(.{ .x = 32, .y = 32, .cond = .first_use_ever });

    zgui.pushStyleVar2f(.{ .idx = .window_padding, .v = .{ 0.0, 0.0 } });

    if (zgui.begin("VMUs", .{ .flags = .{
        .no_resize = !editable,
        .no_move = !editable,
        .no_title_bar = !editable,
        .no_mouse_inputs = !editable,
        .no_nav_inputs = !editable,
        .no_nav_focus = !editable,
        .no_background = !editable,
        .no_docking = true,
        .no_scrollbar = true,
    } })) {
        if (!editable) zgui.dummy(.{ .w = 0, .h = 18.0 });
        const win_width = zgui.getWindowSize()[0];
        inline for (&self.vmu_displays, 0..) |*tex, idx| {
            if (tex.valid) {
                if (tex.display) {
                    if (tex.dirty)
                        self.upload_vmu_texture(@intCast(idx));
                    zgui.image(.{ .tex_data = null, .tex_id = @enumFromInt(@intFromPtr((self.deecy.gctx.lookupResource(tex.view).?))) }, .{ .w = win_width, .h = win_width * 32.0 / 48.0 });
                }
                if (editable) {
                    _ = zgui.checkbox("Display #" ++ std.fmt.comptimePrint("{d}", .{idx}), .{ .v = &tex.display });
                } else zgui.dummy(.{ .w = 0, .h = 24.0 });
            }
        }
    }
    zgui.end();

    zgui.popStyleVar(.{});
}

// Locks gctx_queue_mutex.
fn update_game_info(self: *@This(), path: []const u8, image_width: u32, image_height: u32, image: []const u8) void {
    self.deecy.gctx_queue_mutex.lock();
    const texture = self.deecy.gctx.createTexture(.{
        .usage = .{ .texture_binding = true, .copy_dst = true },
        .size = .{
            .width = image_width,
            .height = image_height,
            .depth_or_array_layers = 1,
        },
        .format = .bgra8_unorm,
        .mip_level_count = 1,
    });
    const view = self.deecy.gctx.createTextureView(texture, .{});
    self.deecy.gctx.queue.writeTexture(
        .{ .texture = self.deecy.gctx.lookupResource(texture).? },
        .{ .bytes_per_row = 4 * image_width, .rows_per_image = image_height },
        .{ .width = image_width, .height = image_height },
        u8,
        image,
    );
    self.deecy.gctx_queue_mutex.unlock();

    {
        self.disc_files_mutex.lock();
        defer self.disc_files_mutex.unlock();
        for (self.disc_files.items) |*entry| {
            if (std.mem.eql(u8, entry.path, path)) {
                entry.texture = texture;
                entry.view = view;
                return;
            }
        }
    }

    ui_log.err("Failed to find disc entry for '{s}'", .{path});
    self.deecy.gctx_queue_mutex.lock();
    defer self.deecy.gctx_queue_mutex.unlock();
    self.deecy.gctx.releaseResource(view);
    self.deecy.gctx.releaseResource(texture);
}

fn get_game_image(self: *@This(), path: []const u8, cache: ?*GameInfoCache) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var disc = Disc.init(allocator, path) catch |err| {
        ui_log.err("Failed to load disc '{s}': {t}", .{ path, err });
        if (cache) |c| c.add(path, .{ .width = 0, .height = 0 }, null) catch {};
        return;
    };
    defer disc.deinit(allocator);

    const tex_buffer: []u8 = allocator.alloc(u8, 1024 * 1024) catch |err| {
        ui_log.err("Failed to allocate texture buffer: {t}", .{err});
        return;
    };
    defer allocator.free(tex_buffer);

    if (disc.load_file("0GDTEX.PVR;1", tex_buffer)) |len| {
        if (PVRFile.decode(allocator, tex_buffer[0..len])) |result| {
            defer result.deinit(allocator);
            self.update_game_info(path, result.width, result.height, result.bgra);
            if (cache) |c| c.add(path, .{ .width = result.width, .height = result.height }, result.bgra) catch {};
        } else |err| {
            ui_log.err(termcolor.red("Failed to decode 0GDTEX.PVR for '{s}': {t}"), .{ path, err });
            if (cache) |c| c.add(path, .{ .width = 0, .height = 0 }, null) catch {};
        }
    } else |err| {
        ui_log.info("Failed to find 0GDTEX.PVR for '{s}': {t}", .{ path, err });
        if (cache) |c| c.add(path, .{ .width = 0, .height = 0 }, null) catch {};
    }
}

const lz4 = @import("lz4");

const GameInfoCache = struct {
    const Signature: u32 = 0x43444247;
    const Version: u32 = 2;

    const Entry = struct {
        path: []const u8,
        name: []const u8,
        image_size: struct { width: u32, height: u32 },
        image: ?[]const u8,
    };

    map: std.StringHashMap(Entry),

    _path: []const u8,
    _mutex: std.Thread.Mutex = .{},
    _arena: std.heap.ArenaAllocator,

    pub fn create(allocator: std.mem.Allocator) !*@This() {
        var r = try allocator.create(@This());
        r.* = .{
            .map = undefined,
            ._path = undefined,
            ._arena = std.heap.ArenaAllocator.init(allocator),
        };
        r.map = std.StringHashMap(Entry).init(r._arena.allocator());
        r._path = try get_path(r._arena.allocator());
        return r;
    }

    pub fn destroy(self: *@This(), allocator: std.mem.Allocator) void {
        self._arena.deinit();
        allocator.destroy(self);
    }

    fn get_path(allocator: std.mem.Allocator) ![]const u8 {
        return try std.fs.path.join(allocator, &[_][]const u8{ HostPaths.get_userdata_path(), "game_info_cache" });
    }

    pub fn load(self: *@This()) !void {
        var file = try std.fs.cwd().openFile(self._path, .{});
        defer file.close();
        const file_size = try file.getEndPos();
        const data = try file.readToEndAllocOptions(self._arena.allocator(), file_size, file_size, .@"8", null);
        defer self._arena.allocator().free(data);
        try self.deserialize(data);
    }

    pub fn add(self: *@This(), path: []const u8, image_size: struct { width: u32, height: u32 }, image: ?[]const u8) !void {
        self._mutex.lock();
        defer self._mutex.unlock();
        const arena_alloc = self._arena.allocator();
        const duped_path = try arena_alloc.dupe(u8, path);
        try self.map.put(duped_path, .{
            .path = duped_path,
            .name = duped_path,
            .image_size = .{ .width = image_size.width, .height = image_size.height },
            .image = if (image) |i| try arena_alloc.dupe(u8, i) else null,
        });
    }

    pub fn save_to_disk(self: *@This()) !void {
        var file = try std.fs.cwd().createFile(self._path, .{});
        defer file.close();

        var allocating_writer = std.Io.Writer.Allocating.init(self._arena.allocator());
        defer allocating_writer.deinit();
        var uncompressed_writer = &allocating_writer.writer;

        try uncompressed_writer.writeInt(u32, self.map.count(), .little);
        var it = self.map.iterator();
        while (it.next()) |entry| {
            const value = entry.value_ptr.*;
            try write_string(uncompressed_writer, value.path);
            try write_string(uncompressed_writer, value.name);
            try uncompressed_writer.writeInt(u32, value.image_size.width, .little);
            try uncompressed_writer.writeInt(u32, value.image_size.height, .little);
            if (value.image) |image| try uncompressed_writer.writeAll(image);
        }
        try uncompressed_writer.flush();

        var uncompressed = allocating_writer.toArrayList();
        defer uncompressed.deinit(self._arena.allocator());

        const compressed = try lz4.Standard.compress(self._arena.allocator(), uncompressed.items);
        defer self._arena.allocator().free(compressed);

        const buffer = try self._arena.allocator().alloc(u8, 4 * 1024);
        defer self._arena.allocator().free(buffer);
        var file_writer = file.writer(buffer);
        var writer = &file_writer.interface;

        try writer.writeInt(u32, Signature, .little);
        try writer.writeInt(u32, Version, .little);
        try writer.writeInt(u32, @intCast(uncompressed.items.len), .little);
        try writer.writeAll(compressed);

        try writer.flush();
    }

    fn deserialize(self: *@This(), data: []const u8) !void {
        var header_reader = std.Io.Reader.fixed(data);

        const signature = try header_reader.takeInt(u32, .little);
        if (signature != Signature) return error.InvalidSignature;
        const version = try header_reader.takeInt(u32, .little);
        if (version != Version) return error.IncompatibleVersion;

        const uncompressed_size = try header_reader.takeInt(u32, .little);

        const decompressed = try lz4.Standard.decompress(self._arena.allocator(), data[header_reader.seek..], uncompressed_size);
        errdefer self._arena.allocator().free(decompressed); // Kept alive in success path: Entries will refer to it.

        if (decompressed.len != uncompressed_size) return error.UnexpectedDecompressedSize;

        var reader = std.Io.Reader.fixed(decompressed);

        const count = try reader.takeInt(u32, .little);
        try self.map.ensureTotalCapacity(count);

        for (0..count) |_| {
            const path = try read_string(&reader);
            const name = try read_string(&reader);
            const image_size_width = try reader.takeInt(u32, .little);
            const image_size_height = try reader.takeInt(u32, .little);
            const image_byte_size = image_size_width * image_size_height * 4;
            if (reader.buffer[reader.seek..].len < image_byte_size) return error.EndOfStream;
            const image = if (image_size_width > 0 and image_size_height > 0) reader.buffer[reader.seek..][0..image_byte_size] else null;
            reader.toss(image_byte_size);
            self.map.putAssumeCapacity(path, .{
                .path = path,
                .name = name,
                .image_size = .{ .width = image_size_width, .height = image_size_height },
                .image = image,
            });
        }
    }

    fn read_string(reader: *std.Io.Reader) ![]const u8 {
        const size = try reader.takeInt(u32, .little);
        if (reader.buffer[reader.seek..].len < size) return error.EndOfStream;
        const string = reader.buffer[reader.seek..][0..size];
        reader.toss(size);
        return string;
    }

    fn write_string(writer: *std.Io.Writer, string: []const u8) !void {
        try writer.writeInt(u32, @intCast(string.len), .little);
        try writer.writeAll(string);
    }
};

/// Locks gctx_queue_mutex.
pub fn refresh_games(self: *@This()) !void {
    if (self.deecy.config.game_directory) |dir_path| {
        const start = std.time.milliTimestamp();
        defer ui_log.info("Checked {d} disc files in {d}ms", .{ self.disc_files.items.len, std.time.milliTimestamp() - start });

        var cache = try GameInfoCache.create(self.allocator);
        defer cache.destroy(self.allocator);
        cache.load() catch |err|
            ui_log.err(termcolor.red("Failed to load game info cache: {t}"), .{err});

        {
            var tmp_disc_files: std.ArrayList(GameFile) = .empty;
            errdefer tmp_disc_files.deinit(self.allocator);

            var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch |err| {
                ui_log.err(termcolor.red("Failed to open game directory: {t}"), .{err});
                return;
            };
            defer dir.close();
            var walker = try dir.walk(self.allocator);
            defer walker.deinit();

            while (try walker.next()) |entry| {
                if (entry.kind == .file and (std.mem.endsWith(u8, entry.path, ".gdi") or std.mem.endsWith(u8, entry.path, ".cdi") or std.mem.endsWith(u8, entry.path, ".chd"))) {
                    const path = try std.fs.path.joinZ(self.allocator, &[_][]const u8{ dir_path, entry.path });
                    errdefer self.allocator.free(path);
                    const name = try self.allocator.dupeZ(u8, entry.basename);
                    errdefer self.allocator.free(name);
                    try tmp_disc_files.append(self.allocator, .{
                        .name = name,
                        .path = path,
                        .texture = null,
                        .view = null,
                    });
                }
            }

            std.mem.sort(GameFile, tmp_disc_files.items, {}, GameFile.sort);

            self.deecy.gctx_queue_mutex.lock();
            defer self.deecy.gctx_queue_mutex.unlock();
            self.disc_files_mutex.lock();
            defer self.disc_files_mutex.unlock();
            for (self.disc_files.items) |*entry| entry.free(self.allocator, self.deecy.gctx);
            self.disc_files.deinit(self.allocator);
            self.disc_files = tmp_disc_files;
        }

        var pool: std.Thread.Pool = undefined;
        try pool.init(.{ .allocator = self.allocator });
        defer pool.deinit();

        var wait_group = std.Thread.WaitGroup{};

        for (0..self.disc_files.items.len) |file_idx| { // NOTE: I want to prioritize the first items on the list, and spawning jobs in the reverse order seem to do the trick for some reason ¯\_(ツ)_/¯
            const idx = self.disc_files.items.len - 1 - file_idx;
            if (cache.map.get(self.disc_files.items[idx].path)) |entry| {
                if (entry.image) |image|
                    self.update_game_info(entry.path, entry.image_size.width, entry.image_size.height, image);
            } else {
                pool.spawnWg(&wait_group, get_game_image, .{ self, self.disc_files.items[idx].path, cache });
            }
        }

        wait_group.wait();

        try cache.save_to_disk();
    }
}

fn menu_from_enum(comptime name: [:0]const u8, value: anytype, options: struct { enabled: bool = true }) bool {
    if (@typeInfo(@TypeOf(value)) != .pointer or @typeInfo(@TypeOf(value.*)) != .@"enum")
        @compileError("menu_from_enum: value must be a pointer to an enum, got: " ++ @typeName(@TypeOf(value)));
    const T = @TypeOf(value.*);
    var changed = false;
    if (zgui.beginMenu(name, options.enabled)) {
        inline for (@typeInfo(T).@"enum".fields) |field| {
            const v: T = @enumFromInt(field.value);
            if (zgui.menuItem(field.name, .{ .selected = value.* == v })) {
                value.* = v;
                changed = true;
            }
        }
        zgui.endMenu();
    }
    return changed;
}

pub fn draw(self: *@This()) !void {
    const d = self.deecy;
    var error_popup_to_open: [:0]const u8 = "";

    if (zgui.beginMainMenuBar()) {
        if (zgui.beginMenu("File", true)) {
            if (zgui.menuItem("Load Disc", .{})) {
                const was_running = d.running;
                if (was_running) d.pause();

                // Workaround for fullscreen on Windows: The dialog appears behind the fullscreen window and can't be interacted with.
                const exit_fullscreen = builtin.os.tag == .windows and self.deecy.config.fullscreen;
                if (exit_fullscreen) self.deecy.toggle_fullscreen();
                defer if (exit_fullscreen) self.deecy.toggle_fullscreen();

                const open_path = try nfd.openFileDialog("gdi,cdi,chd", null);

                if (open_path) |path| {
                    defer nfd.freePath(path);
                    d.load_and_start(path) catch |err| {
                        switch (err) {
                            error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                            else => {
                                ui_log.err("Failed to load disc: {t}", .{err});
                                error_popup_to_open = "Unknown error";
                            },
                        }
                    };
                }
            }
            zgui.separator();
            if (zgui.menuItem("Exit", .{})) {
                d.pause();
                zglfw.setWindowShouldClose(d.window, true);
            }
            zgui.endMenu();
        }

        if (zgui.beginMenu("Dreamcast", true)) {
            if (!d.running) {
                if (zgui.menuItem("Start", .{ .shortcut = "Space" }))
                    d.start();
            } else {
                if (zgui.menuItem("Pause", .{ .shortcut = "Space" }))
                    d.pause();
            }
            if (zgui.menuItem("Reset", .{}))
                try d.reset();
            if (zgui.menuItem("Stop", .{}))
                try d.stop();
            zgui.separator();

            var realtime = self.deecy.realtime;
            if (zgui.checkbox("Realtime", .{ .v = &realtime }))
                self.deecy.set_realtime(realtime);
            zgui.separator();

            if (menu_from_enum("Region", &d.config.region, .{ .enabled = !d.running })) {
                try d.dc.set_region(d.config.region.to_dreamcast());
            }
            zgui.separator();

            if (menu_from_enum("Cable", &d.config.video_cable, .{})) {
                d.dc.cable_type = d.config.video_cable.to_dreamcast();
            }
            zgui.separator();

            if (zgui.beginMenu("Save States", true)) {
                inline for (0..4) |i| {
                    if (zgui.menuItem("Save Slot " ++ std.fmt.comptimePrint("{d}", .{i}), .{ .shortcut = std.fmt.comptimePrint("F{d}", .{i + 1}) })) {
                        d.save_state(i) catch |err| {
                            ui_log.err("Failed to save state: {}", .{err});
                        };
                    }
                }
                zgui.separator();
                inline for (0..4) |i| {
                    if (zgui.menuItem("Load Slot " ++ std.fmt.comptimePrint("{d}", .{i}), .{ .enabled = d.save_state_slots[i], .shortcut = std.fmt.comptimePrint("F{d}", .{i + 5}) })) {
                        d.load_state(i) catch |err| {
                            ui_log.err("Failed to load state: {}", .{err});
                        };
                    }
                }
                zgui.endMenu();
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Drive", true)) {
            if (d.dc.gdrom.disc) |*disc| {
                if (disc.get_product_name()) |name| {
                    zgui.textColored(.{ 1.0, 1.0, 1.0, 0.75 }, "Disc: {s}", .{name});
                } else {
                    zgui.textColored(.{ 1.0, 1.0, 1.0, 0.75 }, "Unknown Disc", .{});
                }
            } else {
                zgui.textColored(.{ 1.0, 1.0, 1.0, 0.5 }, "No Disc", .{});
            }
            zgui.separator();
            if (zgui.menuItem("Swap Disc", .{})) {
                // Workaround for fullscreen on Windows: The dialog appears behind the fullscreen window and can't be interacted with.
                const exit_fullscreen = builtin.os.tag == .windows and self.deecy.config.fullscreen;
                if (exit_fullscreen) self.deecy.toggle_fullscreen();
                defer if (exit_fullscreen) self.deecy.toggle_fullscreen();

                const open_path = try nfd.openFileDialog("gdi,cdi,chd", null);

                const was_running = d.running;
                if (was_running) d.pause();

                if (open_path) |path| err_brk: {
                    defer nfd.freePath(path);
                    // TODO! Emulate opening the tray and inserting a new disc.
                    d.load_disc(path) catch |err| {
                        ui_log.err("Failed to load disc: {t}", .{err});
                        self.last_error = "Failed to load disc.";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };
                    d.dc.gdrom.state = .Open;
                    if (was_running)
                        d.start();
                }
            }
            if (zgui.menuItem("Open Tray", .{ .enabled = d.dc.gdrom.state != .Open })) {
                d.dc.gdrom.state = .Open;
            }
            if (zgui.menuItem("Remove Disc", .{ .enabled = d.dc.gdrom.disc != null })) {
                const was_running = d.running;
                if (was_running) d.pause();
                d.dc.gdrom.disc.?.deinit(d._allocator);
                d.dc.gdrom.disc = null;
                if (d.dc.gdrom.state != .Open)
                    d.dc.gdrom.state = .Empty;
                if (was_running)
                    d.start();
            }
            if (zgui.menuItem("Close Tray", .{ .enabled = d.dc.gdrom.state == .Open })) {
                if (d.dc.gdrom.disc == null) {
                    d.dc.gdrom.state = .Empty;
                } else {
                    d.dc.gdrom.state = .Standby;
                }
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Settings", true)) {
            _ = menu_from_enum("Performance Overlay", &d.config.performance_overlay, .{});
            if (zgui.menuItem("Display VMUs", .{ .selected = d.config.display_vmus })) {
                d.config.display_vmus = !d.config.display_vmus;
            }
            zgui.separator();
            if (zgui.menuItem("Debug Menu", .{ .selected = d.config.display_debug_ui, .shortcut = "D" })) {
                d.config.display_debug_ui = !d.config.display_debug_ui;
            }
            zgui.endMenu();
        }
        zgui.endMainMenuBar();
    }

    if (!d.running and d.dc.gdrom.disc == null and !self.binary_loaded)
        self.draw_game_library() catch |err| {
            // Most likely due to game loading, and not actually drawing the UI.
            switch (err) {
                error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                else => {
                    error_popup_to_open = "Unknown error";
                    ui_log.err(termcolor.red("Error: {t}"), .{err});
                },
            }
        };

    zgui.setNextWindowSize(.{ .w = 290, .h = 800, .cond = .first_use_ever });
    zgui.setNextWindowPos(.{ .x = 1408, .y = 32, .cond = .first_use_ever });
    if (zgui.begin("Settings", .{})) {
        if (zgui.beginTabBar("SettingsTabBar", .{})) {
            if (zgui.beginTabItem("Renderer", .{})) {
                var fullscreen = self.deecy.config.fullscreen;
                if (zgui.checkbox("Fullscreen", .{ .v = &fullscreen })) {
                    self.deecy.toggle_fullscreen();
                }
                zgui.text("Curent Resolution: {d}x{d}", .{ d.renderer.resolution.width, d.renderer.resolution.height });
                zgui.setNextItemWidth(128.0);
                var resolution: enum(u8) { Native = 1, x2 = 2, x3 = 3, x4 = 4, x5 = 5 } = @enumFromInt(d.renderer.resolution.width / Deecy.Renderer.NativeResolution.width);
                if (zgui.comboFromEnum("Resolution", &resolution)) {
                    d.config.renderer.internal_resolution_factor = @intFromEnum(resolution);
                    // NOTE: This might not be the best idea to do this here without explicit synchronization but... This has worked flawlessly so far.
                    d.renderer.resolution = .{ .width = Deecy.Renderer.NativeResolution.width * @intFromEnum(resolution), .height = Deecy.Renderer.NativeResolution.height * @intFromEnum(resolution) };
                    const fb_size = d.window.getFramebufferSize();
                    d.renderer.on_inner_resolution_change(@intCast(fb_size[0]), @intCast(fb_size[1]), d.config.renderer.display_mode, d.config.renderer.scaling_filter);
                    // Force a re-render if we're paused
                    if (!d.running)
                        try d.renderer.render(&d.dc.gpu, false);
                }
                zgui.setNextItemWidth(128.0);
                if (zgui.comboFromEnum("Display Mode", &d.config.renderer.display_mode)) {
                    const fb_size = d.window.getFramebufferSize();
                    d.renderer.update_blit_to_screen_vertex_buffer(@intCast(fb_size[0]), @intCast(fb_size[1]), d.config.renderer.display_mode);
                }
                zgui.setNextItemWidth(128.0);
                if (zgui.comboFromEnum("Scaling Filter", &d.config.renderer.scaling_filter)) {
                    d.renderer.set_scaling_filter(d.config.renderer.scaling_filter);
                }
                zgui.separator();

                {
                    const static = struct {
                        var available_modes_buffer: [@typeInfo(zgpu.wgpu.PresentMode).@"enum".fields.len]zgpu.wgpu.PresentMode = @splat(.undefined);
                        var available_modes_count: u32 = 0;
                        var present_modes: []const zgpu.wgpu.PresentMode = &.{};
                        pub fn init(deecy: *Deecy) !void {
                            const capabilities = try deecy.gctx.surface.getCapabilities(deecy.gctx.adapter);
                            defer capabilities.deinit();
                            available_modes_count = @intCast(capabilities.getPresentModes().len);
                            for (capabilities.getPresentModes(), 0..) |present_mode, idx|
                                available_modes_buffer[idx] = present_mode;
                            present_modes = available_modes_buffer[0..available_modes_count];
                        }
                    };
                    if (Once(@src())) try static.init(d);

                    zgui.setNextItemWidth(128.0);
                    if (zgui.beginCombo("Present Mode", .{ .preview_value = @tagName(Deecy.PresentMode.fromWGPU(d.gctx.present_mode)) })) {
                        for (static.present_modes) |present_mode| {
                            if (zgui.selectable(@tagName(Deecy.PresentMode.fromWGPU(present_mode)), .{ .selected = d.gctx.present_mode == present_mode })) {
                                d.gctx.present_mode = present_mode;
                                d.config.present_mode = Deecy.PresentMode.fromWGPU(d.gctx.present_mode);
                                const fb_size = d.window.getFramebufferSize();
                                d.gctx.surface.configure(.{
                                    .device = d.gctx.device,
                                    .format = zgpu.GraphicsContext.surface_texture_format,
                                    .usage = .{ .render_attachment = true },
                                    .width = @intCast(fb_size[0]),
                                    .height = @intCast(fb_size[1]),
                                    .present_mode = d.gctx.present_mode,
                                });
                            }
                        }
                        zgui.endCombo();
                    }
                    zgui.separator();
                }

                zgui.text("Experimental settings", .{});
                zgui.setNextItemWidth(128.0);
                _ = zgui.comboFromEnum("Frame Limiter", &d.config.frame_limiter);
                _ = zgui.checkbox("Framebuffer Emulation", .{ .v = &d.renderer.ExperimentalFramebufferEmulation });
                _ = zgui.checkbox("Render to Texture", .{ .v = &d.renderer.ExperimentalRenderToTexture });
                _ = zgui.checkbox("Render to Guest VRAM", .{ .v = &d.renderer.ExperimentalRenderToVRAM });
                _ = zgui.checkbox("Clamp Sprites UVs", .{ .v = &d.renderer.ExperimentalClampSpritesUVs });
                _ = zgui.checkbox("Render on Emulation Thread", .{ .v = &d.renderer.ExperimentalRenderOnEmulationThread });
                _ = zgui.checkbox("Use Pipeline Cache (Restart Required)", .{ .v = &d.config.enable_dawn_pipeline_cache });
                if (builtin.mode == .Debug) {
                    if (zgui.button("Reset Pipeline Cache", .{})) {
                        try @import("pipeline_cache.zig").clear(d._allocator);
                    }
                }
                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Controls", .{})) {
                var per_game_vmu = d.config.per_game_vmu;
                if (zgui.checkbox("Per-Game VMU", .{ .v = &per_game_vmu })) {
                    try d.set_per_game_vmu(per_game_vmu);
                }

                var available_controllers: std.ArrayList(struct { id: ?zglfw.Joystick, name: [:0]const u8 }) = try .initCapacity(self.allocator, zglfw.Joystick.maximum_supported + 1);
                defer available_controllers.deinit(self.allocator);

                available_controllers.appendAssumeCapacity(.{ .id = null, .name = "None" });

                for (0..zglfw.Joystick.maximum_supported) |idx| {
                    const joystick: zglfw.Joystick = @enumFromInt(idx);
                    if (joystick.isPresent()) {
                        if (joystick.asGamepad()) |gamepad| {
                            available_controllers.appendAssumeCapacity(.{ .id = joystick, .name = gamepad.getName() });
                        }
                    }
                }

                const static = struct {
                    var VMUFilenamesInputBuffers: [4][2][64]u8 = @splat(@splat(@splat(0)));
                };
                if (Once(@src())) {
                    for (0..4) |i| {
                        for (0..2) |slot| {
                            if (d.config.controllers[i].subperipherals[slot] == .VMU) {
                                const vmu_config = d.config.controllers[i].subperipherals[slot].VMU;
                                const filename_length = @min(vmu_config.filename.len, static.VMUFilenamesInputBuffers[i][slot].len - 1);
                                @memcpy(static.VMUFilenamesInputBuffers[i][slot][0..filename_length], vmu_config.filename[0..filename_length]);
                            } else {
                                @memcpy(static.VMUFilenamesInputBuffers[i][slot][0..Deecy.DefaultVMUPaths[i][slot].len], Deecy.DefaultVMUPaths[i][slot]);
                            }
                        }
                    }
                }
                if (zgui.beginTabBar("Ports TabBar", .{})) {
                    inline for (0..4) |port| {
                        if (zgui.beginTabItem("Port " ++ &[1]u8{'A' + port}, .{})) {
                            zgui.pushIntId(port);
                            defer zgui.popId();

                            var connected: bool = d.dc.maple.ports[port].main != null;
                            if (zgui.checkbox("Plugged in", .{ .v = &connected })) {
                                try d.enable_controller(port, connected);
                            }
                            if (d.dc.maple.ports[port].main) |*peripheral| {
                                var gamepad_id: ?zglfw.Gamepad = null;
                                if (d.controllers[port]) |j| {
                                    if (j.id.isPresent())
                                        gamepad_id = j.id.asGamepad();
                                }
                                const name = if (gamepad_id) |gamepad| gamepad.getName() else "None";
                                if (zgui.beginCombo("Device", .{ .preview_value = name })) {
                                    for (available_controllers.items, 0..) |item, index| {
                                        if (available_controllers.items[index].id) |id| {
                                            if (zgui.selectable(item.name, .{ .selected = d.controllers[port] != null and d.controllers[port].?.id == id }))
                                                d.controllers[port] = .{ .id = id };
                                        } else {
                                            if (zgui.selectable(item.name, .{ .selected = d.controllers[port] == null }))
                                                d.controllers[port] = null;
                                        }
                                    }
                                    zgui.endCombo();
                                }
                                if (d.controllers[port]) |*j| {
                                    _ = zgui.sliderFloat("Deadzone", .{ .v = &j.deadzone, .min = 0.0, .max = 1.0, .flags = .{} });
                                }
                                switch (peripheral.*) {
                                    .Controller => try @import("ui/controller_settings.zig").draw_controller_settings(d, port),
                                    else => {},
                                }
                            }
                            if (zgui.collapsingHeader("Expansion slots", .{ .default_open = true })) {
                                if (d.dc.maple.ports[port].main) |_| {
                                    inline for (0..2) |slot| {
                                        zgui.pushIntId(slot);
                                        defer zgui.popId();
                                        var peripheral_type = std.meta.activeTag(d.config.controllers[port].subperipherals[slot]);
                                        if (zgui.comboFromEnum("#" ++ &[1]u8{'0' + slot}, &peripheral_type)) {
                                            d.deinit_peripheral(port, slot);
                                            d.config.controllers[port].subperipherals[slot] = switch (peripheral_type) {
                                                .None => .None,
                                                .VMU => .{ .VMU = .{ .filename = Deecy.DefaultVMUPaths[port][slot] } },
                                                .VibrationPack => .VibrationPack,
                                            };
                                            try d.init_peripheral(port, slot);
                                        }
                                        zgui.indent(.{});
                                        defer zgui.unindent(.{});
                                        if (d.dc.maple.ports[port].subperipherals[slot]) |*s| {
                                            switch (s.*) {
                                                .VMU => |vmu| {
                                                    zgui.textColored(.{ 1.0, 1.0, 1.0, 0.75 }, "Loaded: {s}", .{vmu.backing_file_path});
                                                    if (d.config.controllers[port].subperipherals[slot] == .VMU) {
                                                        const vmu_config = &d.config.controllers[port].subperipherals[slot].VMU;
                                                        if (vmu_config.filename.len < static.VMUFilenamesInputBuffers[port][slot].len - 1) {
                                                            const c_str: [:0]u8 = static.VMUFilenamesInputBuffers[port][slot][0 .. static.VMUFilenamesInputBuffers[port][slot].len - 1 :0];
                                                            _ = zgui.inputText("##Filename", .{ .buf = c_str });
                                                            zgui.sameLine(.{});
                                                            if (zgui.button("Load", .{})) {
                                                                d.deinit_peripheral(port, slot);
                                                                vmu_config.filename = static.VMUFilenamesInputBuffers[port][slot][0..std.mem.indexOfSentinel(u8, 0, c_str)];
                                                                try d.init_peripheral(port, slot);
                                                            }
                                                        }
                                                    }
                                                },
                                                else => {},
                                            }
                                        }
                                    }
                                }
                            }
                            zgui.endTabItem();
                        }
                    }

                    zgui.endTabItem();
                }
                zgui.endTabBar();
            }

            if (zgui.beginTabItem("Audio", .{})) {
                if (zgui.sliderFloat("Volume", .{ .v = &d.config.audio_volume, .min = 0.0, .max = 1.0, .flags = .{} })) {
                    try d.audio_device.setMasterVolume(d.config.audio_volume);
                }
                var dsp_emulation = d.dc.aica.dsp_emulation;
                if (zgui.comboFromEnum("DSP", &dsp_emulation)) {
                    const was_running = d.running;
                    if (was_running) d.pause();
                    defer if (was_running) d.start();
                    d.dc.aica.dsp_emulation = dsp_emulation;
                }
                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Shortcuts", .{})) {
                const Shortcuts = @import("ui/shortcuts.zig");
                if (zgui.beginTable("Shortcuts##Table", .{ .column = 4, .flags = .{ .sortable = true, .sort_multi = true, .sizing = .fixed_fit } })) {
                    const Column = enum(u32) { Type = 0, Key, Action };

                    zgui.tableSetupColumn("Type", .{ .user_id = @intFromEnum(Column.Type) });
                    zgui.tableSetupColumn("Key/Button", .{ .user_id = @intFromEnum(Column.Key) });
                    zgui.tableSetupColumn("Action", .{ .user_id = @intFromEnum(Column.Action) });
                    zgui.tableSetupColumn("", .{ .user_id = 4, .flags = .{ .no_sort = true } });
                    zgui.tableHeadersRow();

                    const SortedShortcut = struct {
                        var sort_spec: ?zgui.TableSortSpecs = null;
                        key: Shortcuts.Key,
                        action: Shortcuts.Action.Name,

                        fn column_order(col: Column, a: @This(), b: @This()) std.math.Order {
                            switch (col) {
                                .Type => return std.ascii.orderIgnoreCase(@tagName(std.meta.activeTag(a.key)), @tagName(std.meta.activeTag(b.key))),
                                .Key => {
                                    if (std.meta.activeTag(a.key) != std.meta.activeTag(b.key)) return column_order(.Type, a, b);
                                    return switch (a.key) {
                                        .keyboard => std.ascii.orderIgnoreCase(@tagName(a.key.keyboard.key), @tagName(b.key.keyboard.key)),
                                        .controller => std.ascii.orderIgnoreCase(@tagName(a.key.controller), @tagName(b.key.controller)),
                                    };
                                },
                                .Action => return std.ascii.orderIgnoreCase(@tagName(a.action), @tagName(b.action)),
                            }
                        }

                        pub fn order(_: void, a: @This(), b: @This()) std.math.Order {
                            if (sort_spec) |s| {
                                for (0..@intCast(s.count)) |sort_order| {
                                    for (s.specs[0..@intCast(s.count)]) |spec| {
                                        if (spec.sort_direction != .none and spec.sort_order == sort_order) {
                                            const ord = column_order(@enumFromInt(spec.user_id), a, b);
                                            if (ord != .eq) return switch (spec.sort_direction) {
                                                .ascending => ord,
                                                .descending => ord.invert(),
                                                .none => unreachable,
                                            };
                                            break;
                                        }
                                    }
                                }
                            }
                            return std.math.Order.lt;
                        }

                        pub fn less_than(_: void, a: @This(), b: @This()) bool {
                            return order({}, a, b) == .lt;
                        }
                    };
                    SortedShortcut.sort_spec = zgui.tableGetSortSpecs();
                    var sorted_list = try std.ArrayList(SortedShortcut).initCapacity(d._allocator, d.shortcuts.shortcuts.count());
                    defer sorted_list.deinit(d._allocator);

                    var it = d.shortcuts.shortcuts.iterator();
                    while (it.next()) |entry| {
                        sorted_list.appendAssumeCapacity(.{ .key = entry.key_ptr.*, .action = entry.value_ptr.name });
                    }
                    std.mem.sortUnstable(SortedShortcut, sorted_list.items, {}, SortedShortcut.less_than);

                    var idx: i32 = 0;
                    for (sorted_list.items) |*entry| {
                        zgui.pushIntId(@intCast(idx));
                        defer zgui.popId();
                        idx += 1;

                        zgui.tableNextRow(.{});
                        _ = zgui.tableSetColumnIndex(0);
                        zgui.text("{t}", .{std.meta.activeTag(entry.key)});
                        _ = zgui.tableSetColumnIndex(1);
                        if (zgui.button("E", .{})) {
                            const maybe_key = wait_for.any_button(d);
                            if (maybe_key) |key| {
                                d.shortcuts.remove(entry.key);
                                entry.key = switch (key) {
                                    .keyboard => Shortcuts.Key{ .keyboard = .{ .key = key.keyboard.key, .mods = .from_glfw(key.keyboard.modifiers) } },
                                    .controller => Shortcuts.Key{ .controller = key.controller },
                                };
                                try d.shortcuts.put(entry.key, entry.action);
                            }
                        }
                        zgui.sameLine(.{});
                        zgui.text("{f}", .{entry.key});
                        _ = zgui.tableSetColumnIndex(2);
                        zgui.setNextItemWidth(200.0);
                        if (zgui.comboFromEnum("##Action", &entry.action)) {
                            try d.shortcuts.put(entry.key, entry.action);
                        }
                        _ = zgui.tableSetColumnIndex(3);
                        zgui.pushStyleColor4f(.{ .c = .{ 0.86, 0.12, 0.15, 1.0 }, .idx = .button });
                        defer zgui.popStyleColor(.{ .count = 1 });
                        if (zgui.button("X", .{})) {
                            d.shortcuts.remove(entry.key);
                        }
                    }
                    zgui.endTable();
                }
                zgui.separator();
                {
                    const static = struct {
                        var action = Shortcuts.Action.Name.Screenshot;
                        var key: ?Shortcuts.Key = null;
                    };
                    zgui.setNextItemWidth(200.0);
                    _ = zgui.comboFromEnum("##NewAction", &static.action);
                    zgui.sameLine(.{});
                    if (static.key) |key| {
                        zgui.text("{f}", .{key});
                    } else {
                        zgui.text("(None)", .{});
                    }
                    zgui.sameLine(.{});
                    if (zgui.button("Select Key", .{})) {
                        const maybe_key = wait_for.any_button(d);
                        if (maybe_key) |key| {
                            static.key = switch (key) {
                                .keyboard => Shortcuts.Key{ .keyboard = .{ .key = key.keyboard.key, .mods = .from_glfw(key.keyboard.modifiers) } },
                                .controller => Shortcuts.Key{ .controller = key.controller },
                            };
                        }
                    }
                    zgui.sameLine(.{});
                    zgui.beginDisabled(.{ .disabled = static.key == null });
                    defer zgui.endDisabled();
                    if (zgui.button("Add", .{})) {
                        if (static.key) |key|
                            try d.shortcuts.put(key, static.action);
                    }
                }
                if (zgui.button("Reset to default", .{})) {
                    try d.shortcuts.load_default_shortcuts();
                }
                zgui.endTabItem();
            }

            zgui.endTabBar();
        }
    }
    zgui.end();

    // NOTE: Modals have to be in the same ID stack as the openPopup call :(
    //       Hence the weird workaround.
    if (error_popup_to_open.len > 0) {
        zgui.openPopup(error_popup_to_open, .{});
    }

    if (zgui.beginPopupModal("Error: Missing Flash", .{ .flags = .{ .always_auto_resize = true } })) {
        zgui.text("Failed to load flash. Did you put a copy of 'dc_flash.bin' in 'data/'?", .{});
        if (zgui.button("OK", .{}))
            zgui.closeCurrentPopup();
        zgui.endPopup();
    }
    if (zgui.beginPopupModal("Unknown error", .{})) {
        zgui.text("The console might have more information!", .{});
        if (zgui.button("OK", .{}))
            zgui.closeCurrentPopup();
        zgui.endPopup();
    }
}

/// A few random colors to help differentiate games without images.
const GameColors = [_]u32{ 0xFF222B55, 0xFF553D22, 0xFF2D3D2D, 0xFF4D334D, 0xFF444422, 0xFF224444, 0xFF333366, 0xFF554433, 0xFF3D4D42, 0xFF663333 };

pub fn draw_game_library(self: *@This()) !void {
    const d = self.deecy;
    const bg_color = zgui.colorConvertFloat4ToU32(zgui.getStyle().colors[@intFromEnum(zgui.StyleCol.window_bg)]);
    const title_height = 24;
    const text_padding = .{ 16, 8 };
    const target_width = 4 * 256 + 64;
    const fb_size = d.window.getFramebufferSize();
    zgui.setNextWindowPos(.{ .x = @floatFromInt((@max(target_width, fb_size[0]) - target_width) / 2), .y = 32, .cond = .always });
    zgui.setNextWindowSize(.{ .w = target_width, .h = @floatFromInt(@max(64, fb_size[1]) - 64), .cond = .always });

    defer zgui.end();
    if (zgui.begin("Library", .{ .flags = .{ .no_resize = true, .no_move = true, .no_title_bar = true, .no_docking = true, .no_bring_to_front_on_focus = true } })) {
        zgui.alignTextToFramePadding();
        const static = struct {
            var display_game_search: [63:0]u8 = @splat(0);
            var lowercase_buffer: [64]u8 = @splat(0);
            var lowercase_game_search: []u8 = lowercase_buffer[0..0];
        };
        zgui.setNextItemWidth(150.0);
        if (zgui.inputTextWithHint("##Search", .{ .hint = "Search file...", .buf = @ptrCast(&static.display_game_search), .flags = .{ .auto_select_all = true } })) {
            static.lowercase_buffer = @splat(0);
            for (0..static.display_game_search.len) |i| {
                if (static.display_game_search[i] == 0) {
                    static.lowercase_game_search = static.lowercase_buffer[0..i];
                    break;
                }
                static.lowercase_buffer[i] = std.ascii.toLower(static.display_game_search[i]);
            }
        }
        // TODO: Set active on startup when this API is added to ImGui (https://github.com/ocornut/imgui/issues/3949)
        // if (zgui.isWindowAppearing()) zgui.setActive("##Search");
        zgui.sameLine(.{ .spacing = 24.0 });
        zgui.alignTextToFramePadding();
        if (d.config.game_directory) |dir| {
            zgui.text("Directory: {s}", .{dir});
        } else {
            zgui.text("Directory: None", .{});
        }
        zgui.sameLine(.{ .spacing = 24.0 });
        if (zgui.button("Refresh", .{})) {
            try self.deecy.launch_async(refresh_games, .{self});
        }
        zgui.sameLine(.{});
        if (zgui.button("Change Directory", .{})) {
            // Workaround for fullscreen on Windows: The dialog appears behind the fullscreen window and can't be interacted with.
            const exit_fullscreen = builtin.os.tag == .windows and self.deecy.config.fullscreen;
            if (exit_fullscreen) self.deecy.toggle_fullscreen();
            defer if (exit_fullscreen) self.deecy.toggle_fullscreen();

            const open_path = try nfd.openFolderDialog(null);
            if (open_path) |path| {
                defer nfd.freePath(path);
                if (d.config.game_directory) |old_dir| self.allocator.free(old_dir);
                d.config.game_directory = try self.allocator.dupe(u8, path);
                try self.deecy.launch_async(refresh_games, .{self});
            }
        }

        const draw_list = zgui.getWindowDrawList();

        {
            self.disc_files_mutex.lock();
            defer self.disc_files_mutex.unlock();

            {
                _ = zgui.beginChild("Games", .{});
                defer zgui.endChild();

                const clip_min = zgui.getWindowPos();
                const clip_max = .{ clip_min[0] + zgui.getContentRegionAvail()[0], clip_min[1] + zgui.getContentRegionAvail()[1] };
                draw_list.pushClipRect(.{ .pmin = clip_min, .pmax = clip_max });
                defer draw_list.popClipRect();

                var truncated_name: [28:0]u8 = undefined;
                var lowercase_name: [256]u8 = undefined;
                zgui.pushStyleVar2f(.{ .idx = .frame_padding, .v = .{ 0, 0 } });
                zgui.pushStyleVar2f(.{ .idx = .item_spacing, .v = .{ 8.0, 8.0 } });
                zgui.pushStyleVar1f(.{ .idx = .frame_rounding, .v = 8.0 });
                defer zgui.popStyleVar(.{ .count = 3 });
                var displayed_count: usize = 0;
                for (self.disc_files.items, 0..) |entry, idx| {
                    lowercase_name = @splat(0);
                    for (0..@min(entry.name.len, lowercase_name.len)) |i| lowercase_name[i] = std.ascii.toLower(entry.name[i]);
                    if (static.lowercase_game_search.len > 0 and std.mem.indexOf(u8, &lowercase_name, static.lowercase_game_search) == null) continue;

                    {
                        zgui.beginGroup();
                        defer zgui.endGroup();
                        zgui.pushIntId(@intCast(idx));
                        defer zgui.popId();

                        const cursor_pos = zgui.getCursorScreenPos();
                        if (zgui.button("##Launch", .{ .w = 256.0, .h = 256.0 + 8.0 + title_height }))
                            try d.load_and_start(entry.path);
                        zgui.setCursorScreenPos(.{ cursor_pos[0] + text_padding[0], cursor_pos[1] + text_padding[1] });

                        if (entry.view) |view| {
                            @memset(&truncated_name, 0);
                            @memcpy(truncated_name[0..@min(entry.name.len, truncated_name.len - 1)], entry.name[0..@min(entry.name.len, truncated_name.len - 1)]);
                            zgui.text("{s}", .{truncated_name});
                            zgui.setCursorScreenPos(.{ cursor_pos[0], cursor_pos[1] + text_padding[1] + title_height });
                            zgui.image(.{ .tex_data = null, .tex_id = @enumFromInt(@intFromPtr(((self.deecy.gctx.lookupResource(view).?)))) }, .{ .w = 256, .h = 256 });
                        } else {
                            const p = .{ cursor_pos[0] + 128.0, cursor_pos[1] + text_padding[1] + title_height + 128.0 };
                            draw_list.addCircleFilled(.{ .p = p, .r = 96.0, .col = 0x80FFFFFF, .num_segments = 0 });
                            draw_list.addCircleFilled(.{ .p = p, .r = 92.0, .col = GameColors[entry.name.len % GameColors.len], .num_segments = 0 });
                            draw_list.addCircleFilled(.{ .p = p, .r = 24.0, .col = 0x80FFFFFF, .num_segments = 0 });
                            {
                                zgui.pushFont(null, 130.0);
                                defer zgui.popFont();
                                draw_list.addText(.{ p[0] - 65.0, p[1] - 65.0 }, 0x80808080, "{s}", .{entry.name[0..2]});
                            }
                            draw_list.addCircleFilled(.{ .p = .{ cursor_pos[0] + 128.0, cursor_pos[1] + text_padding[1] + title_height + 128.0 }, .r = 16.0, .col = bg_color, .num_segments = 0 });

                            zgui.pushTextWrapPos(zgui.getCursorPosX() + 256.0 - 2 * text_padding[0]);
                            defer zgui.popTextWrapPos();
                            zgui.textWrapped("{s}", .{entry.name});
                        }
                    }

                    if (zgui.isItemHovered(.{ .for_tooltip = true }) and zgui.beginTooltip()) {
                        zgui.text("{s}", .{entry.name});
                        zgui.endTooltip();
                    }

                    if (displayed_count % 4 != 3) zgui.sameLine(.{});
                    displayed_count += 1;
                }
            }
        }
    }
}
