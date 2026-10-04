const std = @import("std");

/// Calling this with an unique identifier (e.g. `Once(@src())`) will return true only the first time.
/// Useful for initialization, or for logging:
///   ```if (Once(@src())) std.log.info("This is wrong, I won't repeat myself.", .{});```
pub fn Once(comptime id: anytype) bool {
    const static = struct {
        const src = id;
        pub var first: bool = true;
    };
    const value = static.first;
    static.first = false;
    return value;
}

/// Calling this with an unique identifier (e.g. `UpTo(@src(), n)`) will return the number of times it has been called,
/// up to `n`, then return null.
/// Useful for limiting spam while logging:
///   ```if (UpTo(@src(), 10)) |n| std.log.info("This happened {} times.", .{n});```
pub fn UpTo(comptime id: anytype, comptime max: u32) ?u32 {
    const static = struct {
        const src = id;
        pub var count: u32 = 0;
    };
    if (static.count < max) {
        static.count += 1;
        return static.count;
    }
    return null;
}

/// Returns a struct type where all fields from T are optional.
pub fn Partial(comptime T: type) type {
    const info = @typeInfo(T);
    switch (info) {
        .@"struct" => |s| {
            comptime var field_names: []const []const u8 = &[_][]const u8{};
            comptime var field_types: []const type = &[_]type{};
            comptime var field_attrs: []const std.builtin.Type.Struct.FieldAttributes = &[_]std.builtin.Type.Struct.FieldAttributes{};
            inline for (s.field_names, s.field_types, s.field_attrs) |field_name, field_type, field_attr| {
                if (field_attr.@"comptime") @compileError("Partial type cannot contain comptime members");
                const optional_field_type = switch (@typeInfo(field_type)) {
                    .optional => field_type,
                    else => ?field_type,
                };
                const default_value: optional_field_type = null;
                field_names = field_names ++ &[1][]const u8{field_name};
                field_types = field_types ++ &[1]type{optional_field_type};
                field_attrs = field_attrs ++ &[1]std.builtin.Type.Struct.FieldAttributes{.{
                    .@"comptime" = field_attr.@"comptime",
                    .@"align" = field_attr.@"align",
                    .default_value_ptr = &default_value,
                }};
            }
            return @Struct(
                s.layout,
                s.backing_integer,
                field_names,
                field_types[0..field_names.len],
                field_attrs[0..field_names.len],
            );
        },
        else => @compileError("Partial type must be a struct"),
    }
}

/// Converts a Partial(T) to a T, using T fields default values.
pub fn to_complete(comptime T: type, partial: Partial(T)) T {
    var result: T = undefined;
    const ti = @typeInfo(T).@"struct";
    inline for (ti.field_names, ti.field_types, ti.field_attrs) |field_name, field_type, field_attr| {
        @field(result, field_name) = @field(partial, field_name) orelse @as(*const field_type, @ptrCast(@alignCast(field_attr.default_value_ptr))).*;
    }
    return result;
}

pub fn use_wayland(allocator: std.mem.Allocator) bool {
    if (@import("builtin").os.tag == .windows) return false;
    var env_var = std.process.getEnvMap(allocator) catch return false;
    defer env_var.deinit();
    return std.mem.eql(u8, env_var.get("XDG_SESSION_TYPE") orelse "", "wayland");
}

pub fn title_case(comptime input: []const u8) []const u8 {
    comptime var result: []const u8 = "";
    comptime var capitalize_next = true;
    inline for (input) |char| {
        if (char == '_') {
            result = result ++ " ";
            capitalize_next = true;
        } else if (capitalize_next) {
            if (char >= 'a' and char <= 'z') {
                result = result ++ [_]u8{char - ('a' - 'A')};
            } else {
                result = result ++ [_]u8{char};
            }
            capitalize_next = false;
        } else {
            result = result ++ [_]u8{char};
        }
    }
    return result;
}

pub fn title_case_enum(enum_value: anytype) []const u8 {
    switch (enum_value) {
        inline else => |value| return title_case(@tagName(value)),
    }
}
