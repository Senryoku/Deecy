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

/// Returns a struct type where all fields from T are optional.
pub fn Partial(comptime T: type) type {
    const info = @typeInfo(T);
    switch (info) {
        .@"struct" => |s| {
            comptime var fields: []const std.builtin.Type.StructField = &[_]std.builtin.Type.StructField{};
            for (s.fields) |field| {
                if (field.is_comptime) @compileError("Partial type cannot contain comptime members");
                const field_type = switch (@typeInfo(field.type)) {
                    .optional => field.type,
                    else => ?field.type,
                };
                const default_value: field_type = null;
                const optional_field = [_]std.builtin.Type.StructField{.{
                    .alignment = field.alignment,
                    .default_value_ptr = @ptrCast(@alignCast(&default_value)),
                    .is_comptime = false,
                    .name = field.name,
                    .type = field_type,
                }};
                fields = fields ++ optional_field;
            }
            return @Type(.{ .@"struct" = .{
                .backing_integer = s.backing_integer,
                .decls = &[_]std.builtin.Type.Declaration{},
                .fields = fields,
                .is_tuple = s.is_tuple,
                .layout = s.layout,
            } });
        },
        else => @compileError("Partial type must be a struct"),
    }
}

/// Converts a Partial(T) to a T, using T fields default values.
pub fn to_complete(comptime T: type, partial: Partial(T)) T {
    var result: T = undefined;
    inline for (comptime std.meta.fields(T)) |field| {
        @field(result, field.name) = @field(partial, field.name) orelse @as(*const field.type, @ptrCast(@alignCast(field.default_value_ptr))).*;
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
