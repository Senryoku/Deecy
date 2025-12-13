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
