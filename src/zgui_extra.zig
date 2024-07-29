const std = @import("std");
const zgui = @import("zgui");

pub fn selectEnum(comptime name: [:0]const u8, target: anytype) bool {
    var modified = false;
    if (zgui.beginCombo(name, .{ .preview_value = @tagName(target.*) })) {
        inline for (std.meta.fields(@TypeOf(target.*))) |mode| {
            const value: @TypeOf(target.*) = @enumFromInt(mode.value);
            if (zgui.selectable(mode.name, .{ .selected = target.* == value })) {
                target.* = value;
                modified = true;
            }
        }
        zgui.endCombo();
    }
    return modified;
}
