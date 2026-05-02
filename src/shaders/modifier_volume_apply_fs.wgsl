// @group(0) @binding(0) var area1: texture_2d<f32>;  // NOTE: Real type injected from Zig. Depends on MSAA setting.

@fragment
fn main(
    @builtin(position) position: vec4<f32>,
) -> @location(0) vec4<f32> {
    let frag_coords = vec2<i32>(position.xy);
    return textureLoad(area1, frag_coords, 0);
}
