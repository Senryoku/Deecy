@fragment
fn main(
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) w: f32,
    @location(4) @interpolate(flat) tex: vec2<u32>,
) -> @location(0) vec4<f32> {
    var final_color: vec4<f32> = fragment_color(base_color / w, offset_color / w, uv / w, tex);

    if final_color.a == 0.0 {
        discard;
    }

    return final_color;
}
