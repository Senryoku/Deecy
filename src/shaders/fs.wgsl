struct FragmentOutput {
    @location(0) area0: vec4<f32>,
    @location(1) area1: vec4<f32>,
}

@fragment
fn main(
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) inv_w: f32,
    @location(4) @interpolate(flat) tex: vec2<u32>,
) -> FragmentOutput {
    var final_color = fragment_color(base_color / inv_w, offset_color / inv_w, uv / inv_w, tex, inv_w, true);

    var output: FragmentOutput;
    output.area0 = final_color.area0;
    output.area1 = final_color.area1;

    output.area0.a = 1.0;
    output.area1.a = 1.0;

    return output;
}
