struct FragmentOutput {
    @location(0) area0: vec4<f32>,
    @location(1) area1: vec4<f32>,
}

@fragment
fn main(
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) area1_base_color: vec4<f32>,
    @location(3) area1_offset_color: vec4<f32>,
    @location(4) @interpolate(flat) flat_base_color: u32,
    @location(5) @interpolate(flat) flat_offset_color: u32,
    @location(6) @interpolate(flat) area1_flat_base_color: u32,
    @location(7) @interpolate(flat) area1_flat_offset_color: u32,
    @location(8) @interpolate(flat) tex_idx_shading_instr: vec2<u32>,
    @location(9) @interpolate(flat) area1_tex_idx_shading_instr: vec2<u32>,
    @location(10) uv: vec2<f32>,
    @location(11) area1_uv: vec2<f32>,
    @location(12) @interpolate(flat) index: u32,
    @location(13) original_z: f32,
) -> FragmentOutput {

    var final_color = fragment_color(
        base_color,
        unpack4x8unorm(flat_base_color).zyxw,
        offset_color,
        unpack4x8unorm(flat_offset_color).zyxw,
        uv,
        tex_idx_shading_instr[0],
        tex_idx_shading_instr[1],
        area1_base_color,
        unpack4x8unorm(area1_flat_base_color).zyxw,
        area1_offset_color,
        unpack4x8unorm(area1_flat_offset_color).zyxw,
        area1_uv,
        area1_tex_idx_shading_instr[0],
        area1_tex_idx_shading_instr[1],
        original_z,
        true
    );

    var output: FragmentOutput;
    output.area0 = final_color.area0;
    output.area1 = final_color.area1;

    // output.area0.a = 1.0;
    // output.area1.a = 1.0;

    return output;
}
