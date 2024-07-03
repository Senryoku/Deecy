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
    @location(4) @interpolate(flat) tex_idx_shading_instr: vec2<u32>,
    @location(5) @interpolate(flat) index: u32,
    @location(6) @interpolate(flat) flat_base_color: vec4<f32>,
    @location(7) @interpolate(flat) flat_offset_color: vec4<f32>,
) -> FragmentOutput {
    let gouraud = ((tex_idx_shading_instr[1] >> 23) & 1) == 1;

    var final_color = fragment_color(
        select(flat_base_color, base_color / inv_w , gouraud), 
        select(flat_offset_color, flat_base_color / inv_w, gouraud), 
        uv / inv_w, 
        tex_idx_shading_instr, 
        inv_w, 
        true
    );

    var output: FragmentOutput;
    output.area0 = final_color.area0;
    output.area1 = final_color.area1;

    output.area0.a = 1.0;
    output.area1.a = 1.0;

    return output;
}
