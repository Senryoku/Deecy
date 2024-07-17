@group(0) @binding(0) var<uniform> uniforms: Uniforms;
 
struct VertexOut {
    @builtin(position) position_clip: vec4<f32>,
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) inv_w: f32,
    @location(3) uv: vec2<f32>,
    @location(4) @interpolate(flat) tex_idx_shading_instr: vec2<u32>,
    @location(5) area1_base_color: vec4<f32>,
    @location(6) area1_offset_color: vec4<f32>,
    @location(7) area1_uv: vec2<f32>,
    @location(8) @interpolate(flat) area1_tex_idx_shading_instr: vec2<u32>,
    @location(9) @interpolate(flat) index: u32,
    @location(10) @interpolate(flat) flat_base_color: vec4<f32>, // Probably not the best use of bandwidth, but this beats using a separate pass/shader combination for now. 
    @location(11) @interpolate(flat) flat_offset_color: vec4<f32>,
    @location(12) @interpolate(flat) area1_flat_base_color: vec4<f32>,
    @location(13) @interpolate(flat) area1_flat_offset_color: vec4<f32>,
 }

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) base_color: vec4<f32>,
    @location(2) offset_color: vec4<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) tex_idx_shading_instr: vec2<u32>, // Texture index and Texture control word
    @location(5) area1_base_color: vec4<f32>,
    @location(6) area1_offset_color: vec4<f32>,
    @location(7) area1_uv: vec2<f32>,
    @location(8) area1_tex_idx_shading_instr: vec2<u32>,
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOut {
    var output: VertexOut;

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    if(uniforms.depth_max > uniforms.depth_min) {
        output.position_clip.z = ((1.0 / position.z) - uniforms.depth_min) / (uniforms.depth_max - uniforms.depth_min); // Remap to the [0.0..1.0] range used by WGPU
    } else {
        output.position_clip.z = (1.0 / position.z);
    }
    output.position_clip.w = 1.0;

    let inv_w = position.z;
    
    output.base_color = inv_w * base_color;
    output.offset_color = inv_w * offset_color;

    output.inv_w = inv_w;

    output.uv = inv_w * uv;
    output.tex_idx_shading_instr = tex_idx_shading_instr;
    
    
    output.area1_base_color = inv_w * area1_base_color;
    output.area1_offset_color = inv_w * area1_offset_color;
    output.area1_uv = inv_w * area1_uv;
    output.area1_tex_idx_shading_instr = area1_tex_idx_shading_instr;

    output.index = vertex_index;

    output.flat_base_color = base_color;
    output.flat_offset_color = offset_color;
    output.area1_flat_base_color = area1_base_color;
    output.area1_flat_offset_color = area1_offset_color;

    return output;
}