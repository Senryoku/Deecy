@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct StripsMetadata {
    area0_tex_index: u32,
    area0_instructions: u32,
    area1_tex_index: u32,
    area1_instructions: u32,
};

@group(0) @binding(9) var<storage, read> strips_metadata: array<StripsMetadata>;
 
struct VertexOut {
    @builtin(position) position_clip: vec4<f32>,
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) area1_base_color: vec4<f32>,
    @location(3) area1_offset_color: vec4<f32>,
    @location(4) @interpolate(flat) flat_base_color: u32, // Probably not the best use of bandwidth, but this beats using a separate pass/shader combination for now. 
    @location(5) @interpolate(flat) flat_offset_color: u32,
    @location(6) @interpolate(flat) area1_flat_base_color: u32,
    @location(7) @interpolate(flat) area1_flat_offset_color: u32,
    @location(8) @interpolate(flat) tex_idx_shading_instr: vec2<u32>,
    @location(9) @interpolate(flat) area1_tex_idx_shading_instr: vec2<u32>,
    @location(10) uv: vec2<f32>,
    @location(11) area1_uv: vec2<f32>,
    @location(12) @interpolate(flat) index: u32,
    @location(13) inv_w: f32,
 }

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) primitive_index: u32,
    @location(2) packed_base_color: u32,
    @location(3) packed_offset_color: u32,
    @location(4) packed_area1_base_color: u32,
    @location(5) packed_area1_offset_color: u32,
    @location(6) uv: vec2<f32>,
    @location(7) area1_uv: vec2<f32>,
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOut {
    var output: VertexOut;

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    if uniforms.depth_max > uniforms.depth_min {
        output.position_clip.z = ((1.0 / position.z) - uniforms.depth_min) / (uniforms.depth_max - uniforms.depth_min); // Remap to the [0.0..1.0] range used by WGPU
    } else {
        output.position_clip.z = (1.0 / position.z);
    }
    output.position_clip.w = 1.0;

    let inv_w = position.z;

    let metadata = strips_metadata[primitive_index];

    let base_color = unpack4x8unorm(packed_base_color).zyxw; // BGRA => RGBA
    let offset_color = unpack4x8unorm(packed_offset_color).zyxw;
    let area1_base_color = unpack4x8unorm(packed_area1_base_color).zyxw;
    let area1_offset_color = unpack4x8unorm(packed_area1_offset_color).zyxw;

    output.base_color = inv_w * base_color;
    output.offset_color = inv_w * offset_color;
    output.uv = inv_w * uv;
    output.tex_idx_shading_instr = vec2<u32>(metadata.area0_tex_index, metadata.area0_instructions);
    output.flat_base_color = packed_base_color;
    output.flat_offset_color = packed_offset_color;

    output.area1_base_color = inv_w * area1_base_color;
    output.area1_offset_color = inv_w * area1_offset_color;
    output.area1_uv = inv_w * area1_uv;
    output.area1_tex_idx_shading_instr = vec2<u32>(metadata.area1_tex_index, metadata.area1_instructions);
    output.area1_flat_base_color = packed_area1_base_color;
    output.area1_flat_offset_color = packed_area1_offset_color;

    output.index = vertex_index;
    output.inv_w = inv_w;

    return output;
}