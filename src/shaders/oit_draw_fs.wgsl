// Modified from https://webgpu.github.io/webgpu-samples/samples/A-buffer - BSD-3-Clause license 

struct Heads {
  fragment_count: atomic<u32>,
  data: array<atomic<u32>>
};

@group(2) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(2) @binding(1) var<storage, read_write> heads: Heads;
@group(2) @binding(2) var<storage, read_write> linked_list: LinkedList;
// @group(2) @binding(3) var opaque_depth_texture: texture_depth_multisampled_2d;
@group(2) @binding(4) var<storage, read_write> modvols: array<VolumesInterfaces>;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
    @location(0) area0_base_color: vec4<f32>,
    @location(1) area0_offset_color: vec4<f32>,
    @location(2) area1_base_color: vec4<f32>,
    @location(3) area1_offset_color: vec4<f32>,
    @location(4) @interpolate(flat) area0_flat_base_color: u32,
    @location(5) @interpolate(flat) area0_flat_offset_color: u32,
    @location(6) @interpolate(flat) area1_flat_base_color: u32,
    @location(7) @interpolate(flat) area1_flat_offset_color: u32,
    @location(8) @interpolate(flat) area0_tex_idx_shading_instr: vec2<u32>,
    @location(9) @interpolate(flat) area1_tex_idx_shading_instr: vec2<u32>,
    @location(10) area0_uv: vec2<f32>,
    @location(11) area1_uv: vec2<f32>,
    @location(12) @interpolate(flat) index: u32,
    @location(13) inverse_z: f32,
) {
    let frag_coords = vec2<i32>(position_clip.xy);
    let opaque_depth = textureLoad(opaque_depth_texture, frag_coords, 0);

    // [Depth Compare Setting] is ignored for Translucent polygons in Auto-sort mode;
    // the comparison must be made on a "Greater or Equal" basis.
    if position_clip.z < opaque_depth { discard; }

    // The index in the heads buffer corresponding to the head data for the fragment at the current location.
    let heads_index = (u32(frag_coords.y) - oit_uniforms.start_y) * oit_uniforms.target_width + u32(frag_coords.x);
    
    // Determine in which area the fragment is (if applicable) and compute its color.
    var area = false;
    var shadow_factor: f32 = 1.0;
    let shadow_bit = extractBits(area0_tex_idx_shading_instr[1], 22, 1) == 1;
    let volume_bit = extractBits(area0_tex_idx_shading_instr[1], 24, 1) == 1;
    
    if(shadow_bit) {
        var temp_area = false;
        var curr_depth_interface = 0u;
        while curr_depth_interface < MaxVolumesInterfaces && modvols[heads_index].interfaces[curr_depth_interface] >= 0.0 && modvols[heads_index].interfaces[curr_depth_interface] < position_clip.z {
            temp_area = !temp_area;
            curr_depth_interface += 1;
        }
        if volume_bit {
            area = temp_area;
        } else if temp_area {
            shadow_factor = uniforms.fpu_shad_scale;
        }
    }
    
    let texture_index_palette = select(area0_tex_idx_shading_instr[0], area1_tex_idx_shading_instr[0], area);
    let shading_instructions = select(area0_tex_idx_shading_instr[1], area1_tex_idx_shading_instr[1], area);
    let area0_gouraud = extractBits(area0_tex_idx_shading_instr[1], 23, 1) == 1;
    let area1_gouraud = extractBits(area1_tex_idx_shading_instr[1], 23, 1) == 1;

    let base_color = select(
            select(unpack4x8unorm(area0_flat_base_color).zyxw, area0_base_color, area0_gouraud),
            select(unpack4x8unorm(area1_flat_base_color).zyxw, area1_base_color, area1_gouraud),
        area);
    let offset_color = select(
            select(unpack4x8unorm(area0_flat_offset_color).zyxw, area0_offset_color, area0_gouraud),
            select(unpack4x8unorm(area1_flat_offset_color).zyxw, area1_offset_color, area1_gouraud),
        area);
    let uv = select(area0_uv, area1_uv, area);
    let duvdx = dpdx(uv);
    let duvdy = dpdy(uv);
    let texture_index: u32 = texture_index_palette & 0xFFFF;
    let palette_instructions: u32 = texture_index_palette >> 16;

    var color = area_color(
        base_color,
        offset_color,
        uv,
        duvdx,
        duvdy,
        texture_index,
        palette_instructions,
        shading_instructions,
        1.0 / inverse_z,
        false
    );
    color = vec4<f32>(shadow_factor * color.rgb, color.a);

    let blend_modes = extractBits(shading_instructions, 10, 6);

    // Add the fragment to the linked list
    
    // The index in the linked list buffer at which to store the new fragment
    let frag_index = atomicAdd(&heads.fragment_count, 1u);
    
    // If we run out of space to store the fragments, we just lose them
    if frag_index < oit_uniforms.max_fragments {
        let last_head = atomicExchange(&heads.data[heads_index], frag_index);
        linked_list.data[frag_index].depth = position_clip.z;
        linked_list.data[frag_index].color = pack4x8unorm(color);
        linked_list.data[frag_index].index_and_blend_modes = (index << 6) | blend_modes;
        linked_list.data[frag_index].next = last_head;
    }
}