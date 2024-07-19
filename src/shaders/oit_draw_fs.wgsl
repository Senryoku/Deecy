// Modified from https://webgpu.github.io/webgpu-samples/samples/A-buffer - BSD-3-Clause license 

struct Heads {
  fragment_count: atomic<u32>,
  data: array<atomic<u32>>
};

@group(2) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(2) @binding(1) var<storage, read_write> heads: Heads;
@group(2) @binding(2) var<storage, read_write> linked_list: LinkedList;
@group(2) @binding(3) var opaque_depth_texture: texture_depth_2d;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
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
    @location(13) inv_w: f32,
) {
    let frag_coords = vec2<i32>(position_clip.xy);
    let shading_instructions = tex_idx_shading_instr[1];
    let opaque_depth = textureLoad(opaque_depth_texture, frag_coords, 0);

    // This setting is ignored for Translucent polygons in Auto-sort mode;
    // the comparison must be made on a "Greater or Equal" basis.
    // TODO: Handle pre-sorted mode.
    let depth_compare = 6u; // (shading_instructions >> 16) & 0x7;

    // NOTE: Comparisons are inversed compared to Holly's 1/z depth.
    //       Also, the label denotes when the fragment is kept, not when it's discarded.
    switch(depth_compare) {
        case 0u: { discard; } // Never
        case 1u: { // Less
            if position_clip.z <= opaque_depth { discard; }
        }
        case 2u: { // Equal
            if position_clip.z != opaque_depth { discard; }
        }
        case 3u: { // Less or Equal
            if position_clip.z < opaque_depth { discard; }
        }
        case 4u: { // Greater
            if position_clip.z >= opaque_depth { discard; }
        }
        case 5u: { // Not Equal
            if position_clip.z == opaque_depth { discard; }
        }
        case 6u: { // Greater or Equal
            if position_clip.z > opaque_depth { discard; }
        }
        case 7u: {} // Always
        default: {}
    }

    let gouraud = ((shading_instructions >> 23) & 1) == 1;
    var final_color = fragment_color(
        select(unpack4x8unorm(flat_base_color).zyxw, base_color / inv_w, gouraud),
        select(unpack4x8unorm(flat_offset_color).zyxw, offset_color / inv_w, gouraud),
        uv / inv_w,
        tex_idx_shading_instr,
        select(unpack4x8unorm(area1_flat_base_color).zyxw, area1_base_color / inv_w, gouraud),
        select(unpack4x8unorm(area1_flat_offset_color).zyxw, area1_offset_color / inv_w, gouraud),
        area1_uv / inv_w,
        area1_tex_idx_shading_instr,
        inv_w,
        false
    );

    if final_color.area0.a == 0 { discard; }

    // Add the fragment to the linked list

    // The index in the heads buffer corresponding to the head data for the fragment at the current location.
    let heads_index = (u32(frag_coords.y) - oit_uniforms.start_y) * oit_uniforms.target_width + u32(frag_coords.x);
    
    // The index in the linkedList buffer at which to store the new fragment
    let frag_index = atomicAdd(&heads.fragment_count, 1u);

    // If we run out of space to store the fragments, we just lose them
    if frag_index < oit_uniforms.max_fragments {
        let last_head = atomicExchange(&heads.data[heads_index], frag_index);
        linked_list.data[frag_index].depth = position_clip.z;
        linked_list.data[frag_index].color = pack4x8unorm(final_color.area0); // TODO: Handle Modifier volumes/Area 1
        linked_list.data[frag_index].index_and_blend_mode = ((shading_instructions >> 10) & 0x3F) | (index << 6);
        linked_list.data[frag_index].next = last_head;
    }
}