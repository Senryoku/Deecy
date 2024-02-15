// Modified from https://webgpu.github.io/webgpu-samples/samples/A-buffer - BSD-3-Clause license 

struct OITUniforms {
    max_fragments: u32,
    target_width: u32,
    start_y: u32,
};

struct Heads {
  fragment_count: atomic<u32>,
  data: array<atomic<u32>>
};

struct LinkedListElement {
  color: vec4<f32>,
  depth: f32,
  blend_mode: u32,
  next: u32
};

struct LinkedList {
  data: array<LinkedListElement>
};

@group(2) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(2) @binding(1) var<storage, read_write> heads: Heads;
@group(2) @binding(2) var<storage, read_write> linked_list: LinkedList;
@group(2) @binding(3) var opaque_depth_texture: texture_depth_2d;

@fragment
fn main(
    @builtin(position) position: vec4<f32>,
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) w: f32,
    @location(4) @interpolate(flat) tex: vec2<u32>,
) {
    let frag_coords = vec2<i32>(position.xy);
    let opaque_depth = textureLoad(opaque_depth_texture, frag_coords, 0);

    let depth_compare = (tex[1] >> 16) & 0x7;

    // NOTE: Comparisons are inversed compared to Holly's 1/z depth.
    //       Also, the label denotes when the fragment is kept, not when it's discarded.
    switch(depth_compare) {
        case 0u: { discard; } // Never
        case 1u: { // Less
            if position.z <= opaque_depth { discard; }
        }
        case 2u: { // Equal
            if position.z != opaque_depth { discard; }
        }
        case 3u: { // Less or Equal
            if position.z < opaque_depth { discard; }
        }
        case 4u: { // Greater
            if position.z >= opaque_depth { discard; }
        }
        case 5u: { // Not Equal
            if position.z == opaque_depth { discard; }
        }
        case 6u: { // Greater or Equal
            if position.z > opaque_depth { discard; }
        }
        case 7u: {} // Always
        default: {}
    }

    var final_color: vec4<f32> = fragment_color(base_color / w, offset_color / w, uv / w, tex);

    // Add the fragment to the linked list

    // The index in the heads buffer corresponding to the head data for the fragment at
    // the current location.
    let heads_index = (u32(frag_coords.y) - oit_uniforms.start_y) * oit_uniforms.target_width + u32(frag_coords.x);
    
    // The index in the linkedList buffer at which to store the new fragment
    let frag_index = atomicAdd(&heads.fragment_count, 1u);

    // If we run out of space to store the fragments, we just lose them
    if frag_index < oit_uniforms.max_fragments {
        let last_head = atomicExchange(&heads.data[heads_index], frag_index);
        linked_list.data[frag_index].depth = position.z;
        linked_list.data[frag_index].color = final_color;
        linked_list.data[frag_index].blend_mode = (tex[1] >> 10) & 0x3F;
        linked_list.data[frag_index].next = last_head;
    }
}