struct Heads {
  fragment_count: atomic<u32>,
  data: array<atomic<u32>>
};

struct OITFSUniforms {
    max_fragments: u32,
    target_width: u32,
    start_y: u32,
    volume_index: u32,
};

@group(1) @binding(0) var<uniform> oit_uniforms: OITFSUniforms;
@group(1) @binding(1) var<storage, read_write> heads: Heads;
@group(1) @binding(2) var<storage, read_write> linked_list: VolumeLinkedList;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
) {
    let frag_coords = vec2<i32>(position_clip.xy);
	
	// See oit_draw_fs.wgsl
    let heads_index = (u32(frag_coords.y) - oit_uniforms.start_y) * oit_uniforms.target_width + u32(frag_coords.x);
    let frag_index = atomicAdd(&heads.fragment_count, 1u);
    if frag_index < oit_uniforms.max_fragments {
        let last_head = atomicExchange(&heads.data[heads_index], frag_index);
        linked_list.data[frag_index].depth = position_clip.z;
        linked_list.data[frag_index].volume_index = oit_uniforms.volume_index;
        linked_list.data[frag_index].next = last_head;
    }
}