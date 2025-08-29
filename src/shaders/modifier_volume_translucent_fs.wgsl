struct Heads {
  fragment_count: array<atomic<u32>>
};

struct OITFSUniforms {
    volume_index: u32,
};

@group(1) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(1) @binding(1) var<storage, read_write> heads: Heads;
@group(1) @binding(2) var<storage, read_write> linked_list: VolumeLinkedList;
@group(1) @binding(3) var<uniform> oit_fs_uniforms: OITFSUniforms;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
) {
    let frag_coords = vec2<i32>(position_clip.xy);
	
    let heads_index = (u32(frag_coords.y) - oit_uniforms.start_y) * oit_uniforms.target_width + u32(frag_coords.x);
    let frag_index = atomicAdd(&heads.fragment_count[heads_index], 1u);
    if frag_index < 16 {
        linked_list.data[16 * heads_index + frag_index] = vec2<u32>(oit_fs_uniforms.volume_index, bitcast<u32>(position_clip.z));
    }
}