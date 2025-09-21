struct Heads {
  fragment_count: array<atomic<u32>>
};

struct OITFSUniforms {
    volume_index: u32,
};

@group(1) @binding(0) var<uniform> oit_tmv_uniforms: OITTMVUniforms;
@group(1) @binding(1) var<storage, read_write> heads: Heads;
@group(1) @binding(2) var<storage, read_write> fragments: VolumeFragmentList;
@group(1) @binding(3) var<uniform> oit_fs_uniforms: OITFSUniforms;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
) {
    let frag_coords = vec2<u32>(position_clip.xy);
	
    let heads_index = (frag_coords.y - oit_tmv_uniforms.start_y) * oit_tmv_uniforms.target_width + frag_coords.x;
    let frag_index = atomicAdd(&heads.fragment_count[heads_index], 1u);
    if frag_index < MaxVolumeFragments {
        // NOTE: This might seem like a weird way of indexing the array, but this improves memory access coherence a lot,
        //       and thus performance.
        let index =  frag_index * oit_tmv_uniforms.pixels_per_slice + heads_index;
        fragments.data[index] = vec2<u32>(oit_fs_uniforms.volume_index, bitcast<u32>(position_clip.z));
    }
}