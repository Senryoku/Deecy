struct OITFSUniforms {
    volume_index: u32,
};

@group(1) @binding(0) var<uniform> oit_tmv_uniforms: OITTMVUniforms;
@group(1) @binding(1) var<storage, read_write> fragment_counts: array<atomic<u32>>;
@group(1) @binding(2) var<storage, read_write> fragments: VolumeFragmentList;
@group(1) @binding(3) var<uniform> oit_fs_uniforms: OITFSUniforms;
@group(1) @binding(4) var opaque_depth_texture: texture_depth_2d;

@fragment
fn main(
    @builtin(position) position_clip: vec4<f32>,
) {
    let frag_coords = vec2<u32>(position_clip.xy);

    // NOTE/FIXME: Writing to a storage buffer disables early depth test, which is a shame for performance.
    //             It currently can't forced on in WebGPU (https://github.com/gpuweb/gpuweb/issues/4891).
    let opaque_depth = textureLoad(opaque_depth_texture, frag_coords, 0);
    if position_clip.z < opaque_depth { discard; }

    let pixel_index = slice_coords_to_pixel_index(oit_tmv_uniforms, vec2<u32>(frag_coords.x, frag_coords.y - oit_tmv_uniforms.start_y));

    let frag_index = atomicAdd(&fragment_counts[pixel_index], 1u);
    if frag_index < MaxVolumeFragments {
        let index = tmv_fragment_index(oit_tmv_uniforms, pixel_index, frag_index);
        fragments.data[index] = vec2<u32>(oit_fs_uniforms.volume_index, bitcast<u32>(position_clip.z));
    }
}