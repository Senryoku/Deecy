@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var dst0: texture_storage_2d<bgra8unorm, write>;
@group(0) @binding(2) var dst1: texture_storage_2d<bgra8unorm, write>;
@group(0) @binding(3) var dst2: texture_storage_2d<bgra8unorm, write>;

var<workgroup> cache : array<vec4<f32>, 16>;

@compute @workgroup_size(4, 4)
fn main(@builtin(global_invocation_id) id: vec3<u32>, @builtin(local_invocation_id) local_id: vec3<u32>, @builtin(local_invocation_index) local_idx: u32) {
    var color = (textureLoad(src, 2 * id.xy + vec2<u32>(0, 0), 0) + //
		textureLoad(src, 2 * id.xy + vec2<u32>(0, 1), 0) + // 
		textureLoad(src, 2 * id.xy + vec2<u32>(1, 0), 0) + //
		textureLoad(src, 2 * id.xy + vec2<u32>(1, 1), 0)) * 0.25;
    textureStore(dst0, id.xy, color);
    cache[local_idx] = color;

    workgroupBarrier();

    if local_id.x % 2 == 0 && local_id.y % 2 == 0 {
        color = (cache[local_idx / 2 + 0] + cache[local_idx / 2 + 1] + cache[local_idx / 2 + 4] + cache[local_idx / 2 + 5]) * 0.25;
        textureStore(dst1, id.xy / 2u, color);
    }

    workgroupBarrier();

    if local_id.x % 2 == 0 && local_id.y % 2 == 0 {
        cache[local_idx / 2] = color;
    }

    workgroupBarrier();

    if local_idx == 0 {
        color = (cache[0] + cache[1] + cache[2] + cache[3]) * 0.25;
        textureStore(dst2, id.xy / 4u, color);
    }
}