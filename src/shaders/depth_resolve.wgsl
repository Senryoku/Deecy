@group(0) @binding(0) var multisampled_depth: texture_depth_multisampled_2d;

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> @builtin(position) vec4<f32> {
    const Vertices = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    return vec4<f32>(Vertices[vertex_index], 0.0, 1.0);
}

@fragment
fn fs_main(@builtin(position) position: vec4<f32>) -> @builtin(frag_depth) f32 {
    let coords = vec2<i32>(position.xy);
    let sample_count = textureNumSamples(multisampled_depth);
    
    var resolved_depth = textureLoad(multisampled_depth, coords, 0);
    
    for (var i = 1u; i < sample_count; i++) {
        let sample_depth = textureLoad(multisampled_depth, coords, i);
        resolved_depth = min(resolved_depth, sample_depth); 
    }

    return resolved_depth;
}
