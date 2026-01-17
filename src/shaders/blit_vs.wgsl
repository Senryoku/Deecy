struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
     @location(0) uv: vec2<f32>,
}

struct OITUniforms {
    min: vec2<f32>,
    max: vec2<f32>,
};

@group(0) @binding(2) var<uniform> uniforms: OITUniforms;

@vertex
fn main(
    @location(0) position: vec2<f32>,
    @location(1) uv: vec2<f32>,
) -> VertexOut {
    var output: VertexOut;
    output.position_clip = vec4(position.xy, 0.0, 1.0);
    output.uv = uv * (uniforms.max - uniforms.min) + uniforms.min;
    return output;
}