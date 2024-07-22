// Convert vertex positions from the TA to clip positions for WebGPU

const screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

fn to_position_clip(position: vec3<f32>, depth_max: f32) -> vec4<f32> {
    // FIXME: Soul Calibur sends some very high (sometimes infinite) z values, breaking everything.
	//        Clamping seems to be enough for now, but obviously not ideal (and the value is completely arbitrary).
	//        Might want to revisit later if we get issues with vertices really close to the near plane.
    var max_depth = min(32768.0, depth_max);
    var z = min(32768.0, position.z);

    let inv_w = 1.0 / z;

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    return vec4<f32>(
        inv_w * (position.x * 2.0 / screen_size.x - 1.0),
        inv_w * (position.y * -2.0 / screen_size.y + 1.0),
        1.0 / max_depth, // Remap to the [0..1] clip range used by WebGPU
        inv_w
    );
}