// Translucent Modifier Volumes structures

// Limits per pixel
const MaxVolumeFragments = 16;
const MaxVolumes = 4;
const MaxVolumesInterfaces = 2 * MaxVolumes;

struct OITTMVUniforms {
    square_size: u32,
    pixels_per_slice: u32,
    target_width: u32,
    start_y: u32,
};

struct VolumeFragmentList {
	// data[0]: volume index (u32)
	// data[1]: depth (f32)
	data: array<vec2<u32>>
};

struct Volumes {
	count: u32,
	intervals: array<vec2<f32>, MaxVolumes>,
};

// NOTE: I tried packing the depths into u8 and u16, but the depths values are too clamped together and this results
//       in very visible precision loss. u16 was ok in certain scenes, but there are some where all the opaque geometry
//       lies in a very small depth range while the UI is very far away. In this case 16bit precision isn't enough.
//       Unused slots are set to -1.0.
struct StoredVolumes {
	intervals: array<vec2<f32>, MaxVolumes>,
};

// A way to re-interpret the "StoredVolumes" struct
struct VolumesInterfaces {
	interfaces: array<f32, MaxVolumesInterfaces>,
};


// The buffers are slighly oversized to allow a more coherent access pattern.
// Slices are vertically split into power-of-two sized squares which are accessed following a z-order curve.

fn slice_coords_to_pixel_index(info: OITTMVUniforms, slice_coords: vec2<u32>) -> u32 {
	// Here we assume that the slice is wider than tall.
    let hozizontal_square : u32 = slice_coords.x / info.square_size;
    let pixels_per_square : u32 = info.square_size * info.square_size;

    let square_local_coords = vec2<u32>(slice_coords.x % info.square_size, slice_coords.y);
    let pixel_index = pixels_per_square * hozizontal_square + morton2(square_local_coords);

	return pixel_index;
}

fn tmv_fragment_index(info: OITTMVUniforms, pixel_index: u32, frag_index: u32) -> u32 {
	return frag_index * info.pixels_per_slice + pixel_index;
}