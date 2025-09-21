// Translucent Modifier Volumes structures

// Limits per pixel
const MaxVolumeFragments = 16;
const MaxVolumes = 4;
const MaxVolumesInterfaces = 2 * MaxVolumes;

struct OITTMVUniforms {
    pixels_per_slice: u32,
    target_width: u32,
    start_y: u32,
};

struct VolumeFragmentList {
	// data[0]: volume index (u32)
	// data[1]: depth (f32)
	data: array<vec2<u32>>
};

// NOTE: I tried packing the depths into u8 and u16, but the depths values are too clamped together and this results
//       in very visible precision loss. u16 was ok in certain scenes, but there are some where all the opaque geometry
//       lies in a very small depth range while the UI is very far away. In this case 16bit precision isn't enough.
struct Volumes {
	count: u32,
	_padding: u32,
	intervals: array<vec2<f32>, MaxVolumes>,
};

// A way to re-interpret the "Volumes" struct
struct VolumesInterfaces {
	count: u32,
	_padding: u32,
	interfaces: array<f32, MaxVolumesInterfaces>,
};
