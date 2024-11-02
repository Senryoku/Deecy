struct OITUniforms {
    max_fragments: u32,
    target_width: u32,
    start_y: u32,
};

struct LinkedListElement {
  next: u32,
  depth: f32,
  index_and_blend_modes: u32, // I hope 24 bits for the index is enough!
  color_area0: u32,
  color_area1: u32,
};

struct LinkedList {
  data: array<LinkedListElement>
};

const MaxVolumes = 4;
const MaxVolumesInterfaces = 2 * MaxVolumes;

struct VolumeLinkedListElement {
  next: u32,
  depth: f32,
  volume_index: u32,
};

struct VolumeLinkedList {
  data: array<VolumeLinkedListElement>
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