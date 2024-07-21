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
};

struct VolumeLinkedList {
  data: array<VolumeLinkedListElement>
};

struct Volumes {
	count: u32,
	intervals: array<vec2<f32>, MaxVolumes>,
};