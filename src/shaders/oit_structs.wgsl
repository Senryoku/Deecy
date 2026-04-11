struct OITUniforms {
    max_fragments: u32,
    target_width: u32,
    start_y: u32,
};


struct LinkedListElement {
  depth: f32,
  index_and_blend_modes: u32,
  color: u32,
  next: u32,
};

struct LinkedList {
  data: array<LinkedListElement>
};