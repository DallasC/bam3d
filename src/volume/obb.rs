//! Oriented bounding boxes

use glam::Vec3;

/// Generic object bounding box, centered on `center`, aligned with `axis`,
/// and with size `extents`.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Obb {
    /// OBB center point in world space
    pub center: Vec3,
    /// Axis OBB is aligned with
    pub axis: Vec3,
    /// Size of the OBB
    pub extents: Vec3,
}

impl Obb {
    /// Create a new generic OBB with the given `center`, `axis` and `extents`
    pub fn new(center: Vec3, axis: Vec3, extents: Vec3) -> Self {
        Self {
            center,
            axis,
            extents
        }
    }
}