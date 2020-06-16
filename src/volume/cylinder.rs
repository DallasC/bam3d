//! Oriented bounding cylinder

use glam::Vec3;

/// Bounding cylinder
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cylinder {
    /// Center point
    pub center: Vec3,
    /// Axis the cylinder is aligned with
    pub axis: Vec3,
    /// Radius of the cylinder
    pub radius: f32,
}