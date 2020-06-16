//! Line segments
use glam::Vec3;

/// A generic directed line segment from `origin` to `dest`.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Line {
    /// Origin of the line
    pub origin: Vec3,
    /// Endpoint of the line
    pub dest: Vec3,
}

impl Line {
    /// Create a new directed line segment from `origin` to `dest`.
    pub fn new(origin: Vec3, dest: Vec3) -> Line {
        Line {
            origin,
            dest,
        }
    }
}