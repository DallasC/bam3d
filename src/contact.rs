//! Collision contact manifold

use glam::{Vec3};

/// Collision strategy to use for collisions.
///
/// This is used both to specify what collision strategy to use for each shape, and also each
/// found contact will have this returned on it, detailing what data is relevant in the
/// [`Contact`](struct.Contact.html).
#[derive(Debug, PartialEq, Clone, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CollisionStrategy {
    /// Compute full contact manifold for the collision
    FullResolution,

    /// Only report that a collision occurred, skip computing contact information for the collision.
    CollisionOnly,
}

/// Contact manifold for a single collision contact point.
#[derive(Debug, Clone)]
pub struct Contact {
    /// The collision strategy used for this contact.
    pub strategy: CollisionStrategy,

    /// The collision normal. Only applicable if the collision strategy is not `CollisionOnly`
    pub normal: Vec3,

    /// The penetration depth. Only applicable if the collision strategy is not `CollisionOnly`
    pub penetration_depth: f32,

    /// The contact point. Only applicable if the collision strategy is not `CollisionOnly`
    pub contact_point: Vec3,

    /// The time of impact, only applicable for continuous collision detection, value is in
    /// range 0.0..1.0
    pub time_of_impact: f32,
}

impl Contact {
    /// Create a new contact manifold, with default collision normal and penetration depth
    pub fn new(strategy: CollisionStrategy) -> Self {
        Self::new_impl(strategy, Vec3::zero(), 0.)
    }

    /// Create a new contact manifold, with the given collision normal and penetration depth
    pub fn new_impl(
        strategy: CollisionStrategy,
        normal: Vec3,
        penetration_depth:f32,
    ) -> Self {
        Self::new_with_point(strategy, normal, penetration_depth, Vec3::zero())
    }

    /// Create a new contact manifold, complete with contact point
    pub fn new_with_point(
        strategy: CollisionStrategy,
        normal: Vec3,
        penetration_depth: f32,
        contact_point: Vec3,
    ) -> Self {
        Self {
            strategy,
            normal,
            penetration_depth,
            contact_point,
            time_of_impact: 0.,
        }
    }
}