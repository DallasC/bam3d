use glam::{Vec3, Mat4};

/// An intersection test with a result.
///
/// An example would be a Ray vs AABB intersection test that returns a Point in space.
///
pub trait Continuous<RHS> {
    type Result;
    /// Intersection test
    fn intersection(&self, _: &RHS) -> Option<Self::Result>;
}

/// A boolean intersection test.
///
pub trait Discrete<RHS> {
    /// Intersection test
    fn intersects(&self, _: &RHS) -> bool;
}

/// Boolean containment test.
///
pub trait Contains<RHS> {
    /// Containment test
    fn contains(&self, _: &RHS) -> bool;
}

/// Shape surface area
///
pub trait SurfaceArea {
    /// Compute surface area
    fn surface_area(&self) -> f32;
}

/// Build the union of two shapes.
///
pub trait Union<RHS = Self> {
    /// Union shape created
    type Output;

    /// Build the union shape of self and the given shape.
    fn union(&self, _: &RHS) -> Self::Output;
}

/// Bounding volume abstraction for use with algorithms
pub trait Bound {

    /// Minimum extents of the bounding volume
    fn min_extent(&self) -> Vec3;
    /// Maximum extents of the bounding volume
    fn max_extent(&self) -> Vec3;
    /// Create a new bounding volume extended by the given amount
    fn with_margin(&self, add: Vec3) -> Self;
    /// Apply an arbitrary transform to the bounding volume
    fn transform_volume(&self, transform: &Mat4) -> Self;
    /// Create empty volume
    fn empty() -> Self;
}

/// Primitive with bounding volume
pub trait HasBound {
    /// Bounding volume type
    type Bound: Bound;

    /// Borrow the bounding volume
    fn bound(&self) -> &Self::Bound;
}

/// Utilities for computing bounding volumes of primitives
pub trait ComputeBound<B>
where
    B: Bound,
{
    /// Compute the bounding volume
    fn compute_bound(&self) -> B;
}

/// Minkowski support function for primitive
pub trait Primitive {
    /// Get the support point on the shape in a given direction.
    ///
    /// ## Parameters
    ///
    /// - `direction`: The search direction in world space.
    /// - `transform`: The current local to world transform for this primitive.
    ///
    /// ## Returns
    ///
    /// Return the point that is furthest away from the origin, in the given search direction.
    /// For discrete shapes, the furthest vertex is enough, there is no need to do exact
    /// intersection point computation.
    ///
    /// ## Type parameters
    ///
    /// - `P`: Transform type
    fn support_point(
        &self,
        direction: &Vec3,
        transform: &Mat4,
    ) -> Vec3;
}

/// Discrete intersection test on transformed primitive
pub trait DiscreteTransformed<RHS> {

    /// Intersection test for transformed self
    fn intersects_transformed(&self, _: &RHS, _: &Mat4) -> bool;
}

/// Continuous intersection test on transformed primitive
pub trait ContinuousTransformed<RHS> {
    /// Intersection test for transformed self
    fn intersection_transformed(&self, _: &RHS, _: &Mat4) -> Option<Vec3>;
}

/// Trait used for interpolation of values
///
/// ## Type parameters:
///
/// - `S`: The scalar type used for amount
pub trait Interpolate {
    /// Interpolate between `self` and `other`, using amount to calculate how much of other to use.
    ///
    /// ## Parameters:
    ///
    /// - `amount`: amount in the range 0. .. 1.
    /// - `other`: the other value to interpolate with
    ///
    /// ## Returns
    ///
    /// A new value approximately equal to `self * (1. - amount) + other * amount`.
    fn interpolate(&self, other: &Self, amount: f32) -> Self;
}

/// Trait used for interpolation of translation only in transforms
pub trait TranslationInterpolate {
    /// Interpolate between `self` and `other`, using amount to calculate how much of other to use.
    ///
    /// ## Parameters:
    ///
    /// - `amount`: amount in the range 0. .. 1.
    /// - `other`: the other value to interpolate with
    ///
    /// ## Returns
    ///
    /// A new value approximately equal to `self * (1. - amount) + other * amount`.
    fn translation_interpolate(&self, other: &Self, amount: f32) -> Self;
}

mod interpolate {
    use super::{Interpolate, TranslationInterpolate};
    use glam::{Mat3, Mat4, Quat};

    impl Interpolate for Quat {
        fn interpolate(&self, other: &Self, amount: f32) -> Self {
            self.lerp(*other, amount)
        }
    }

    impl Interpolate for Mat3 {
        fn interpolate(&self, other: &Self, amount: f32) -> Self {
            Mat3::from_quat(
                Quat::from_rotation_mat3(self).lerp(Quat::from_rotation_mat3(other), amount),
            )
        }
    }

    impl Interpolate for Mat4 {
        fn interpolate(&self, other: &Self, amount: f32) -> Self {
            let (self_scale, self_rotation, self_translation) = self.to_scale_rotation_translation();
            let (other_scale, other_rotation, other_translation) = other.to_scale_rotation_translation();

            let scale = self_scale * (1.0 - amount) + other_scale * amount;
            let rotation = self_rotation.interpolate(&other_rotation, amount);
            let translation = self_translation.lerp(other_translation, amount);

            Mat4::from_scale_rotation_translation(scale, rotation, translation)
        }
    }

    impl TranslationInterpolate for Mat4{
        fn translation_interpolate(&self, other: &Self, amount: f32) -> Self {
            let (_self_scale, _self_rotation, self_translation) = self.to_scale_rotation_translation();
            let (other_scale, other_rotation, other_translation) = other.to_scale_rotation_translation();

            let scale = other_scale;
            let rotation = other_rotation;
            let translation = self_translation.lerp(other_translation, amount);

            Mat4::from_scale_rotation_translation(scale, rotation, translation)
        }
    }
}