//! GJK distance/collision detection algorithm. For now only have implementation of collision
//! detection, not distance computation.

pub use self::simplex::SimplexProcessor;

use std::cmp::Ordering;
use std::ops::{Neg, Range};

use glam::{Vec3, Mat4};

use self::simplex::{Simplex, SimplexProcessor3};
use crate::contact::{CollisionStrategy, Contact};
use crate::algorithm::minkowski::{EPA3, SupportPoint, EPA};
use crate::traits::*;

mod simplex;

const MAX_ITERATIONS: u32 = 100;
const GJK_DISTANCE_TOLERANCE: f32 = 0.000_001;
const GJK_CONTINUOUS_TOLERANCE: f32 = 0.000_001;

/// Gilbert-Johnson-Keerthi narrow phase collision detection algorithm.
#[derive(Debug)]
pub struct GJK{
    simplex_processor: SimplexProcessor3,
    epa: EPA3,
    distance_tolerance: f32,
    continuous_tolerance: f32,
    max_iterations: u32,
}

impl GJK {
    /// Create a new GJK algorithm implementation
    pub fn new() -> Self {
        Self {
            simplex_processor: SimplexProcessor3::new(),
            epa: EPA3::new(),
            distance_tolerance: GJK_DISTANCE_TOLERANCE,
            continuous_tolerance: GJK_CONTINUOUS_TOLERANCE,
            max_iterations: MAX_ITERATIONS,
        }
    }

    /// Create a new GJK algorithm implementation with the given tolerance settings
    pub fn new_with_settings(
        distance_tolerance: f32,
        continuous_tolerance: f32,
        epa_tolerance: f32,
        max_iterations: u32,
    ) -> Self {
        Self {
            simplex_processor: SimplexProcessor3::new(),
            epa: EPA3::new_with_tolerance(epa_tolerance, max_iterations),
            distance_tolerance,
            continuous_tolerance,
            max_iterations,
        }
    }

    /// Do intersection test on the given primitives
    ///
    /// ## Parameters:
    ///
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will return a simplex if a collision was detected. For 3D, it will be a tetrahedron. The simplex will enclose the origin.
    /// If no collision was detected, None is returned.
    ///
    pub fn intersect<PL, PR>(
        &self,
        left: &PL,
        left_transform: &Mat4,
        right: &PR,
        right_transform: &Mat4,
    ) -> Option<Simplex>
    where
        PL: Primitive,
        PR: Primitive,
    {
        let left_pos = left_transform.transform_point3(Vec3::zero());
        let right_pos = right_transform.transform_point3(Vec3::zero());
        let mut d = right_pos - left_pos;
        if d.cmpeq(Vec3::zero()).all() {
            d = Vec3::splat(1.);
        }
        let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
        if a.v.dot(d) <= 0. {
            return None;
        }
        let mut simplex = Simplex::new();
        simplex.push(a);
        d = d.neg();
        for _ in 0..self.max_iterations {
            let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
            if a.v.dot(d) <= 0. {
                return None;
            } else {
                simplex.push(a);
                if self.simplex_processor
                    .reduce_to_closest_feature(&mut simplex, &mut d)
                {
                    return Some(simplex);
                }
            }
        }

        None
    }

    /// Do time of impact intersection testing on the given primitives, and return a valid contact
    /// at the time of impact.
    ///
    /// ## Parameters:
    ///
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will optionally return a contact manifold at the time of impact. If no collision was
    /// detected, None is returned.
    #[allow(unused_variables)]
    pub fn intersection_time_of_impact<PL, PR>(
        &self,
        left: &PL,
        left_transform: Range<&Mat4>,
        right: &PR,
        right_transform: Range<&Mat4>,
    ) -> Option<Contact>
    where
        PL: Primitive,
        PR: Primitive,
    {
        // build the ray, A.velocity - B.velocity is the ray direction
        let left_lin_vel = left_transform.end.transform_point3(Vec3::zero())
            - left_transform.start.transform_point3(Vec3::zero());
        let right_lin_vel = right_transform.end.transform_point3(Vec3::zero())
            - right_transform.start.transform_point3(Vec3::zero());
        let ray = right_lin_vel - left_lin_vel;

        // initialize time of impact
        let mut lambda = 0.;
        let mut normal = Vec3::zero();
        let mut ray_origin = Vec3::zero();

        // build simplex and get an initial support point to bootstrap the algorithm
        let mut simplex = Simplex::new();
        let p = SupportPoint::from_minkowski(
            left,
            left_transform.start,
            right,
            right_transform.start,
            &-ray,
        );
        // we only need the actual support point for this
        let mut v = p.v;

        // if the squared magnitude is small enough, we have a hit and can stop
        while v.dot(v) > self.continuous_tolerance {
            // get a new support point
            let p = SupportPoint::from_minkowski(
                left,
                left_transform.start,
                right,
                right_transform.start,
                &-v,
            );

            let vp = v.dot(p.v);
            let vr = v.dot(ray);
            // check if we have a hit point along the ray further than the current clipped ray
            if vp > lambda * vr {
                // if the hit point is in the positive ray direction, we clip the ray, clear
                // the simplex and start over from a new ray origin
                if vr > 0. {
                    lambda = vp / vr;
                    // if the clipped hit point is beyond the end of the ray,
                    // we can never have a hit
                    if lambda > 1. {
                        return None;
                    }
                    ray_origin = ray * lambda;
                    simplex.clear();
                    normal = -v;
                } else {
                    // if the hitpoint is behind the ray origin, we can never have a hit
                    return None;
                }
            }
            // we construct the simplex around the current ray origin (if we can)
            simplex.push(p - ray_origin);
            v = self.simplex_processor
                .get_closest_point_to_origin(&mut simplex);
        }
        if v.dot(v) <= self.continuous_tolerance {
            let transform = right_transform
                .start
                .translation_interpolate(right_transform.end, lambda);
            let mut contact = Contact::new_with_point(
                CollisionStrategy::FullResolution,
                -normal.normalize(), // our convention is normal points from B towards A
                (v.dot(v)).sqrt(),       // will always be very close to zero
                transform.transform_point3(ray_origin),
            );
            contact.time_of_impact = lambda;
            Some(contact)
        } else {
            None
        }
    }

    /// Compute the distance between the given primitives.
    ///
    /// ## Parameters:
    ///
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will optionally return the distance between the objects. Will return None, if the objects
    /// are colliding.
    pub fn distance<PL, PR>(
        &self,
        left: &PL,
        left_transform: &Mat4,
        right: &PR,
        right_transform: &Mat4,
    ) -> Option<f32>
    where
        PL: Primitive,
        PR: Primitive,
    {
        let zero = Vec3::zero();
        let right_pos = right_transform.transform_point3(Vec3::zero());
        let left_pos = left_transform.transform_point3(Vec3::zero());
        let mut simplex = Simplex::new();
        let mut d = right_pos - left_pos;
        if d.cmpeq(Vec3::zero()).all() {
            d = Vec3::splat(1.);
        }
        for d in &[d, d.neg()] {
            simplex.push(SupportPoint::from_minkowski(
                left,
                left_transform,
                right,
                right_transform,
                d,
            ));
        }
        for _ in 0..self.max_iterations {
            let d = self.simplex_processor
                .get_closest_point_to_origin(&mut simplex);
            if d.cmpeq(zero).all() {
                return None;
            }
            let d = d.neg();
            let p = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
            let dp = p.v.dot(d);
            let d0 = simplex[0].v.dot(d);
            if dp - d0 < self.distance_tolerance {
                return Some((d.dot(d)).sqrt());
            }
            simplex.push(p);
        }
        None
    }

    /// Given a GJK simplex that encloses the origin, compute the contact manifold.
    ///
    /// Uses the EPA algorithm to find the contact information from the simplex.
    pub fn get_contact_manifold<PL, PR>(
        &self,
        mut simplex: &mut Vec<SupportPoint>,
        left: &PL,
        left_transform: &Mat4,
        right: &PR,
        right_transform: &Mat4,
    ) -> Option<Contact>
    where
        PL: Primitive,
        PR: Primitive,
    {
        self.epa
            .process(&mut simplex, left, left_transform, right, right_transform)
    }

    /// Do intersection testing on the given primitives, and return the contact manifold.
    ///
    /// ## Parameters:
    ///
    /// - `strategy`: strategy to use, if `CollisionOnly` it will only return a boolean result,
    ///               otherwise, EPA will be used to compute the exact contact point.
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will optionally return a `Contact` if a collision was detected. In `CollisionOnly` mode,
    /// this contact will only be a boolean result. For `FullResolution` mode, the contact will
    /// contain a full manifold (collision normal, penetration depth and contact point).
    pub fn intersection<PL, PR>(
        &self,
        strategy: &CollisionStrategy,
        left: &PL,
        left_transform: &Mat4,
        right: &PR,
        right_transform: &Mat4,
    ) -> Option<Contact>
    where
        PL: Primitive,
        PR: Primitive,
    {
        use CollisionStrategy::*;
        self.intersect(left, left_transform, right, right_transform)
            .and_then(|simplex| match *strategy {
                CollisionOnly => Some(Contact::new(CollisionOnly)),
                FullResolution => self.get_contact_manifold(
                    &mut simplex.into_vec(),
                    left,
                    left_transform,
                    right,
                    right_transform,
                ),
            })
    }

    /// Do intersection test on the given complex shapes, and return the actual intersection point
    ///
    /// ## Parameters:
    ///
    /// - `strategy`: strategy to use, if `CollisionOnly` it will only return a boolean result,
    ///               otherwise, EPA will be used to compute the exact contact point.
    /// - `left`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `left_transform`: model-to-world-transform for the left shape
    /// - `right`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `right_transform`: model-to-world-transform for the right shape
    ///
    /// ## Returns:
    ///
    /// Will optionally return a `Contact` if a collision was detected. In `CollisionOnly` mode,
    /// this contact will only be a boolean result. For `FullResolution` mode, the contact will
    /// contain a full manifold (collision normal, penetration depth and contact point), for the
    /// contact with the highest penetration depth.
    pub fn intersection_complex<PL, PR>(
        &self,
        strategy: &CollisionStrategy,
        left: &[(PL, Mat4)],
        left_transform: &Mat4,
        right: &[(PR, Mat4)],
        right_transform: &Mat4,
    ) -> Option<Contact>
    where
        PL: Primitive,
        PR: Primitive,
    {
        use CollisionStrategy::*;
        let mut contacts = Vec::default();
        for &(ref left_primitive, ref left_local_transform) in left.iter() {
            let left_transform = left_transform.mul_mat4(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in right.iter() {
                let right_transform = right_transform.mul_mat4(right_local_transform);
                if let Some(contact) = self.intersection(
                    strategy,
                    left_primitive,
                    &left_transform,
                    right_primitive,
                    &right_transform,
                ) {
                    match *strategy {
                        CollisionOnly => {
                            return Some(contact);
                        }
                        FullResolution => contacts.push(contact),
                    }
                }
            }
        }

        // CollisionOnly handling will have returned already if there was a contact, so this
        // scenario will only happen when we have a contact in FullResolution mode, or no contact
        // at all.
        contacts.into_iter().max_by(|l, r| {
            // Penetration depth defaults to 0., and can't be nan from EPA,
            // so unwrapping is safe
            l.penetration_depth
                .partial_cmp(&r.penetration_depth)
                .unwrap()
        })
    }

    /// Compute the distance between the given shapes.
    ///
    /// ## Parameters:
    ///
    /// - `left`: left shape
    /// - `left_transform`: model-to-world-transform for the left shape
    /// - `right`: right shape,
    /// - `right_transform`: model-to-world-transform for the right shape
    ///
    /// ## Returns:
    ///
    /// Will optionally return the smallest distance between the objects. Will return None, if the
    /// objects are colliding.
    pub fn distance_complex<PL, PR>(
        &self,
        left: &[(PL, Mat4)],
        left_transform: &Mat4,
        right: &[(PR, Mat4)],
        right_transform: &Mat4,
    ) -> Option<f32>
    where
        PL: Primitive,
        PR: Primitive,
    {
        let mut min_distance = None;
        for &(ref left_primitive, ref left_local_transform) in left.iter() {
            let left_transform = left_transform.mul_mat4(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in right.iter() {
                let right_transform = right_transform.mul_mat4(right_local_transform);
                match self.distance(
                    left_primitive,
                    &left_transform,
                    right_primitive,
                    &right_transform,
                ) {
                    None => return None, // colliding,
                    Some(distance) => {
                        min_distance = Some(
                            min_distance
                                .map_or(distance, |min_distance| distance.min(min_distance)),
                        )
                    }
                }
            }
        }

        min_distance
    }

    /// Do intersection time of impact test on the given complex shapes, and return the contact at
    /// the time of impact
    ///
    /// ## Parameters:
    ///
    /// - `strategy`: strategy to use, if `CollisionOnly` it will only return a boolean result,
    ///               otherwise, a full contact manifold will be returned.
    /// - `left`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `left_transform`: model-to-world-transform for the left shape
    /// - `right`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `right_transform`: model-to-world-transform for the right shape
    ///
    /// ## Returns:
    ///
    /// Will optionally return the contact if a collision was detected.
    /// In `CollisionOnly` mode, this contact will only be a time of impact. For `FullResolution`
    /// mode, the time of impact will be the earliest found among all shape primitives.
    /// Will return None if no collision was found.
    pub fn intersection_complex_time_of_impact<PL, PR>(
        &self,
        strategy: &CollisionStrategy,
        left: &[(PL, Mat4)],
        left_transform: Range<&Mat4>,
        right: &[(PR, Mat4)],
        right_transform: Range<&Mat4>,
    ) -> Option<Contact>
    where
        PL: Primitive,
        PR: Primitive,
    {
        use CollisionStrategy::*;
        let mut contacts = Vec::default();
        for &(ref left_primitive, ref left_local_transform) in left.iter() {
            let left_start_transform = left_transform.start.mul_mat4(left_local_transform);
            let left_end_transform = left_transform.end.mul_mat4(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in right.iter() {
                let right_start_transform = right_transform.start.mul_mat4(right_local_transform);
                let right_end_transform = right_transform.end.mul_mat4(right_local_transform);
                if let Some(mut contact) = self.intersection_time_of_impact(
                    left_primitive,
                    &left_start_transform..&left_end_transform,
                    right_primitive,
                    &right_start_transform..&right_end_transform,
                ) {
                    match *strategy {
                        CollisionOnly => {
                            contact.strategy = CollisionOnly;
                            return Some(contact);
                        }
                        FullResolution => contacts.push(contact),
                    }
                }
            }
        }

        // CollisionOnly handling will have returned already if there was a contact, so this
        // scenario will only happen when we have a contact in FullResolution mode or no contact
        // at all
        contacts.into_iter().min_by(|l, r| {
            l.time_of_impact
                .partial_cmp(&r.time_of_impact)
                .unwrap_or(Ordering::Equal)
        })
    }
}

#[cfg(test)]
mod tests {
    use glam::{Vec3, Quat};
    use crate::primitive::{Cuboid, Sphere};
    
    use super::*;

    fn transform_3d(
        x: f32,
        y: f32,
        z: f32,
        angle_z: f32,
    ) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rot = Quat::from_rotation_z(angle_z.to_radians());
        let tran = Vec3::new(x, y, z);
        Mat4::from_scale_rotation_translation(scale, rot, tran)
    }

    #[test]
    fn test_gjk_exact_3d() {
        let shape = Cuboid::new(1., 1., 1.);
        let t = transform_3d(0., 0., 0., 0.);
        let gjk = GJK::new();
        let p = gjk.intersection(&CollisionStrategy::FullResolution, &shape, &t, &shape, &t);
        assert!(p.is_some());
        let d = gjk.distance(&shape, &t, &shape, &t);
        assert!(d.is_none());
    }

    #[test]
    fn test_gjk_sphere() {
        let shape = Sphere::new(1.);
        let t = transform_3d(0., 0., 0., 0.);
        let gjk = GJK::new();
        let p = gjk.intersection(&CollisionStrategy::FullResolution, &shape, &t, &shape, &t);
        assert!(p.is_some());
        let d = gjk.distance(&shape, &t, &shape, &t);
        assert!(d.is_none());
    }

    #[test]
    fn test_gjk_3d_hit() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let gjk = GJK::new();
        let simplex = gjk.intersect(&left, &left_transform, &right, &right_transform);
        assert!(simplex.is_some());
        let contact = gjk.intersection(
            &CollisionStrategy::FullResolution,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(Vec3::new(-1., 0., 0.), contact.normal);
        assert_eq!(2., contact.penetration_depth);
        assert!(Vec3::new(10., 1., 5.).cmpeq(contact.contact_point).all());
    }

    #[test]
    fn test_gjk_distance_3d() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let gjk = GJK::new();
        assert_eq!(
            None,
            gjk.distance(&left, &left_transform, &right, &right_transform)
        );

        let right_transform = transform_3d(1., 0., 0., 0.);
        assert_eq!(
            Some(4.),
            gjk.distance(&left, &left_transform, &right, &right_transform)
        );
    }

    #[test]
    fn test_gjk_time_of_impact_3d() {
        let left = Cuboid::new(10., 11., 10.);
        let left_start_transform = transform_3d(0., 0., 0., 0.);
        let left_end_transform = transform_3d(30., 0., 0., 0.);
        let right = Cuboid::new(10., 15., 10.);
        let right_transform = transform_3d(15., 0., 0., 0.);
        let gjk = GJK::new();

        let contact = gjk.intersection_time_of_impact(
            &left,
            &left_start_transform..&left_end_transform,
            &right,
            &right_transform..&right_transform,
        ).unwrap();

        assert_eq!(0.166_666_7, contact.time_of_impact);
        assert_eq!(Vec3::new(-1., 0., 0.), contact.normal);
        assert_eq!(0., contact.penetration_depth);
        assert_eq!(Vec3::new(10., 0., 0.), contact.contact_point);

        assert!(gjk.intersection_time_of_impact(
            &left,
            &left_start_transform..&left_start_transform,
            &right,
            &right_transform..&right_transform
        ).is_none());
    }
}