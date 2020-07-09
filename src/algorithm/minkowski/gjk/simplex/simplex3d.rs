use std::ops::Neg;

use glam::Vec3;

use super::{Simplex, SimplexProcessor};
use crate::primitive::util::{barycentric_vector, get_closest_point_on_edge};

/// Simplex processor implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct SimplexProcessor3 {
}

#[allow(clippy::many_single_char_names)]
impl SimplexProcessor for SimplexProcessor3 {

    fn reduce_to_closest_feature(
        &self,
        simplex: &mut Simplex,
        v: &mut Vec3,
    ) -> bool {
        // 4 points, full tetrahedron, origin could be inside
        if simplex.len() == 4 {
            let a = simplex[3].v;
            let b = simplex[2].v;
            let c = simplex[1].v;
            let d = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            let abc = ab.cross(ac);

            // origin outside plane ABC, remove D and check side
            // need to check both edges
            if abc.dot(ao) > 0. {
                simplex.remove(0);
                check_side(&abc, &ab, &ac, &ao, simplex, v, true, false);
            } else {
                let ad = d - a;
                let acd = ac.cross(ad);
                // origin outside plane ACD, remove B and check side
                // no need to test first edge, since that region is also over ABC
                if acd.dot(ao) > 0. {
                    simplex.remove(2);
                    check_side(&acd, &ac, &ad, &ao, simplex, v, true, true);
                } else {
                    let adb = ad.cross(ab);
                    // origin outside plane ADB, remove C and check side
                    // no need to test edges, since those regions are covered in earlier tests
                    if adb.dot(ao) > 0. {
                        // [b, d, a]
                        simplex.remove(1);
                        simplex.swap(0, 1);
                        *v = adb;
                    // origin is inside simplex
                    } else {
                        return true;
                    }
                }
            }
        }
        // 3 points, can't do origin check, find closest feature to origin, and move that way
        else if simplex.len() == 3 {
            let a = simplex[2].v;
            let b = simplex[1].v;
            let c = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            check_side(&ab.cross(ac), &ab, &ac, &ao, simplex, v, false, false);
        }
        // 2 points, can't do much with only an edge, only find a new search direction
        else if simplex.len() == 2 {
            let a = simplex[1].v;
            let b = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;

            *v = cross_aba(&ab, &ao);
            if v.cmpeq(Vec3::zero()).all() {
                v.set_x(0.1);
            }
        }
        // 0-1 points
        false
    }

    /// Get the closest point on the simplex to the origin.
    ///
    /// Make simplex only retain the closest feature to the origin.
    fn get_closest_point_to_origin(&self, simplex: &mut Simplex) -> Vec3 {
        let mut d = Vec3::zero();

        // reduce simplex to the closest feature to the origin
        // if check_origin return true, the origin is inside the simplex, so return the zero vector
        // if not, the simplex will be the closest face or edge to the origin, and d the normal of
        // the feature in the direction of the origin
        if self.reduce_to_closest_feature(simplex, &mut d) {
            return d;
        }

        if simplex.len() == 1 {
            simplex[0].v
        } else if simplex.len() == 2 {
            get_closest_point_on_edge(&simplex[1].v, &simplex[0].v, &Vec3::zero())
        } else {
            get_closest_point_on_face(&simplex[2].v, &simplex[1].v, &simplex[0].v, &d)
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[allow(clippy::many_single_char_names)]
#[inline]
fn get_closest_point_on_face(
    a: &Vec3,
    b: &Vec3,
    c: &Vec3,
    normal: &Vec3,
) -> Vec3 {
    use crate::{traits::Continuous, plane::Plane, ray::Ray};
    let ap = *a;
    let bp = *b;
    let cp = *c;
    let ray = Ray::new(Vec3::zero(), -*normal);
    // unwrapping is safe, because the degenerate face will have been removed by the outer algorithm
    let plane = Plane::from_points(ap, bp, cp).unwrap();
    match plane.intersection(&ray) {
        Some(point) => {
            let (u, v, w) = barycentric_vector(point, *a, *b, *c);
            assert!(
                in_range(u) && in_range(v) && in_range(w),
                "Simplex processing failed to deduce that this simplex {:?} is an edge case",
                [a, b, c]
            );

            point
        }

        _ => Vec3::zero(),
    }
}

#[inline]
fn in_range(v: f32) -> bool {
    v >= 0. && v <= 1.
}

#[inline]
fn cross_aba(a: &Vec3, b: &Vec3) -> Vec3 {
    a.cross(*b).cross(*a)
}

#[allow(clippy::too_many_arguments)]
#[inline]
fn check_side(
    abc: &Vec3,
    ab: &Vec3,
    ac: &Vec3,
    ao: &Vec3,
    simplex: &mut Simplex,
    v: &mut Vec3,
    above: bool,
    ignore_ab: bool,
) {
    let ab_perp = ab.cross(*abc);

    // origin outside AB, remove C and v = edge normal towards origin
    if !ignore_ab && ab_perp.dot(*ao) > 0. {
        simplex.remove(0);
        *v = cross_aba(ab, ao);
        return;
    }

    let ac_perp = abc.cross(*ac);

    // origin outside AC, remove B and v = edge normal towards origin
    if ac_perp.dot(*ao) > 0. {
        simplex.remove(1);
        *v = cross_aba(ac, ao);
        return;
        // origin above triangle, set v = surface normal towards origin
    }

    if above || abc.dot(*ao) > 0. {
        // [c, b, a]
        *v = *abc;
    // origin below triangle, rewind simplex and set v = surface normal towards origin
    } else {
        // [b, c, a]
        simplex.swap(0, 1);
        *v = abc.neg();
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Neg;

    use glam::Vec3;

    use super::*;
    use crate::algorithm::minkowski::SupportPoint;

    #[test]
    fn test_check_side_outside_ab() {
        let mut simplex = smallvec![sup(8., -10., 0.), sup(-1., -10., 0.), sup(3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vec3::new(-1., -10., 0.), simplex[0].v); // B should be last in the simplex
        assert_eq!(-375., v.x());
        assert_eq!(100., v.y());
        assert_eq!(0., v.z());
    }

    #[test]
    fn test_check_side_outside_ac() {
        let mut simplex = smallvec![sup(2., -10., 0.), sup(-7., -10., 0.), sup(-3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vec3::new(2., -10., 0.), simplex[0].v); // C should be last in the simplex
        assert_eq!(300., v.x());
        assert_eq!(100., v.y());
        assert_eq!(0., v.z());
    }

    #[test]
    fn test_check_side_above() {
        let mut simplex = smallvec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vec3::new(5., -10., -1.), simplex[0].v); // C should be last in the simplex
        assert_eq!(0., v.x());
        assert_eq!(0., v.y());
        assert_eq!(135., v.z());
    }

    #[test]
    fn test_check_side_below() {
        let mut simplex = smallvec![sup(5., -10., 1.), sup(-4., -10., 1.), sup(0., 5., 1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vec3::new(-4., -10., 1.), simplex[0].v); // B should be last in the simplex
        assert_eq!(0., v.x());
        assert_eq!(0., v.y());
        assert_eq!(-135., v.z());
    }

    #[test]
    fn test_check_origin_empty() {
        let mut simplex = smallvec![];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert!(simplex.is_empty());
        assert!(Vec3::zero().cmpeq(v).all())
    }

    #[test]
    fn test_check_origin_point() {
        let mut simplex = smallvec![sup(8., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(1, simplex.len());
        assert!(Vec3::zero().cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_line() {
        let mut simplex = smallvec![sup(8., -10., 0.), sup(-1., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(2, simplex.len());
        assert!(Vec3::zero().cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_triangle() {
        let mut simplex = smallvec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert!(Vec3::zero().cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_abc() {
        let mut simplex = smallvec![
            sup(8., -10., -1.),
            sup(-1., -10., -1.),
            sup(3., 5., -1.),
            sup(3., -3., 5.)
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert!(Vec3::new(-1., -10., -1.).cmpeq(simplex[0].v).all());
        assert!(Vec3::zero().cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_acd() {
        let mut simplex = smallvec![
            sup(8., 0.1, -1.),
            sup(-1., 0.1, -1.),
            sup(3., 15., -1.),
            sup(3., 7., 5.)
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert!(Vec3::new(3., 7., 5.).cmpeq(simplex[2].v).all());
        assert!(Vec3::new(-1., 0.1, -1.).cmpeq(simplex[1].v).all());
        assert!(Vec3::new(8., 0.1, -1.).cmpeq(simplex[0].v).all());
        assert!(Vec3::new(0., -54., 62.1).cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_adb() {
        let mut simplex = smallvec![
            sup(2., -10., -1.),
            sup(-7., -10., -1.),
            sup(-3., 5., -1.),
            sup(-3., -3., 5.)
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert!(Vec3::new(-3., -3., 5.).cmpeq(simplex[2].v).all());
        assert!(Vec3::new(2., -10., -1.).cmpeq(simplex[1].v).all());
        assert!(Vec3::new(-3., 5., -1.).cmpeq(simplex[0].v).all());
        assert!(Vec3::new(90., 30., 40.).cmpeq(v).all());
    }

    #[test]
    fn test_check_origin_tetrahedron_inside() {
        let mut simplex = smallvec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.)
        ];
        let (hit, _) = test_check_origin(&mut simplex);
        assert!(hit);
        assert_eq!(4, simplex.len());
    }

    fn test_check_origin(simplex: &mut Simplex) -> (bool, Vec3) {
        let mut v = Vec3::zero();
        let b = SimplexProcessor3::new().reduce_to_closest_feature(simplex, &mut v);
        (b, v)
    }

    fn test_check_side(
        simplex: &mut Simplex,
        above: bool,
        ignore: bool,
    ) -> Vec3 {
        let ab = simplex[1].v - simplex[2].v;
        let ac = simplex[0].v - simplex[2].v;
        let ao = simplex[2].v.neg();
        let abc = ab.cross(ac);
        let mut v = Vec3::zero();
        check_side(&abc, &ab, &ac, &ao, simplex, &mut v, above, ignore);
        v
    }

    fn sup(x: f32, y: f32, z: f32) -> SupportPoint {
        let mut s = SupportPoint::new();
        s.v = Vec3::new(x, y, z);
        s
    }
}