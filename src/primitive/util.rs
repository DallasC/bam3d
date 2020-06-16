//! Utilities
//!

use crate::{Aabb, ray::Ray};
use glam::{Vec2, Vec3, Mat4};

pub(crate) fn get_max_point<'a, I>(vertices: I, direction: &Vec3, transform: &Mat4) -> Vec3 
where 
    I: Iterator<Item = &'a Vec3>,
{
    let direction = transform.inverse().transform_vector3(*direction);
    let (p, _) = vertices.map(|&v| (v, v.dot(direction))).fold(
        (Vec3::zero(), f32::NEG_INFINITY),
        |(max_p, max_dot), (v, dot)| {
            if dot > max_dot {
                (v, dot)
            } else {
                (max_p, max_dot)
            }
        },
    );
    transform.transform_point3(p)
}

pub(crate) fn get_bound<'a, I, A: 'a>(vertices: I) -> A
where
    A: Aabb,
    I: Iterator<Item = &'a Vec3>,
{
    vertices.fold(A::zero(), |bound, p| bound.grow(*p))
}

#[allow(dead_code)]
#[inline]
pub(crate) fn triple_product(a: &Vec2, b: &Vec2, c: &Vec2) -> Vec2 {
    let ac = a.x() * c.x() + a.y() * c.y();
    let bc = b.x() * c.x() + b.y() * c.y();
    Vec2::new(b.x() * ac - a.x() * bc, b.y() * ac - a.y() * bc)
}

/// Compute barycentric coordinates of p in relation to the triangle defined by (a, b, c).
#[allow(dead_code)]
pub(crate) fn barycentric_vector(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (f32, f32, f32){
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let inv_denom = 1. / (d00 * d11 - d01 * d01);

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = 1. - v - w;
    (u, v, w)
}

/// Compute barycentric coordinates of p in relation to the triangle defined by (a, b, c).
pub(crate) fn barycentric_point(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (f32, f32, f32) {
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let inv_denom = 1. / (d00 * d11 - d01 * d01);

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = 1. - v - w;
    (u, v, w)
}

#[inline]
pub(crate) fn get_closest_point_on_edge(start: &Vec3, end: &Vec3, point: &Vec3) -> Vec3 {
    let line = *end - *start;
    let line_dir = line.normalize();
    let v = *point - *start;
    let d = v.dot(line_dir);
    if d < 0. {
        *start
    } else if (d * d) > line.dot(line) {
        *end
    } else {
        *start + line_dir * d
    }
}

pub(crate) fn cylinder_ray_quadratic_solve(r: &Ray, radius: f32) -> Option<(f32, f32)> {

    let a = r.direction.x() * r.direction.x() + r.direction.z() * r.direction.z();
    let b = 2. * (r.direction.x() * r.origin.x() + r.direction.z() * r.origin.z());
    let c = r.origin.x() * r.origin.x() + r.origin.z() * r.origin.z() - radius * radius;

    let dr = b * b - 4. * a * c;
    if dr < 0. {
        return None;
    }
    let drsqrt = dr.sqrt();
    let t1 = (-b + drsqrt) / (2. * a);
    let t2 = (-b - drsqrt) / (2. * a);
    Some((t1, t2))
}

// TODO 3d Tests