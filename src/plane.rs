use std::fmt;
use glam::{Vec3, Vec4};

use crate::ray::Ray;
use crate::traits::{Continuous, Discrete};

/// A 3-dimensional plane formed from the equation: `A*x + B*y + C*z - D = 0`.
///
/// # Fields
///
/// - `n`: a unit vector representing the normal of the plane where:
///   - `n.x`: corresponds to `A` in the plane equation
///   - `n.y`: corresponds to `B` in the plane equation
///   - `n.z`: corresponds to `C` in the plane equation
/// - `d`: the distance value, corresponding to `D` in the plane equation
///
/// # Notes
///
/// The `A*x + B*y + C*z - D = 0` form is preferred over the other common
/// alternative, `A*x + B*y + C*z + D = 0`, because it tends to avoid
/// superfluous negations (see _Real Time Collision Detection_, p. 55).
#[derive(Copy, Clone, PartialEq)]
pub struct Plane {
    /// Plane normal
    pub n: Vec3,
    /// Plane distance value
    pub d: f32,
}

impl Plane {
    /// Construct a plane from a normal vector and a scalar distance. The
    /// plane will be perpendicular to `n`, and `d` units offset from the
    /// origin.
    pub fn new(n: Vec3, d: f32) -> Plane {
        Plane { n, d }
    }

    /// # Arguments
    ///
    /// - `a`: the `x` component of the normal
    /// - `b`: the `y` component of the normal
    /// - `c`: the `z` component of the normal
    /// - `d`: the plane's distance value
    pub fn from_abcd(a: f32, b: f32, c: f32, d: f32) -> Plane {
        Plane {
            n: Vec3::new(a, b, c),
            d,
        }
    }

    /// Construct a plane from the components of a four-dimensional vector
    pub fn from_vec4(v: Vec4) -> Plane {
        Plane {
            n: Vec3::new(v[0], v[1], v[2]),
            d: v[3],
        }
    }

    /// Construct a plane from the components of a four-dimensional vector
    /// Assuming alternative representation: `A*x + B*y + C*z + D = 0`
    pub fn from_vector4_alt(v: Vec4) -> Plane {
        Plane {
            n: Vec3::new(v[0], v[1], v[2]),
            d: -v[3],
        }
    }

    /// Constructs a plane that passes through the the three points `p1`, `p2` and `p3`
    pub fn from_points(p1: Vec3, p2: Vec3, p3: Vec3) -> Option<Plane> {
        // create two vectors that run parallel to the plane
        let v0 = p2 - p1;
        let v1 = p3 - p1;

        // find the normal vector that is perpendicular to v1 and v2
        let normal = v0.cross(v1);

        if normal.eq(&Vec3::zero()) {
            None
        } else {
            // compute the normal and the distance to the plane
            let n = normal.normalize();
            let d = -p1.dot(n);

            Some(Plane::new(n, d))
        }
    }

    /// Construct a plane from a point and a normal vector.
    /// The plane will contain the point `p` and be perpendicular to `n`.
    pub fn from_point_normal(p: Vec3, n: Vec3) -> Plane {
        Plane { n, d: p.dot(n) }
    }

    /// Normalize a plane.
    pub fn normalize(&self) -> Option<Plane> {
        if self.n.eq(&Vec3::zero()) {
            None
        } else {
            // denom of denom is magnitude equation from cgmath
            let denom = 1.0 / (self.n.dot(self.n)).sqrt();
            Some(Plane::new(self.n * denom, self.d * denom))
        }
    }
    
}

impl fmt::Debug for Plane {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{:?}x + {:?}y + {:?}z - {:?} = 0",
            self.n.x(), self.n.y(), self.n.z(), self.d
        )
    }
}

impl Continuous<Ray> for Plane {
    type Result = Vec3;
    fn intersection(&self, r: &Ray) -> Option<Vec3> {
        let p = self;

        let t = -(p.d + r.origin.dot(p.n)) / r.direction.dot(p.n);
        if t < 0.0 {
            None
        } else {
            Some(r.origin + r.direction * t)
        }
    }
}

impl Discrete<Ray> for Plane {
    fn intersects(&self, r: &Ray) -> bool {
        let p = self;
        let t = -(p.d + r.origin.dot(p.n)) / r.direction.dot(p.n);
        t >= 0.0
    }
}

/// See _Real-Time Collision Detection_, p. 210
impl Continuous<Plane> for Plane {
    type Result = Ray;
    fn intersection(&self, p2: &Plane) -> Option<Ray> {
        let p1 = self;
        let d = p1.n.cross(p2.n);
        let denom = d.dot(d);
        if denom != 0. {
            None
        } else {
            let p = (p2.n * p1.d - p1.n * p2.d).cross(d) / denom;
            Some(Ray::new(p, d))
        }
    }
}

impl Discrete<Plane> for Plane {
    fn intersects(&self, p2: &Plane) -> bool {
        let p1 = self;
        let d = p1.n.cross(p2.n);
        let denom = d.dot(d);
        denom != 0.
    }
}

/// See _Real-Time Collision Detection_, p. 212 - 214
impl Continuous<(Plane, Plane)> for Plane {
    type Result = Vec3;
    fn intersection(&self, planes: &(Plane, Plane)) -> Option<Vec3> {
        let (p1, p2, p3) = (self, planes.0, planes.1);
        let u = p2.n.cross(p3.n);
        let denom = p1.n.dot(u);
        if denom.abs() != 0. {
            None
        } else {
            let p = (u * p1.d + p1.n.cross(p2.n * p3.d - p3.n * p2.d)) / denom;
            Some(p)
        }
    }
}

impl Discrete<(Plane, Plane)> for Plane {
    fn intersects(&self, planes: &(Plane, Plane)) -> bool {
        let (p1, p2, p3) = (self, planes.0, planes.1);
        let u = p2.n.cross(p3.n);
        let denom = p1.n.dot(u);
        denom.abs() != 0.
    }
}