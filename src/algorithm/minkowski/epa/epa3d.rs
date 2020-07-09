use glam::{Vec3, Mat4};

use super::*;
use super::SupportPoint;
use crate::{contact::CollisionStrategy, contact::Contact, primitive::util::barycentric_vector};

/// EPA algorithm implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct EPA3 {
    tolerance: f32,
    max_iterations: u32,
}

impl EPA for EPA3 {

    fn process<SL, SR>(
        &self,
        mut simplex: &mut Vec<SupportPoint>,
        left: &SL,
        left_transform: &Mat4,
        right: &SR,
        right_transform: &Mat4,
    ) -> Option<Contact> 
    where
        SL: Primitive,
        SR: Primitive
    {
        if simplex.len() < 4 {
            return None;
        }
        let mut polytope = Polytope::new(&mut simplex);
        let mut i = 1;
        loop {
            let p = {
                let face = polytope.closest_face_to_origin();
                let p = SupportPoint::from_minkowski(
                    left,
                    left_transform,
                    right,
                    right_transform,
                    &face.normal,
                );
                let d = p.v.dot(face.normal);
                if d - face.distance < self.tolerance || i >= self.max_iterations {
                    return contact(&polytope, face);
                }
                p
            };
            polytope.add(p);
            i += 1;
        }
    }

    fn new() -> Self {
        Self::new_with_tolerance(EPA_TOLERANCE, MAX_ITERATIONS)
    }

    fn new_with_tolerance(
        tolerance: f32,
        max_iterations: u32,
    ) -> Self {
        Self {
            tolerance,
            max_iterations,
        }
    }
}

#[inline]
fn contact(polytope: &Polytope, face: &Face) -> Option<Contact> {
    Some(Contact::new_with_point(
        CollisionStrategy::FullResolution,
        face.normal, // negate ?
        face.distance,
        point(polytope, face),
    ))
}

/// This function returns the contact point in world space coordinates on shape A.
///
/// Compute the closest point to the origin on the given simplex face, then use that to interpolate
/// the support points coming from the A shape.
fn point(polytope: &Polytope, face: &Face) -> Vec3 {
    let (u, v, w) = barycentric_vector(
        face.normal * face.distance,
        polytope.vertices[face.vertices[0]].v,
        polytope.vertices[face.vertices[1]].v,
        polytope.vertices[face.vertices[2]].v,
    );

    polytope.vertices[face.vertices[0]].sup_a * u
        + polytope.vertices[face.vertices[1]].sup_a * v
        + polytope.vertices[face.vertices[2]].sup_a * w
}

#[derive(Debug)]
struct Polytope<'a> {
    vertices: &'a mut Vec<SupportPoint>,
    faces: Vec<Face>,
}

impl<'a> Polytope<'a> {
    pub fn new(simplex: &'a mut Vec<SupportPoint>) -> Self {
        let faces = Face::new(simplex);
        Self {
            vertices: simplex,
            faces,
        }
    }

    pub fn closest_face_to_origin(&'a self) -> &'a Face {
        let mut face = &self.faces[0];
        for f in self.faces[1..].iter() {
            if f.distance < face.distance {
                face = f;
            }
        }
        face
    }

    pub fn add(&mut self, sup: SupportPoint) {
        // remove faces that can see the point
        let mut edges = Vec::default();
        let mut i = 0;
        while i < self.faces.len() {
            let dot = self.faces[i]
                .normal
                .dot(sup.v - self.vertices[self.faces[i].vertices[0]].v);
            if dot > 0. {
                let face = self.faces.swap_remove(i);
                remove_or_add_edge(&mut edges, (face.vertices[0], face.vertices[1]));
                remove_or_add_edge(&mut edges, (face.vertices[1], face.vertices[2]));
                remove_or_add_edge(&mut edges, (face.vertices[2], face.vertices[0]));
            } else {
                i += 1;
            }
        }

        // add vertex
        let n = self.vertices.len();
        self.vertices.push(sup);

        // add new faces
        let new_faces = edges
            .into_iter()
            .map(|(a, b)| Face::new_impl(self.vertices, n, a, b))
            .collect::<Vec<_>>();
        self.faces.extend(new_faces);
    }
}

#[derive(Debug)]
struct Face {
    pub vertices: [usize; 3],
    pub normal: Vec3,
    pub distance: f32,
}

impl Face {
    fn new_impl(simplex: &[SupportPoint], a: usize, b: usize, c: usize) -> Self {
        let ab = simplex[b].v - simplex[a].v;
        let ac = simplex[c].v - simplex[a].v;
        let normal = ab.cross(ac).normalize();
        let distance = normal.dot(simplex[a].v);
        Self {
            vertices: [a, b, c],
            normal,
            distance,
        }
    }

    pub fn new(simplex: &[SupportPoint]) -> Vec<Self> {
        vec![
            Self::new_impl(simplex, 3, 2, 1), // ABC
            Self::new_impl(simplex, 3, 1, 0), // ACD
            Self::new_impl(simplex, 3, 0, 2), // ADB
            Self::new_impl(simplex, 2, 0, 1), // BDC
        ]
    }
}

#[inline]
fn remove_or_add_edge(edges: &mut Vec<(usize, usize)>, edge: (usize, usize)) {
    match edges.iter().position(|e| edge.0 == e.1 && edge.1 == e.0) {
        Some(i) => {
            edges.remove(i);
        }
        None => edges.push(edge),
    }
}

#[cfg(test)]
mod tests {
    use glam::{Mat4, Quat, Vec3};
    use crate::primitive::Cuboid;

    use super::*;

    #[test]
    fn test_remove_or_add_edge_added() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (4, 3));
        assert_eq!(3, edges.len());
        assert_eq!((4, 3), edges[2]);
    }

    #[test]
    fn test_remove_or_add_edge_removed() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (2, 1));
        assert_eq!(1, edges.len());
        assert_eq!((6, 5), edges[0]);
    }

    #[test]
    fn test_face_impl() {
        let simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let faces = Face::new(&simplex);
        assert_eq!(4, faces.len());
        assert_face(
            &faces[0],
            3,
            2,
            1,
            -0.872_871_5,
            0.436_435_76,
            0.218_217_88,
            1.091_089_4,
        );
        assert_face(&faces[1], 3, 1, 0, 0., -0.894_427_24, 0.447_213_62, 2.236_068);
        assert_face(
            &faces[2],
            3,
            0,
            2,
            0.872_871_5,
            0.436_435_76,
            0.218_217_88,
            1.091_089_4,
        );
        assert_face(&faces[3], 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_closest_to_origin() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let polytope = Polytope::new(&mut simplex);
        let face = polytope.closest_face_to_origin();
        assert_face(face, 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_add() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let mut polytope = Polytope::new(&mut simplex);
        polytope.add(sup(0., 0., -2.));
        assert_eq!(5, polytope.vertices.len());
        assert_eq!(6, polytope.faces.len());
        assert_eq!([4, 2, 0], polytope.faces[3].vertices);
        assert_eq!([4, 0, 1], polytope.faces[4].vertices);
        assert_eq!([4, 1, 2], polytope.faces[5].vertices);
    }

    #[test]
    fn test_epa_3d() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let mut simplex = vec![
            sup(18., -12., 0.),
            sup(-2., 8., 0.),
            sup(-2., -12., 0.),
            sup(8., -2., -10.),
        ];
        let contact = EPA3::new().process(
            &mut simplex,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(Vec3::new(-1., 0., 0.), contact.normal);
        assert_eq!(2., contact.penetration_depth);
    }

    #[allow(clippy::too_many_arguments)]
    fn assert_face(
        face: &Face,
        a: usize,
        b: usize,
        c: usize,
        nx: f32,
        ny: f32,
        nz: f32,
        d: f32,
    ) {
        assert_eq!([a, b, c], face.vertices);
        assert_eq!(nx, face.normal.x());
        assert_eq!(ny, face.normal.y());
        assert_eq!(nz, face.normal.z());
        assert_eq!(d, face.distance);
    }

    fn sup(x: f32, y: f32, z: f32) -> SupportPoint {
        let mut s = SupportPoint::new();
        s.v = Vec3::new(x, y, z);
        s
    }

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
}