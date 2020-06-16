use std::cmp::Ordering;
use std::collections::HashMap;

use bit_set::BitSet;
use glam::{Vec3, Mat4};

use crate::{Aabb3, plane::Plane, ray::Ray, traits::*};
use crate::primitive::{Primitive, util::barycentric_point};
use crate::volume::Sphere;

use crate::volume::{Aabb};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
enum PolyhedronMode {
    VertexOnly,
    HalfEdge,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
struct Vertex {
    position: Vec3,
    edge: usize,
    ready: bool,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
struct Edge {
    target_vertex: usize,
    left_face: usize,
    next_edge: usize,
    previous_edge: usize,
    twin_edge: usize,
    ready: bool,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
struct Face {
    edge: usize,
    vertices: (usize, usize, usize),
    plane: Plane,
    ready: bool,
}

/// Convex polyhedron primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. It is recommended for high vertex counts, to also provide the
/// faces, this will cause the support function to use hill climbing on a half edge structure,
/// resulting in better performance. The breakpoint is around 250 vertices, but the face version is
/// only marginally slower on lower vertex counts (about 1-2%), while for higher vertex counts it's
/// about 2-5 times faster.
///
///
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConvexPolyhedron {
    mode: PolyhedronMode,
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    faces: Vec<Face>,
    bound: Aabb3,
    max_extent: f32,
}

impl ConvexPolyhedron {
    /// Create a new convex polyhedron from the given vertices.
    pub fn new(vertices: Vec<Vec3>) -> Self {
        Self {
            mode: PolyhedronMode::VertexOnly,
            vertices: vertices
                .iter()
                .map(|&v| Vertex {
                    position: v,
                    edge: 0,
                    ready: true,
                })
                .collect(),
            edges: Vec::default(),
            faces: Vec::default(),
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(*p)),
            max_extent: vertices
                .iter()
                // TODO
                .map(|p| (p.dot(*p)).sqrt())
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal))
                .unwrap_or_else(|| 0.),
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces.
    pub fn new_with_faces(vertices: Vec<Vec3>, faces: Vec<(usize, usize, usize)>) -> Self {
        let (vertices, edges, faces) = build_half_edges(&vertices, &faces);
        Self {
            mode: PolyhedronMode::HalfEdge,
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(p.position)),
            max_extent: vertices
                .iter()
                .map(|p| (p.position.dot(p.position)).sqrt())
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal))
                .unwrap_or_else(|| 0.),
            vertices,
            edges,
            faces,
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces. Will remove any duplicate
    /// vertices.
    pub fn new_with_faces_dedup(
        vertices: Vec<Vec3>,
        faces: Vec<(usize, usize, usize)>,
    ) -> Self {
        let (vertices, map) = dedup_vertices(&vertices);
        Self::new_with_faces(vertices, dedup_faces(&faces, &map))
    }

    /// Return an iterator that will yield tuples of the 3 vertices of each face
    pub fn faces_iter(&self) -> FaceIterator {
        assert_eq!(self.mode, PolyhedronMode::HalfEdge);
        FaceIterator {
            polyhedron: self,
            current: 0,
        }
    }

    #[inline]
    fn brute_force_support_point(&self, direction: Vec3) -> Vec3 {
        let (p, _) = self.vertices
            .iter()
            .map(|v| (v.position, v.position.dot(direction)))
            .fold(
                (Vec3::zero(), f32::NEG_INFINITY),
                |(max_p, max_dot), (v, dot)| {
                    if dot > max_dot {
                        (v, dot)
                    } else {
                        (max_p, max_dot)
                    }
                },
            );
        p
    }

    #[inline]
    fn hill_climb_support_point(&self, direction: Vec3) -> Vec3 {
        let mut best_index = 0;
        let mut best_dot = self.vertices[best_index].position.dot(direction);

        loop {
            let previous_best_dot = best_dot;
            let mut edge_index = self.vertices[best_index].edge;
            let start_edge_index = edge_index;

            loop {
                let vertex_index = self.edges[edge_index].target_vertex;

                let dot = self.vertices[vertex_index].position.dot(direction);
                if dot > best_dot {
                    best_index = vertex_index;
                    best_dot = dot;
                }

                edge_index = self.edges[self.edges[edge_index].twin_edge].next_edge;
                if start_edge_index == edge_index {
                    break;
                }
            }

            if (best_dot - previous_best_dot).abs() < std::f32::EPSILON {
                break;
            }
        }

        self.vertices[best_index].position
    }
}

/// Iterate over polyhedron faces.
/// Yields a tuple with the positions of the 3 vertices of each face
pub struct FaceIterator<'a> {
    polyhedron: &'a ConvexPolyhedron,
    current: usize,
}

impl<'a> Iterator for FaceIterator<'a> {
    type Item = (&'a Vec3, &'a Vec3, &'a Vec3);

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.polyhedron.faces.len() {
            None
        } else {
            self.current += 1;
            let face = &self.polyhedron.faces[self.current - 1];
            Some((
                &self.polyhedron.vertices[face.vertices.0].position,
                &self.polyhedron.vertices[face.vertices.1].position,
                &self.polyhedron.vertices[face.vertices.2].position,
            ))
        }
    }
}

fn dedup_vertices(vertices: &[Vec3]) -> (Vec<Vec3>, HashMap<usize, usize>) {
    let mut vs: Vec<Vec3> = Vec::with_capacity(vertices.len() / 2);
    let mut dup = HashMap::default();
    for (i, &vertex) in vertices.iter().enumerate() {
        let mut found = false;
        for (j, &v) in vs.iter().enumerate() {
            if v.cmpeq(vertex).all() {
                dup.insert(i, j);
                found = true;
            }
        }
        if !found {
            vs.push(vertex);
        }
    }
    (vs, dup)
}

fn dedup_faces(
    faces: &[(usize, usize, usize)],
    duplicates: &HashMap<usize, usize>,
) -> Vec<(usize, usize, usize)> {
    faces
        .iter()
        .map(|&(a, b, c)| {
            (
                *duplicates.get(&a).unwrap_or(&a),
                *duplicates.get(&b).unwrap_or(&b),
                *duplicates.get(&c).unwrap_or(&c),
            )
        })
        .collect()
}

/// Create half edge data structure from vertices and faces
fn build_half_edges(
    vertices: &[Vec3],
    in_faces: &[(usize, usize, usize)],
) -> (Vec<Vertex>, Vec<Edge>, Vec<Face>) {
    let mut vertices: Vec<Vertex> = vertices
        .iter()
        .map(|&v| Vertex {
            position: v,
            edge: 0,
            ready: false,
        })
        .collect();
    let mut edges: Vec<Edge> = vec![];
    let mut faces: Vec<Face> = vec![];
    let mut edge_map: HashMap<(usize, usize), usize> = HashMap::default();
    for &(a, b, c) in in_faces {
        let face_vertices = [a, b, c];
        let mut face = Face {
            edge: 0,
            vertices: (a, b, c),
            plane: Plane::from_points(
                vertices[a].position,
                vertices[b].position,
                vertices[c].position,
            ).unwrap(),
            ready: false,
        };
        let face_index = faces.len();
        let mut face_edge_indices = [0, 0, 0];
        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let v0 = face_vertices[i];
            let v1 = face_vertices[j];

            let (edge_v0_v1_index, edge_v1_v0_index) = if let Some(edge) = edge_map.get(&(v0, v1)) {
                (*edge, edges[*edge].twin_edge)
            } else {
                let edge_v0_v1_index = edges.len();
                let edge_v1_v0_index = edges.len() + 1;

                edges.push(Edge {
                    target_vertex: v1,
                    left_face: 0,
                    next_edge: 0,
                    previous_edge: 0,
                    twin_edge: edge_v1_v0_index,
                    ready: false,
                });

                edges.push(Edge {
                    target_vertex: v0,
                    left_face: 0,
                    next_edge: 0,
                    previous_edge: 0,
                    twin_edge: edge_v0_v1_index,
                    ready: false,
                });

                (edge_v0_v1_index, edge_v1_v0_index)
            };

            edge_map.insert((v0, v1), edge_v0_v1_index);
            edge_map.insert((v1, v0), edge_v1_v0_index);

            if !edges[edge_v0_v1_index].ready {
                edges[edge_v0_v1_index].left_face = face_index;
                edges[edge_v0_v1_index].ready = true;
            }

            if !vertices[v0].ready {
                vertices[v0].edge = edge_v0_v1_index;
                vertices[v0].ready = true;
            }

            if !face.ready {
                face.edge = edge_v0_v1_index;
                face.ready = true;
            }

            face_edge_indices[i] = edge_v0_v1_index;
        }

        faces.push(face);

        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let edge_i = face_edge_indices[i];
            let edge_j = face_edge_indices[j];
            edges[edge_i].next_edge = edge_j;
            edges[edge_j].previous_edge = edge_i;
        }
    }

    (vertices, edges, faces)
}

impl Primitive for ConvexPolyhedron {
    fn support_point(&self, direction: &Vec3, transform: &Mat4) -> Vec3 {
        let p = match self.mode {
            PolyhedronMode::VertexOnly => self.brute_force_support_point(
                transform.inverse().transform_vector3(*direction),
            ),

            PolyhedronMode::HalfEdge => self.hill_climb_support_point(
                transform.inverse().transform_vector3(*direction),
            ),
        };
        transform.transform_point3(p)
    }
}

impl ComputeBound<Aabb3> for ConvexPolyhedron {
    fn compute_bound(&self) -> Aabb3 {
        self.bound
    }
}

impl ComputeBound<Sphere> for ConvexPolyhedron {
    fn compute_bound(&self) -> Sphere {
        Sphere {
            center: Vec3::zero(),
            radius: self.max_extent,
        }
    }
}

/// TODO: better algorithm for finding faces to intersect with?
impl Discrete<Ray> for ConvexPolyhedron{
    /// Ray must be in object space
    fn intersects(&self, ray: &Ray) -> bool {
        find_intersecting_face(self, ray).is_some()
    }
}

impl Continuous<Ray> for ConvexPolyhedron {
    type Result = Vec3;

    /// Ray must be in object space
    fn intersection(&self, ray: &Ray) -> Option<Vec3> {
        find_intersecting_face(self, ray).and_then(|(face_index, (u, v, w))| {
            let f = &self.faces[face_index];
            let v0 = f.vertices.0;
            let v1 = f.vertices.1;
            let v2 = f.vertices.2;
            let p = (self.vertices[v0].position * u) + (self.vertices[v1].position * v)
                + (self.vertices[v2].position * w);
            Some(p)
        })
    }
}

// TODO: better algorithm for finding faces to intersect with?
// Current algorithm walks in the direction of the plane/ray intersection point for the current
// tested face, assuming the intersection point isn't on the actual face.
// It uses barycentric coordinates to find out where to walk next.
#[inline]
fn find_intersecting_face(
    polytope: &ConvexPolyhedron,
    ray: &Ray,
) -> Option<(usize, (f32, f32, f32))> {
    let mut face = Some(0);
    let mut checked = BitSet::with_capacity(polytope.faces.len());
    while face.is_some() {
        let face_index = face.unwrap();
        checked.insert(face_index);
        let uvw = match intersect_ray_face(ray, polytope, &polytope.faces[face_index]) {
            Some((u, v, w)) => {
                if in_range(u) && in_range(v) && in_range(w) {
                    return Some((face_index, (u, v, w)));
                }
                Some((u, v, w))
            }
            _ => None,
        };
        face = next_face_classify(polytope, face_index, uvw, &mut checked);
    }
    None
}

#[inline]
fn in_range(v: f32) -> bool {
    v >= 0. && v <= 1.
}

#[inline]
fn next_face_classify(
    polytope: &ConvexPolyhedron,
    face_index: usize,
    bary_coords: Option<(f32, f32, f32)>,
    checked: &mut BitSet,
) -> Option<usize>
where {
    if polytope.faces.len() < 10 {
        if face_index == polytope.faces.len() - 1 {
            None
        } else {
            Some(face_index + 1)
        }
    } else {
        match bary_coords {
            None => {
                let mut next = face_index + 1;
                while next < polytope.faces.len() && checked.contains(next) {
                    next += 1;
                }
                if next == polytope.faces.len() {
                    None
                } else {
                    Some(next)
                }
            }

            Some((u, v, _)) => {
                let face = &polytope.faces[face_index];
                let target_vertex_index = if u < 0. {
                    face.vertices.2
                } else if v < 0. {
                    face.vertices.0
                } else {
                    face.vertices.1
                };

                let face_edge = &polytope.edges[face.edge];

                let edges = if face_edge.target_vertex == target_vertex_index {
                    [face.edge, face_edge.next_edge, face_edge.previous_edge]
                } else if polytope.edges[face_edge.previous_edge].target_vertex
                    == target_vertex_index
                {
                    [face_edge.previous_edge, face.edge, face_edge.next_edge]
                } else {
                    [face_edge.next_edge, face_edge.previous_edge, face.edge]
                };

                for edge_index in &edges {
                    let twin_edge = polytope.edges[*edge_index].twin_edge;
                    if !checked.contains(polytope.edges[twin_edge].left_face) {
                        return Some(polytope.edges[twin_edge].left_face);
                    }
                }

                for i in 0..polytope.faces.len() {
                    if !checked.contains(i) {
                        return Some(i);
                    }
                }

                None
            }
        }
    }
}

/// Compute intersection point of ray and face in barycentric coordinates.
#[inline]
fn intersect_ray_face(
    ray: &Ray,
    polytope: &ConvexPolyhedron,
    face: &Face,
) -> Option<(f32, f32, f32)> {
    let n_dir = face.plane.n.dot(ray.direction);
    if n_dir < 0. {
        let v0 = face.vertices.0;
        let v1 = face.vertices.1;
        let v2 = face.vertices.2;
        face.plane.intersection(ray).map(|p| {
            barycentric_point(
                p,
                polytope.vertices[v0].position,
                polytope.vertices[v1].position,
                polytope.vertices[v2].position,
            )
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {

    use glam::{Vec3, Mat4, Quat};

    use super::ConvexPolyhedron;
    use crate::{Aabb3, ray::Ray, traits::*};

    #[test]
    fn test_polytope_half_edge() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope_with_faces = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let polytope = ConvexPolyhedron::new(vertices);

        let t = transform(0., 0., 0., 0.);

        let direction = Vec3::new(1., 0., 0.);
        assert_eq!(
            Vec3::new(1., 0., 0.),
            polytope.support_point(&direction, &t)
        );
        assert_eq!(
            Vec3::new(1., 0., 0.),
            polytope_with_faces.support_point(&direction, &t)
        );

        let direction = Vec3::new(0., 1., 0.);
        assert_eq!(
            Vec3::new(0., 1., 0.),
            polytope.support_point(&direction, &t)
        );
        assert_eq!(
            Vec3::new(0., 1., 0.),
            polytope_with_faces.support_point(&direction, &t)
        );
    }

    #[test]
    fn test_polytope_bound() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        assert_eq!(
            Aabb3::new(Vec3::new(0., 0., 0.), Vec3::new(1., 1., 1.)),
            polytope.compute_bound()
        );
    }

    #[test]
    fn test_ray_discrete() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        assert!(polytope.intersects(&ray));
        let ray = Ray::new(Vec3::new(0.5, 5., 0.5), Vec3::new(0., 1., 0.));
        assert!(!polytope.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let t = transform(0., 0., 0., 0.);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &t));
        let ray = Ray::new(Vec3::new(0.5, 5., 0.5), Vec3::new(0., 1., 0.));
        assert!(!polytope.intersects_transformed(&ray, &t));
        let t = transform(0., 1., 0., 0.);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &t));
        let t = transform(0., 0., 0., 0.3);
        assert!(polytope.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        let p = polytope.intersection(&ray).unwrap();
        assert!((p.x() - 0.250_000_18).abs() < std::f32::EPSILON);
        assert!((p.y() - 0.499_999_7).abs() < std::f32::EPSILON);
        assert!((p.z() - 0.250_000_18).abs() < std::f32::EPSILON);
        let ray = Ray::new(Vec3::new(0.5, 5., 0.5), Vec3::new(0., 1., 0.));
        assert_eq!(None, polytope.intersection(&ray));
        let ray = Ray::new(Vec3::new(0., 5., 0.), Vec3::new(0., -1., 0.));
        assert_eq!(Some(Vec3::new(0., 1., 0.)), polytope.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let t = transform(0., 0., 0., 0.);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert!((p.x() - 0.250_000_18).abs() < std::f32::EPSILON);
        assert!((p.y() - 0.499_999_7).abs() < std::f32::EPSILON);
        assert!((p.z() - 0.250_000_18).abs() < std::f32::EPSILON);
        let ray = Ray::new(Vec3::new(0.5, 5., 0.5), Vec3::new(0., 1., 0.));
        assert!(None, polytope.intersection_transformed(&ray, &t));
        let t = transform(0., 1., 0., 0.);
        let ray = Ray::new(Vec3::new(0.25, 5., 0.25), Vec3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert!((p.x() - 0.250_000_18).abs() < std::f32::EPSILON);
        assert!((p.y() - 1.499_999_7).abs() < std::f32::EPSILON);
        assert!((p.z() - 0.250_000_18).abs() < std::f32::EPSILON);
        let t = transform(0., 0., 0., 0.3);
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert!((p.x() - 0.25).abs() < std::f32::EPSILON);
        assert!((p.y() - 0.467_716_2).abs() < std::f32::EPSILON);
        assert!((p.z() - 0.25).abs() < std::f32::EPSILON);
    }

    #[test]
    fn test_intersect_face() {
        let vertices = vec![
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
            Vec3::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(vertices.clone(), faces);
        let ray = Ray::new(Vec3::new(1., -1., 1.), Vec3::new(0., 1., 0.));
        polytope.intersection(&ray);
    }

    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Mat4 {
        let scale = Vec3::splat(1.);
        let rotation = Quat::from_rotation_z(rot.to_radians());
        let tran = Vec3::new(dx, dy, dz);
        Mat4::from_scale_rotation_translation(scale, rotation, tran)
    }
}