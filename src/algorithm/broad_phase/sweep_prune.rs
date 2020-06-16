pub use self::variance::Variance;

use std::cmp::Ordering;

use self::variance::{Variance3};
use crate::traits::*;

/// Broad phase sweep and prune algorithm for 3D, see
/// [SweepAndPrune](struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune3<B> = SweepAndPrune<Variance3<B>>;

/// Sweep and prune broad phase collision detection algorithm.
///
/// Will sort the bounding boxes of the collision world along some axis, and will then sweep the
/// sorted list, and compare the bounds along the sweep axis, adding them to an active list when
/// they are encountered, and removing them from the active list when the end extent is passed.
///
/// Any shape pairs found by the base algorithm, will then do a bounding box intersection test,
/// before adding to the resulting pairs list.
///
/// # Type parameters:
///
/// - `V`: Variance type used for computing what axis to use on the next iteration.
///        [SweepAndPrune2](type.SweepAndPrune2.html) and [SweepAndPrune3](type.SweepAndPrune3.html)
///        provide a variance type for you, so they should be used if you do not have a custom type
///        implementing [Variance](trait.Variance.html).
pub struct SweepAndPrune<V> {
    sweep_axis: usize,
    variance: V,
}

impl<V> SweepAndPrune<V>
where
    V: Variance,
{
    /// Create a new sweep and prune algorithm, will use the X axis as the first sweep axis
    pub fn new() -> Self {
        Self::with_sweep_axis(0)
    }

    /// Create a new sweep and prune algorithm, starting with the given axis as the first sweep axis
    pub fn with_sweep_axis(sweep_axis: usize) -> Self {
        Self {
            sweep_axis,
            variance: V::new(),
        }
    }

    /// Get sweep axis
    pub fn get_sweep_axis(&self) -> usize {
        self.sweep_axis
    }

    /// Find all potentially colliding pairs of shapes
    ///
    /// ## Parameters
    ///
    /// - `shapes`: Shapes to do find potential collisions for
    ///
    /// ## Returns
    ///
    /// Returns tuples with indices into the shapes list, of all potentially colliding pairs.
    /// The first value in the tuple will always be first in the list.
    ///
    /// ## Side effects:
    ///
    /// The shapes list might have been resorted. The indices in the return values will be for the
    /// sorted list.
    pub fn find_collider_pairs<A>(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)>
    where
        A: HasBound,
        A::Bound: Bound + Discrete<A::Bound>,
        V: Variance<Bound = A::Bound>,
    {
        let mut pairs = Vec::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        shapes.sort_by(|a, b| {
            let cmp_min = a.bound().min_extent()[self.sweep_axis]
                .partial_cmp(&b.bound().min_extent()[self.sweep_axis]);
            match cmp_min {
                Some(Ordering::Equal) => a.bound().max_extent()[self.sweep_axis]
                    .partial_cmp(&b.bound().max_extent()[self.sweep_axis])
                    .unwrap_or(Ordering::Equal),
                None => Ordering::Equal,
                Some(order) => order,
            }
        });

        self.variance.clear();
        self.variance.add_bound(shapes[0].bound());

        let mut active = vec![0];
        // Remember that the index here will be the index of the iterator, which starts at index 1
        // in the shapes list, so the real shape index is + 1
        for (index, shape) in shapes[1..].iter().enumerate() {
            let shape_index = index + 1;
            // for all currently active bounds, go through and remove any that are to the left of
            // the current bound
            active.retain(|active_index| {
                shapes[*active_index].bound().max_extent()[self.sweep_axis]
                    >= shape.bound().min_extent()[self.sweep_axis]
            });

            // all shapes in the active list are potential hits, do a real bound intersection test
            // for those, and add to pairs if the bounds intersect.
            for active_index in &active {
                if shapes[*active_index].bound().intersects(shape.bound()) {
                    pairs.push((*active_index, shape_index));
                }
            }

            // current bound should be active for the next iteration
            active.push(shape_index);

            // update variance
            self.variance.add_bound(shape.bound());
        }

        // compute sweep axis for the next iteration
        let (axis, _) = self.variance
            .compute_axis(shapes.len() as f32);
        self.sweep_axis = axis;

        pairs
    }
}

mod variance {
    use std::marker;

    use crate::traits::Bound;
    use glam::Vec3;

    /// Trait for variance calculation in sweep and prune algorithm
    pub trait Variance {
        /// Point type
        type Bound: Bound;

        /// Create new variance object
        fn new() -> Self;

        /// Clear variance sums
        fn clear(&mut self);

        /// Add an extent to the variance sums
        fn add_bound(&mut self, bound: &Self::Bound);

        /// Compute the sweep axis based on the internal values
        fn compute_axis(&self, n: f32 ) -> (usize, f32);
    }

    /// Variance for 3D sweep and prune
    #[derive(Debug)]
    pub struct Variance3<B> {
        csum: Vec3,
        csumsq: Vec3,
        m: marker::PhantomData<B>,
    }

    impl<B> Variance for Variance3<B>
    where
        B: Bound,
    {
        type Bound = B;

        fn new() -> Self {
            Self {
                csum: Vec3::zero(),
                csumsq: Vec3::zero(),
                m: marker::PhantomData,
            }
        }

        fn clear(&mut self) {
            self.csum = Vec3::zero();
            self.csumsq = Vec3::zero();
        }

        #[inline]
        fn add_bound(&mut self, bound: &B) {
            let min_vec = bound.min_extent();
            let max_vec = bound.max_extent();
            let sum = min_vec + max_vec;
            let c = sum / 2.0;
            self.csum += c;
            self.csumsq += c * c;
        }

        #[inline]
        fn compute_axis(&self, n: f32) -> (usize, f32) {
            let square_n = self.csum * self.csum / n;
            let variance = self.csumsq * square_n;
            let mut sweep_axis = 0;
            let mut sweep_variance = variance[0];
            for i in 1..3 {
                let v = variance[i];
                if v > sweep_variance {
                    sweep_axis = i;
                    sweep_variance = v;
                }
            }
            (sweep_axis, sweep_variance)
        }
    }
}
