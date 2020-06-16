use std::fmt::Debug;

use glam::Vec3;

use super::TreeValue;
use crate::{traits::*};

/// Value together with bounding volume, for use with DBVT.
#[derive(Debug, Clone)]
pub struct TreeValueWrapped<V, B>
where
    B: Bound,
{
    /// The value
    pub value: V,

    /// The bounding volume
    pub bound: B,

    margin: Vec3,
}

impl<V, B> TreeValueWrapped<V, B>
where
    B: Bound,
{
    /// Create a new shape
    pub fn new(value: V, bound: B, margin: Vec3) -> Self {
        Self {
            value,
            bound,
            margin,
        }
    }
}

impl<V, B> TreeValue for TreeValueWrapped<V, B>
where
    V: Clone,
    B: Bound + Clone,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }

    fn get_bound_with_margin(&self) -> Self::Bound {
        self.bound.with_margin(self.margin)
    }
}

impl<V, B> HasBound for TreeValueWrapped<V, B>
where
    B: Bound,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }
}


impl<V, B> From<(V, B, Vec3)> for TreeValueWrapped<V, B>
where
    B: Bound + Clone,
{
    fn from((value, bound, margin): (V, B, Vec3)) -> Self {
        Self::new(value, bound, margin)
    }
}

impl<V, B> From<(V, B)> for TreeValueWrapped<V, B>
where
    B: Bound + Clone,
{
    fn from((value, bound): (V, B)) -> Self {
        Self::new(
            value,
            bound,
            Vec3::zero(),
        )
    }
}