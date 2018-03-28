//! Implements spatial data structures.
//!
//! Currently, the [`Octree`](struct.Octree.html) is the only supported data structure.

extern crate aitios_geom as geom;

mod octants;
mod octree;

pub use octree::Octree;
