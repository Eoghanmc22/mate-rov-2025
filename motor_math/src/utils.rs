use nalgebra::{vector, Vector3};

use crate::Number;

// https://stackoverflow.com/questions/30011741/3d-vector-defined-by-2-angles
pub fn vec_from_angles<D: Number>(angle_xy: D, angle_yz: D) -> Vector3<D> {
    let x = angle_xy.clone().cos() * angle_yz.clone().cos();
    let y = angle_xy.sin() * angle_yz.clone().cos();
    let z = angle_yz.sin();

    vector![x, y, z]
}

#[derive(Clone, Copy, Debug)]
pub enum VectorTransform {
    ReflectXY,
    ReflectYZ,
    ReflectXZ,
}

impl VectorTransform {
    pub fn transform<D: Number>(&self, vec: Vector3<D>) -> Vector3<D> {
        let [[x, y, z]] = vec.data.0;

        match self {
            VectorTransform::ReflectXY => vector![x, y, -z],
            VectorTransform::ReflectYZ => vector![-x, y, z],
            VectorTransform::ReflectXZ => vector![x, -y, z],
        }
    }
}
