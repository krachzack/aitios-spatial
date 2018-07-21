use geom::{Aabb, Vector3};

pub fn octants(aabb: &Aabb) -> [Aabb; 8] {
    let &Aabb { min, max } = aabb;
    let dims = max - min;
    let center = min + 0.5 * dims;

    [
        // 0: left bottom back
        Aabb {
            min: min,
            max: center,
        },
        // 1: right bottom back
        Aabb {
            min: Vector3::new(center.x, min.y, min.z),
            max: Vector3::new(max.x, center.y, center.z),
        },
        // 2: right bottom front
        Aabb {
            min: Vector3::new(center.x, min.y, center.z),
            max: Vector3::new(max.x, center.y, max.z),
        },
        // 3: left bottom front
        Aabb {
            min: Vector3::new(min.x, min.y, center.z),
            max: Vector3::new(center.x, center.y, max.z),
        },
        // 4: left top back
        Aabb {
            min: Vector3::new(min.x, center.y, min.z),
            max: Vector3::new(center.x, max.y, center.z),
        },
        // 5: right top back
        Aabb {
            min: Vector3::new(center.x, center.y, min.z),
            max: Vector3::new(max.x, max.y, center.z),
        },
        // 6: right top front
        Aabb {
            min: center,
            max: max,
        },
        // 7: left top front
        Aabb {
            min: Vector3::new(min.x, center.y, center.z),
            max: Vector3::new(center.x, max.y, max.z),
        },
    ]
}
