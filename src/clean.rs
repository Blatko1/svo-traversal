use nalgebra::{Vector2, Vector3};

struct Node {
    child_index: usize,
    valid_mask: u8,
    leaf_mask: u8,
}

fn a() {
    let octree = vec![
        Node {
            child_index: 1,
            valid_mask: 0b00100000,
            leaf_mask:  0b00000000,
        },
        Node {
            child_index: 0,
            valid_mask: 0b00100000,
            leaf_mask:  0b00100000,
        },
        Node {
            child_index: 0,
            valid_mask: 0b01010000,
            leaf_mask:  0b01010000,
        },
    ];
    let mut size = 32.0;
    let dir = Vector2::new(0.8017f32, 0.5931f32).normalize();
    let mut origin = Vector2::new(14.6f32, 12.4f32);
    let delta_dist = Vector2::new(dir.x.abs().recip(), dir.y.abs().recip());
    let step = Vector2::new(dir.x.signum(), dir.y.signum());
    size *= 0.5;

    let origin_fract = modulo_vec2_dir(origin, size, dir);
    let mut side_dist_x = delta_dist.x * (size - origin_fract.x);
    let mut side_dist_y = delta_dist.y * (size - origin_fract.y);
    let mut local_pos = modulo_vec2_dir2(origin, 32.0, dir, size);

    let mut node = &octree[0];
    let mut depth = 0;
    //println!("modulo: {} {} {}", modulo(-31.55, 32.0), modulo2(-31.55, 32.0), -31.55 % 32.0);
    for _i in 0..10 {
        if depth < 5 {
            let vox_pos_x = (local_pos.x.floor() as u32 & (0b10000 >> depth)) >> (4 - depth);
            let vox_pos_y = (local_pos.y.floor() as u32 & (0b10000 >> depth)) >> (4 - depth);
            let vox_pos = Vector2::new(vox_pos_x, vox_pos_y);
            let vox_index = vox_pos.y << 1 | vox_pos.x;
            let pos_mask: u8 = 0b10000000 >> vox_index;

            if (node.valid_mask & pos_mask) != 0 {
                if (node.leaf_mask & pos_mask) != 0 {
                    break;
                }
                size *= 0.5;
                depth += 1;

                let origin_fract = modulo_vec2_dir(origin, size, dir);
                side_dist_x = delta_dist.x * (size - origin_fract.x);
                side_dist_y = delta_dist.y * (size - origin_fract.y);

                let child_offset = (node.valid_mask >> (7 - vox_index)).count_ones() as usize - 1;
                let child_index = child_offset + node.child_index;
                node = &octree[child_index];
                local_pos = modulo_vec2_dir2(origin, 32.0, dir, size);
            } else {
                let mut mask = Vector2::new(0.0, 0.0);
                if side_dist_x < side_dist_y {
                    origin += dir * side_dist_x;
                    origin.x = origin.x.round();
                    mask.x = 1.0;
                } else {
                    origin += dir * side_dist_y;
                    origin.y = origin.y.round();
                    mask.y = 1.0;
                }
                let step_mask = Vector2::new(mask.x * step.x, mask.y * step.y);
                let new_pos = Vector2::new(
                    vox_pos.x as f32 + step_mask.x,
                    vox_pos.y as f32 + step_mask.y,
                );
                let pos_check = Vector2::new(
                    (vox_pos.x ^ mask.x as u32) as f32,
                    (vox_pos.y ^ mask.y as u32) as f32,
                );
                if new_pos != pos_check && depth > 0 {
                    size *= 2.0;
                    depth -= 1;
                    node = &octree[0];
                }
                let origin_fract = modulo_vec2_dir(origin, size, dir);
                side_dist_x = delta_dist.x * (size - origin_fract.x);
                side_dist_y = delta_dist.y * (size - origin_fract.y);
                local_pos = modulo_vec2_dir2(origin, 32.0, dir, size);
            }
        } else {
            println!("__Too DEEP!!!__");
            break;
        }
        println!();
    }
}

fn modulo_vec2_dir(mut a: Vector2<f32>, b: f32, dir: Vector2<f32>) -> Vector2<f32> {
    if dir.x < 0.0 {
        a.x = b - a.x;
    }
    if dir.y < 0.0 {
        a.y = b - a.y;
    }
    Vector2::new(((a.x % b) + b) % b, ((a.y % b) + b) % b)
}

fn modulo_vec2_dir2(mut a: Vector2<f32>, b: f32, dir: Vector2<f32>, size: f32) -> Vector2<f32> {
    let modulo = modulo_vec2_dir(a, size, dir);
    if dir.x < 0.0 {
        a.x += modulo.x - size;
    }
    if dir.y < 0.0 {
        a.y += modulo.y - size;
    }
    Vector2::new(((a.x % b) + b) % b, ((a.y % b) + b) % b)
}
