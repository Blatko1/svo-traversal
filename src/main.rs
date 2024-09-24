mod clean;

use nalgebra::{Vector2, Vector3};

struct Node {
    child_index: usize,
    valid_mask: u8,
    leaf_mask: u8,
}

struct SavedNode<'a> {
    node: &'a Node,
    voxel_pos: Vector2<u32>
}

impl<'a> SavedNode<'a> {
    fn new(node: &'a Node, voxel_pos: Vector2<u32>) -> Self {
        Self {
            node,
            voxel_pos,
        }
    }
}

fn main() {
    let octree = vec![
        Node {
            child_index: 1,
            valid_mask: 0b01000000,
            leaf_mask:  0b00000000,
        },
        Node {
            child_index: 3,
            valid_mask: 0b01000000,
            leaf_mask:  0b00000000,
        },
        Node {
            child_index: 0,
            valid_mask: 0b01010000,
            leaf_mask:  0b01010000,
        },
        Node {
            child_index: 0,
            valid_mask: 0b01000000,
            leaf_mask:  0b01000000,
        },
    ];
    let mut size = 32.0;
    let dir = Vector2::new(0.8017f32, 0.1931f32).normalize();
    let mut origin = Vector2::new(0.0f32, 2.0f32);
    let delta_dist = Vector2::new(dir.x.abs().recip(), dir.y.abs().recip());
    let step = Vector2::new(dir.x.signum(), dir.y.signum());
    size *= 0.5;
    let origin_fract = modulo_vec2_dir(origin, size, dir);
    println!("START: ORIGIN_FRACT: x:{} y:{}", origin_fract.x, origin_fract.y);
    let mut node = &octree[0];
    let mut depth = 0;
    let mut stack = Vec::new();
    //println!("modulo: {} {} {}", modulo(-31.55, 32.0), modulo2(-31.55, 32.0), -31.55 % 32.0);
    for i in 0..10 {
        println!("step {}:", i);
        if depth < 5 {
            let local_pos = modulo_vec2_dir2(origin, 32.0, dir, size);
            println!("LOCAL_POS: x:{} y:{}", local_pos.x, local_pos.y);
            let vox_pos_x = (local_pos.x.floor() as u32 & (0b10000 >> depth)) >> (4 - depth);
            let vox_pos_y = (local_pos.y.floor() as u32 & (0b10000 >> depth)) >> (4 - depth);
            let vox_pos = Vector2::new(vox_pos_x, vox_pos_y);
            let child_id = vox_pos.y << 1 | vox_pos.x;
            let index: u8 = 0b10000000 >> child_id;
            println!(
                "vox_pos -> x: {} y: {},\tID -> {},\tINDEX -> {:b}",
                vox_pos.x, vox_pos.y, child_id, index
            );

            if node.valid_mask & index != 0 {
                if node.leaf_mask & index != 0 {
                    println!("HIT A LEAF!!");
                    break;
                }
                size *= 0.5;
                depth += 1;
                stack.push(SavedNode::new(node, vox_pos));
                println!("SUBDIVIDING TO DEPTH: {}, SIZE: {}", depth, size);
                let child_offset = (node.valid_mask >> (7 - child_id)).count_ones() as usize - 1;
                let child_index = child_offset + node.child_index;
                node = &octree[child_index];
                println!("ENTERING CHILD AT INDEX {}", child_id);
            } else {
                println!("ADVANCING AT DEPTH: {}", depth);
                let origin_fract = modulo_vec2_dir(origin, size, dir);
                println!("///ORIGIN_FRACT: x:{} y:{}", origin_fract.x, origin_fract.y);
                //println!("///OLD SIDE DIST -> x:{} y:{}", side_dist_x, side_dist_y);
                let side_dist_x = delta_dist.x * (size - origin_fract.x);
                let side_dist_y = delta_dist.y * (size - origin_fract.y);
                //println!("///NEW SIDE DIST -> x:{} y:{}", side_dist_x, side_dist_y);
                let mut mask = Vector2::new(0.0, 0.0);
                if side_dist_x < side_dist_y {
                    mask.x = 1.0;
                } else {
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
                    for _ in 0..8 {
                        size *= 2.0;
                        depth -= 1;
                        let saved_node = stack.pop().unwrap();
                        let new_pos = Vector2::new(
                            saved_node.voxel_pos.x as f32 + step_mask.x,
                            saved_node.voxel_pos.y as f32 + step_mask.y,
                        );
                        let pos_check = Vector2::new(
                            (saved_node.voxel_pos.x ^ mask.x as u32) as f32,
                            (saved_node.voxel_pos.y ^ mask.y as u32) as f32,
                        );
                        if new_pos == pos_check {
                            node = saved_node.node;
                            break;
                        }
                    }
                    println!("GOING UP TO DEPTH: {}, SIZE: {}", depth, size);
                }
                    if side_dist_x < side_dist_y {
                        origin += dir * side_dist_x;
                        origin.x = origin.x.round();
                        println!(
                            "RECALC DX: MOVED FOR {} TO x:{} y:{}",
                            side_dist_x, origin.x, origin.y
                        );
                    } else {
                        origin += dir * side_dist_y;
                        origin.y = origin.y.round();
                        println!(
                            "RECALC DY: MOVED FOR {} TO x:{} y:{}",
                            side_dist_y, origin.x, origin.y
                        );
                    }
            }
        } else {
            println!("__Too DEEP!!!__");
            break;
        }
        println!();
    }
}

//fn modulo(left: f32, right: f32) -> f32 {
//    ((left % right) + right) % right
//}

fn modulo_vec2_dir(mut a: Vector2<f32>, b: f32, dir: Vector2<f32>) -> Vector2<f32> {
    if dir.x < 0.0 {
        a.x = b - a.x;
    }
    if dir.y < 0.0 {
        a.y = b - a.y;
    }
    Vector2::new(((a.x % b) + b) % b, ((a.y % b) + b) % b)
}

fn modulo_vec3_dir(mut a: Vector3<f32>, b: f32, step: Vector3<f32>) -> Vector3<f32> {
    if step.x < 0.0 {
        a.x = b - a.x;
    }
    if step.y < 0.0 {
        a.y = b - a.y;
    }
    if step.z < 0.0 {
        a.z = b - a.z;
    }
    Vector3::new(((a.x % b) + b) % b, ((a.y % b) + b) % b, ((a.z % b) + b) % b)
}

fn modulo_vec3_dir_wgsl(mut a: Vector3<f32>, b: f32, step: Vector3<f32>) -> Vector3<f32> {
    a.x = -step.x * (b*0.5 - a.x) + b*0.5;
    a.y = -step.y * (b*0.5 - a.y) + b*0.5;
    a.z = -step.z * (b*0.5 - a.z) + b*0.5;
    Vector3::new(a.x - b * (a.x / b).floor(), a.y - b * (a.y / b).floor(), a.z - b * (a.z / b).floor())
}

#[test] 
fn test_modulo() {
    let a = Vector3::new(32100.0, 100.0, 320.0);
    let b = 32.0;
    let dir = Vector3::new(0.9f32, -0.2f32, -0.5f32).normalize();
    let step = Vector3::new(dir.x.signum(), dir.y.signum(), dir.z.signum());
    assert_eq!(modulo_vec3_dir(a, b, step), modulo_vec3_dir_wgsl(a, b, step));
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

//fn modulo2(left: f32, right: f32) -> f32 {
//    left - right * (left / right).floor()
// a - b * (a/b).floor()
//}
