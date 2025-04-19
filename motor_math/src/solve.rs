pub mod forward;
pub mod reverse;

#[cfg(test)]
mod tests {
    extern crate test;
    use nalgebra::{vector, Vector3};
    use std::{collections::HashMap, time::Instant};
    use test::Bencher;

    use crate::{
        blue_rov::BlueRovMotorId,
        motor_preformance::{self},
        solve::forward,
        utils::vec_from_angles,
        x3d::X3dMotorId,
        Direction, FloatType, MotorConfig, Movement, Thruster,
    };

    use super::reverse;

    #[test]
    fn solve_roundtrip_x3d() {
        let seed_motor = Thruster {
            position: vector![1.0, 1.0, 1.0].normalize(),
            orientation: vec_from_angles(60.0, 40.0),
            direction: Direction::Clockwise,
        };

        let motor_data = motor_preformance::read_motor_data_from_path("../robot/motor_data.csv")
            .expect("Read motor data");
        let motor_config =
            MotorConfig::<X3dMotorId, FloatType>::new(seed_motor, Vector3::default());

        let movement = Movement {
            force: vector![-0.6, 0.5, 0.3],
            torque: vector![0.2, 0.1, 0.4],
        };

        let start = Instant::now();
        let forces = reverse::reverse_solve(movement, &motor_config);
        let motor_cmds = reverse::forces_to_cmds(forces, &motor_config, &motor_data);
        let elapsed = start.elapsed();

        println!("motor_cmds: {motor_cmds:#?} in {}us", elapsed.as_micros());

        let actual_movement = forward::forward_solve(
            &motor_config,
            &motor_cmds
                .iter()
                .map(|(id, data)| (*id, data.force))
                .collect(),
        );

        let movement_error = movement - actual_movement;
        assert!(movement_error.force.norm_squared() < 0.0001);
        assert!(movement_error.torque.norm_squared() < 0.0001);
    }

    #[test]
    fn solve_roundtrip_blue_rov() {
        let lateral = Thruster {
            position: vector![1.0, 1.0, 0.0],
            orientation: vector![-1.0, 1.0, 0.0].normalize(),
            direction: Direction::Clockwise,
        };
        let vertical = Thruster {
            position: vector![1.0, 1.0, 0.0],
            orientation: vector![0.0, 0.0, 1.0].normalize(),
            direction: Direction::Clockwise,
        };

        let motor_data = motor_preformance::read_motor_data_from_path("../robot/motor_data.csv")
            .expect("Read motor data");
        let motor_config =
            MotorConfig::<BlueRovMotorId, FloatType>::new(lateral, vertical, Vector3::default());

        let movement = Movement {
            force: vector![0.5, 0.1, 0.4],
            torque: vector![0.2, 0.5, -0.3],
        };

        let start = Instant::now();
        let forces = reverse::reverse_solve(movement, &motor_config);
        let motor_cmds = reverse::forces_to_cmds(forces, &motor_config, &motor_data);
        let elapsed = start.elapsed();

        println!("motor_cmds: {motor_cmds:#?} in {}us", elapsed.as_micros());

        let actual_movement = forward::forward_solve(
            &motor_config,
            &motor_cmds
                .iter()
                .map(|(id, data)| (*id, data.force))
                .collect(),
        );

        let movement_error = movement - actual_movement;
        assert!(movement_error.force.norm_squared() < 0.0001);
        assert!(movement_error.torque.norm_squared() < 0.0001);
    }

    #[test]
    fn solve_roundtrip_arbitrary() {
        let motor_data = motor_preformance::read_motor_data_from_path("../robot/motor_data.csv")
            .expect("Read motor data");

        let mut motors = HashMap::new();

        #[derive(PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Clone, Copy)]
        enum MotorIds {
            Right,
            Left,
            Lateral,
            Up1,
            Up2,
            Up3,
        }

        motors.insert(
            MotorIds::Right,
            Thruster {
                position: vector![1.0, 1.0, 0.0].normalize(),
                orientation: vector![0.0, 1.0, 0.0],
                direction: Direction::Clockwise,
            },
        );

        motors.insert(
            MotorIds::Left,
            Thruster {
                position: vector![-1.0, 1.0, 0.0].normalize(),
                orientation: vector![0.0, 1.0, 0.0],
                direction: Direction::CounterClockwise,
            },
        );

        motors.insert(
            MotorIds::Lateral,
            Thruster {
                position: vector![0.0, 0.0, 0.0],
                orientation: vector![1.0, 0.0, 0.0],
                direction: Direction::Clockwise,
            },
        );

        motors.insert(
            MotorIds::Up1,
            Thruster {
                position: vector![1.0, 1.0, 0.0].normalize() * 2.0,
                orientation: vector![0.0, 0.0, 1.0],
                direction: Direction::Clockwise,
            },
        );

        motors.insert(
            MotorIds::Up2,
            Thruster {
                position: vector![-1.0, 1.0, 0.0].normalize() * 2.0,
                orientation: vector![0.0, 0.0, 1.0],
                direction: Direction::CounterClockwise,
            },
        );

        motors.insert(
            MotorIds::Up3,
            Thruster {
                position: vector![0.0, -1.0, 0.0].normalize() * 2.0,
                orientation: vector![0.0, 0.0, 1.0],
                direction: Direction::Clockwise,
            },
        );

        let motor_config = MotorConfig::new_raw(motors, Vector3::default());

        let movement = Movement {
            force: vector![0.9, -0.5, 0.3],
            torque: vector![-0.2, 0.1, 0.4],
        };

        let start = Instant::now();
        let forces = reverse::reverse_solve(movement, &motor_config);
        let motor_cmds = reverse::forces_to_cmds(forces, &motor_config, &motor_data);
        let elapsed = start.elapsed();

        println!("motor_cmds: {motor_cmds:#?} in {}us", elapsed.as_micros());

        let actual_movement = forward::forward_solve(
            &motor_config,
            &motor_cmds
                .iter()
                .map(|(id, data)| (*id, data.force))
                .collect(),
        );

        let movement_error = movement - actual_movement;
        assert!(movement_error.force.norm_squared() < 0.0001);
        assert!(movement_error.torque.norm_squared() < 0.0001);
    }

    #[bench]
    fn bench_reverse_solver_x3d(b: &mut Bencher) {
        let seed_motor = Thruster {
            position: vector![0.3, 0.5, 0.4].normalize(),
            orientation: vec_from_angles(60.0, 40.0),
            direction: Direction::Clockwise,
        };

        let motor_data = motor_preformance::read_motor_data_from_path("../robot/motor_data.csv")
            .expect("Read motor data");
        let motor_config =
            MotorConfig::<X3dMotorId, FloatType>::new(seed_motor, Vector3::default());

        let movement = Movement {
            force: vector![0.6, 0.0, 0.3],
            torque: vector![0.2, 0.1, 0.3],
        };

        b.iter(|| {
            let forces = reverse::reverse_solve(movement, &motor_config);
            reverse::forces_to_cmds(forces, &motor_config, &motor_data)
        });
    }

    #[bench]
    fn bench_reverse_solver_blue_rov(b: &mut Bencher) {
        let lateral = Thruster {
            position: vector![1.0, 1.0, 0.0],
            orientation: vector![-1.0, 1.0, 0.0].normalize(),
            direction: Direction::Clockwise,
        };
        let vertical = Thruster {
            position: vector![1.0, 1.0, 0.0],
            orientation: vector![0.0, 0.0, 1.0].normalize(),
            direction: Direction::Clockwise,
        };

        let motor_data = motor_preformance::read_motor_data_from_path("../robot/motor_data.csv")
            .expect("Read motor data");
        let motor_config =
            MotorConfig::<BlueRovMotorId, FloatType>::new(lateral, vertical, Vector3::default());

        let movement = Movement {
            force: vector![0.6, 0.0, 0.3],
            torque: vector![0.2, 0.1, 0.3],
        };

        b.iter(|| {
            let forces = reverse::reverse_solve(movement, &motor_config);
            reverse::forces_to_cmds(forces, &motor_config, &motor_data)
        });
    }
}
