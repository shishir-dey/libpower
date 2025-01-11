extern crate test; // Import the built-in test crate for benchmarks

use libpower::control::pid::PID;
use test::Bencher; // Import Bencher from the test crate for benchmarking

#[bench]
fn pid_update_benchmark(b: &mut Bencher) {
    // Initialize the PID controller
    let mut pid = PID::new(1.0, 0.1, 0.01);

    // Define test parameters
    let setpoint = 10.0;
    let mut current_position = 8.0;
    let mut current_time = 0.0;

    // Benchmark the `update` method
    b.iter(|| {
        // Simulate multiple updates with increasing time
        current_time += 0.1; // Increment time by 0.1s
        current_position += 0.1; // Simulate movement toward the setpoint
        pid.update(setpoint, current_position, current_time)
    });
}
