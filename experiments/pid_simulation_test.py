#!/usr/bin/env python3
"""
Lightweight PID controller simulation to analyze speed tracking behavior
WITHOUT requiring CARLA server.

This simulates the vehicle dynamics with a simple first-order model
and tests the Ackermann control PID response at different speeds.

Usage:
    python3 pid_simulation_test.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import sys

try:
    from simple_pid import PID
except ImportError:
    print("ERROR: simple_pid not found. Install with: pip install simple-pid")
    sys.exit(1)


@dataclass
class SimulationConfig:
    """Simulation parameters"""
    dt: float = 0.05  # Control loop rate (20 Hz)
    duration: float = 60.0  # Simulation duration (increased for steady state)
    vehicle_mass: float = 1500.0  # kg
    max_accel: float = 3.0  # m/s^2
    max_decel: float = 8.0  # m/s^2
    max_throttle_force: float = 5000.0  # N (approximate)
    max_brake_force: float = 15000.0  # N (approximate)
    rolling_resistance: float = 0.01
    drag_coefficient: float = 0.3
    frontal_area: float = 2.37  # m^2
    air_density: float = 1.225  # kg/m^3


class SimpleVehicleModel:
    """
    Simple vehicle dynamics model for PID testing.
    Models: throttle/brake -> force -> acceleration -> velocity
    Includes rolling resistance and aerodynamic drag.
    """

    def __init__(self, config: SimulationConfig):
        self.config = config
        self.velocity = 0.0
        self.acceleration = 0.0

    def step(self, throttle: float, brake: float, dt: float) -> Tuple[float, float]:
        """
        Simulate one timestep.
        Returns: (velocity, acceleration)
        """
        # Calculate forces
        throttle_force = throttle * self.config.max_throttle_force
        brake_force = brake * self.config.max_brake_force

        # Resistance forces (oppose motion)
        rolling_force = self.config.rolling_resistance * self.config.vehicle_mass * 9.81
        drag_force = 0.5 * self.config.drag_coefficient * self.config.frontal_area * \
                     self.config.air_density * self.velocity ** 2

        # Net force
        if self.velocity >= 0:
            resistance = rolling_force + drag_force
            net_force = throttle_force - brake_force - resistance
        else:
            net_force = 0  # Simplified: no reverse

        # Acceleration
        self.acceleration = net_force / self.config.vehicle_mass

        # Clip to physical limits
        self.acceleration = np.clip(self.acceleration, -self.config.max_decel, self.config.max_accel)

        # Integrate velocity
        self.velocity += self.acceleration * dt
        self.velocity = max(0, self.velocity)  # No reverse in this test

        return self.velocity, self.acceleration

    def reset(self):
        """Reset vehicle state"""
        self.velocity = 0.0
        self.acceleration = 0.0


class AckermannControllerSim:
    """
    Simplified Ackermann controller matching carla_ackermann_control_node.py
    """

    def __init__(self, speed_kp=0.05, speed_ki=0.0, speed_kd=0.5,
                 accel_kp=0.05, accel_ki=0.0, accel_kd=0.05,
                 min_accel=1.0, dt=0.05):

        self.dt = dt
        self.min_accel = min_accel
        self.sim_time = 0.0  # Simulated time

        # Custom time function for PIDs (to support faster-than-realtime simulation)
        def get_sim_time():
            return self.sim_time

        # PID controllers - use simulated time
        self.speed_pid = PID(Kp=speed_kp, Ki=speed_ki, Kd=speed_kd,
                            sample_time=dt, output_limits=(-1.0, 1.0),
                            time_fn=get_sim_time)
        self.accel_pid = PID(Kp=accel_kp, Ki=accel_ki, Kd=accel_kd,
                            sample_time=dt, output_limits=(-1.0, 1.0),
                            time_fn=get_sim_time)

        # Initialize time to avoid division by zero on first call
        self.sim_time = 0.1

        # State
        self.target_speed = 0.0
        self.target_accel = 0.0
        self.speed_control_activation = 0
        self.speed_control_accel_target = 0.0
        self.accel_control_pedal_target = 0.0
        self.current_accel_filtered = 0.0

        # Impedance parameters
        self.max_pedal = 3.0
        self.max_accel = 3.0
        self.max_decel = 8.0

    def set_target(self, speed: float, accel: float = 0.0):
        """Set target speed and acceleration"""
        self.target_speed = speed
        self.target_accel = accel

    def compute(self, current_speed: float, current_accel: float) -> Tuple[float, float]:
        """
        Compute throttle and brake commands.
        Returns: (throttle, brake)
        """
        # Advance simulated time
        self.sim_time += self.dt

        # Filter acceleration (matching original code)
        self.current_accel_filtered = (self.current_accel_filtered * 4 + current_accel) / 5

        # Speed control activation logic
        target_accel_abs = abs(self.target_accel)
        if target_accel_abs < self.min_accel:
            if self.speed_control_activation < 5:
                self.speed_control_activation += 1
        else:
            if self.speed_control_activation > 0:
                self.speed_control_activation -= 1

        # Run speed controller if activated
        if self.speed_control_activation >= 5:
            self.speed_pid.setpoint = self.target_speed
            speed_delta = float(self.speed_pid(current_speed))

            # Clipping
            if target_accel_abs < 0.00001:
                clip_lower = -self.max_decel
                clip_upper = self.max_accel
            else:
                clip_lower = -target_accel_abs
                clip_upper = target_accel_abs

            self.speed_control_accel_target = np.clip(
                self.speed_control_accel_target + speed_delta,
                clip_lower, clip_upper
            )
        else:
            self.speed_control_accel_target = self.target_accel

        # Run acceleration controller
        self.accel_pid.setpoint = self.speed_control_accel_target
        accel_delta = float(self.accel_pid(self.current_accel_filtered))
        self.accel_control_pedal_target = np.clip(
            self.accel_control_pedal_target + accel_delta,
            -self.max_pedal, self.max_pedal
        )

        # Calculate impedance borders (simplified)
        # Rolling resistance + drag at current speed
        drag_accel = 0.5 * 0.3 * 2.37 * 1.225 * current_speed**2 / 1500.0
        rolling_accel = 0.01 * 9.81
        throttle_lower_border = rolling_accel + drag_accel

        # Engine braking
        engine_brake_accel = 500.0 / 1500.0  # ~0.33 m/s^2
        brake_upper_border = throttle_lower_border + engine_brake_accel

        # Map to throttle/brake
        pedal = self.accel_control_pedal_target

        if pedal > throttle_lower_border:
            throttle = (pedal - throttle_lower_border) / self.max_pedal
            brake = 0.0
        elif pedal > brake_upper_border:
            throttle = 0.0
            brake = 0.0
        else:
            throttle = 0.0
            brake = (brake_upper_border - pedal) / self.max_pedal

        return np.clip(throttle, 0, 1), np.clip(brake, 0, 1)

    def reset(self):
        """Reset controller state"""
        self.speed_pid.reset()
        self.accel_pid.reset()
        self.speed_control_activation = 0
        self.speed_control_accel_target = 0.0
        self.accel_control_pedal_target = 0.0
        self.current_accel_filtered = 0.0
        self.sim_time = 0.1  # Reset simulated time


def run_simulation(target_speed_kmh: float, config: SimulationConfig, debug: bool = False) -> dict:
    """
    Run a single speed tracking simulation.
    Returns dictionary with results.
    """
    target_speed_ms = target_speed_kmh / 3.6

    vehicle = SimpleVehicleModel(config)
    controller = AckermannControllerSim(dt=config.dt)
    controller.set_target(target_speed_ms, accel=0.0)

    # Data storage
    times = []
    velocities = []
    accelerations = []
    throttles = []
    brakes = []
    errors = []
    pedal_targets = []

    num_steps = int(config.duration / config.dt)

    for step in range(num_steps):
        t = step * config.dt

        # Get control
        throttle, brake = controller.compute(vehicle.velocity, vehicle.acceleration)

        if debug and step < 20:
            print(f"  step={step} vel={vehicle.velocity:.2f} acc={vehicle.acceleration:.2f} "
                  f"thr={throttle:.3f} brk={brake:.3f} "
                  f"spd_accel_tgt={controller.speed_control_accel_target:.3f} "
                  f"pedal_tgt={controller.accel_control_pedal_target:.3f} "
                  f"speed_act={controller.speed_control_activation}")

        # Step vehicle
        vel, acc = vehicle.step(throttle, brake, config.dt)

        # Log
        times.append(t)
        velocities.append(vel)
        accelerations.append(acc)
        throttles.append(throttle)
        brakes.append(brake)
        errors.append(target_speed_ms - vel)
        pedal_targets.append(controller.accel_control_pedal_target)

    return {
        'target_speed_kmh': target_speed_kmh,
        'target_speed_ms': target_speed_ms,
        'times': np.array(times),
        'velocities': np.array(velocities),
        'accelerations': np.array(accelerations),
        'throttles': np.array(throttles),
        'brakes': np.array(brakes),
        'errors': np.array(errors),
        'pedal_targets': np.array(pedal_targets)
    }


def analyze_results(results: dict) -> dict:
    """Analyze simulation results for instability"""
    # Use data after 30 seconds (steady state - vehicle should have reached target)
    mask = results['times'] > 30.0
    ss_velocities = results['velocities'][mask]
    ss_errors = results['errors'][mask]
    ss_times = results['times'][mask]

    if len(ss_velocities) == 0:
        return {'stable': True, 'reason': 'No steady-state data'}

    # Metrics
    mean_speed = np.mean(ss_velocities)
    std_speed = np.std(ss_velocities)
    max_error = np.max(np.abs(ss_errors))
    mean_error = np.mean(ss_errors)

    # Count zero crossings (oscillation indicator)
    zero_crossings = np.sum(np.diff(np.sign(ss_errors)) != 0)
    duration = ss_times[-1] - ss_times[0]
    oscillation_freq = zero_crossings / duration / 2  # Divide by 2 for full cycles

    # Determine stability
    is_unstable = False
    reasons = []

    if std_speed * 3.6 > 1.0:
        is_unstable = True
        reasons.append(f"High variance: {std_speed*3.6:.2f} km/h std")

    if oscillation_freq > 1.0:
        is_unstable = True
        reasons.append(f"Oscillations: {oscillation_freq:.2f} Hz")

    if max_error * 3.6 > 3.0:
        is_unstable = True
        reasons.append(f"Large error: {max_error*3.6:.2f} km/h max")

    return {
        'stable': not is_unstable,
        'reasons': reasons,
        'mean_speed_kmh': mean_speed * 3.6,
        'std_speed_kmh': std_speed * 3.6,
        'mean_error_kmh': mean_error * 3.6,
        'max_error_kmh': max_error * 3.6,
        'oscillation_freq_hz': oscillation_freq
    }


def plot_results(all_results: List[dict], filename: str = 'pid_analysis.png'):
    """Create visualization of all test results"""
    n_tests = len(all_results)
    fig, axes = plt.subplots(n_tests, 3, figsize=(15, 4*n_tests))

    if n_tests == 1:
        axes = axes.reshape(1, -1)

    for i, results in enumerate(all_results):
        target_kmh = results['target_speed_kmh']
        times = results['times']
        velocities = results['velocities'] * 3.6  # Convert to km/h
        errors = results['errors'] * 3.6
        throttles = results['throttles']
        brakes = results['brakes']

        analysis = analyze_results(results)

        # Speed tracking plot
        ax1 = axes[i, 0]
        ax1.plot(times, velocities, 'b-', label='Actual', linewidth=1)
        ax1.axhline(y=target_kmh, color='r', linestyle='--', label='Target')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed (km/h)')
        ax1.set_title(f'Target: {target_kmh} km/h - {"UNSTABLE" if not analysis["stable"] else "Stable"}')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Error plot
        ax2 = axes[i, 1]
        ax2.plot(times, errors, 'g-', linewidth=1)
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax2.axhline(y=1, color='r', linestyle=':', alpha=0.5)
        ax2.axhline(y=-1, color='r', linestyle=':', alpha=0.5)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed Error (km/h)')
        ax2.set_title(f'Error (std={analysis["std_speed_kmh"]:.2f}, osc={analysis["oscillation_freq_hz"]:.2f}Hz)')
        ax2.grid(True, alpha=0.3)

        # Control signals plot
        ax3 = axes[i, 2]
        ax3.plot(times, throttles, 'b-', label='Throttle', linewidth=1)
        ax3.plot(times, brakes, 'r-', label='Brake', linewidth=1)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Pedal Position')
        ax3.set_title('Control Signals')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(-0.1, 1.1)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    print(f"\nPlot saved to: {filename}")


def main():
    print("=" * 60)
    print("PID Controller Simulation Analysis")
    print("(No CARLA required)")
    print("=" * 60)

    config = SimulationConfig()

    # Test speeds
    test_speeds = [20, 30, 35, 40, 50]  # km/h

    all_results = []

    for speed in test_speeds:
        print(f"\nSimulating at {speed} km/h...")
        results = run_simulation(speed, config, debug=False)
        analysis = analyze_results(results)
        all_results.append(results)

        print(f"  Mean speed: {analysis['mean_speed_kmh']:.2f} km/h")
        print(f"  Std dev: {analysis['std_speed_kmh']:.3f} km/h")
        print(f"  Max error: {analysis['max_error_kmh']:.3f} km/h")
        print(f"  Oscillation: {analysis['oscillation_freq_hz']:.2f} Hz")

        if analysis['stable']:
            print(f"  Result: STABLE")
        else:
            print(f"  Result: UNSTABLE - {', '.join(analysis['reasons'])}")

    # Generate plot
    plot_results(all_results, 'pid_analysis.png')

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    for results in all_results:
        analysis = analyze_results(results)
        speed = results['target_speed_kmh']
        status = "UNSTABLE" if not analysis['stable'] else "stable"
        print(f"  {speed:3.0f} km/h: {status:8s} | std={analysis['std_speed_kmh']:.3f} km/h | osc={analysis['oscillation_freq_hz']:.2f} Hz")


if __name__ == "__main__":
    main()
