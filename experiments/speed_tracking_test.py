#!/usr/bin/env python3
"""
Minimal experiment to validate speed tracking instability in carla_ackermann_control
at speeds around 30-40 km/h (8.3-11.1 m/s).

This script replicates the PID control logic from carla_ackermann_control_node.py
and tests it against a CARLA vehicle to observe speed tracking behavior.

Usage:
    1. Start CARLA server in headless mode:
       ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000

    2. Run this experiment:
       python3 speed_tracking_test.py
"""

import sys
import time
import math
import csv
import argparse
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import carla
except ImportError:
    print("ERROR: carla package not found.")
    print("Install with: pip install carla==0.9.15")
    print("Or set PYTHONPATH to include CARLA's PythonAPI egg/wheel")
    sys.exit(1)

try:
    from simple_pid import PID
except ImportError:
    print("ERROR: simple_pid package not found.")
    print("Install with: pip install simple-pid")
    sys.exit(1)

import numpy as np


# -----------------------------------------------------------------------------
# Physics calculations (from carla_control_physics.py)
# -----------------------------------------------------------------------------
def get_vehicle_driving_impedance_acceleration(vehicle_info, vehicle_status, reverse):
    """
    Calculate the acceleration required to overcome driving impedance.
    (rolling resistance + aerodynamic drag + slope)
    """
    dominated_force = 0.0
    dominated_force += get_rolling_resistance_force(vehicle_info)
    dominated_force += get_aerodynamic_drag_force(vehicle_status)
    dominated_force += get_slope_force(vehicle_info, vehicle_status)

    mass = get_vehicle_mass(vehicle_info)
    if mass == 0:
        return 0.0

    acceleration = dominated_force / mass
    if reverse:
        acceleration = -acceleration
    return acceleration


def get_vehicle_lay_off_engine_acceleration(vehicle_info):
    """
    Calculate the deceleration from engine braking when throttle is released.
    """
    mass = get_vehicle_mass(vehicle_info)
    if mass == 0:
        return 0.0
    # Approximate engine brake force
    engine_brake_force = 500.0  # N
    return engine_brake_force / mass


def get_rolling_resistance_force(vehicle_info):
    """Rolling resistance force"""
    rolling_resistance_coefficient = 0.01
    normal_force = get_vehicle_mass(vehicle_info) * 9.81
    return rolling_resistance_coefficient * normal_force


def get_aerodynamic_drag_force(vehicle_status):
    """Aerodynamic drag force"""
    drag_coefficient = 0.3
    frontal_area = 2.37  # m^2
    air_density = 1.225  # kg/m^3
    velocity = vehicle_status.velocity if hasattr(vehicle_status, 'velocity') else 0.0
    return 0.5 * drag_coefficient * frontal_area * air_density * velocity * velocity


def get_slope_force(vehicle_info, vehicle_status):
    """Force due to road slope (simplified - assumes flat road)"""
    return 0.0  # Simplified for this experiment


def get_vehicle_mass(vehicle_info):
    """Get vehicle mass"""
    if hasattr(vehicle_info, 'mass') and vehicle_info.mass > 0:
        return vehicle_info.mass
    return 1500.0  # Default mass in kg


def get_vehicle_max_steering_angle(vehicle_info):
    """Get max steering angle in radians"""
    return 1.22  # ~70 degrees default


def get_vehicle_max_speed():
    """Get max speed in m/s"""
    return 50.0  # 180 km/h


def get_vehicle_max_acceleration():
    """Get max acceleration in m/s^2"""
    return 3.0


def get_vehicle_max_deceleration():
    """Get max deceleration in m/s^2"""
    return 8.0


# -----------------------------------------------------------------------------
# Data classes for tracking state
# -----------------------------------------------------------------------------
@dataclass
class VehicleInfo:
    mass: float = 1500.0
    max_steering_angle: float = 1.22


@dataclass
class VehicleStatus:
    velocity: float = 0.0
    orientation_pitch: float = 0.0


@dataclass
class ControlRestrictions:
    max_steering_angle: float = 1.22
    max_speed: float = 50.0
    max_accel: float = 3.0
    max_decel: float = 8.0
    min_accel: float = 1.0
    max_pedal: float = 3.0


@dataclass
class ControlTarget:
    steering_angle: float = 0.0
    speed: float = 0.0
    speed_abs: float = 0.0
    accel: float = 0.0


@dataclass
class ControlCurrent:
    time_sec: float = 0.0
    speed: float = 0.0
    speed_abs: float = 0.0
    accel: float = 0.0


@dataclass
class ControlStatus:
    status: str = "n/a"
    speed_control_activation_count: int = 0
    speed_control_accel_delta: float = 0.0
    speed_control_accel_target: float = 0.0
    accel_control_pedal_delta: float = 0.0
    accel_control_pedal_target: float = 0.0
    brake_upper_border: float = 0.0
    throttle_lower_border: float = 0.0


@dataclass
class ControlOutput:
    throttle: float = 0.0
    brake: float = 1.0
    steer: float = 0.0
    reverse: bool = False
    hand_brake: bool = True


@dataclass
class DataPoint:
    """Single data point for logging"""
    timestamp: float
    target_speed: float
    actual_speed: float
    speed_error: float
    throttle: float
    brake: float
    accel: float
    status: str


# -----------------------------------------------------------------------------
# Ackermann Controller (simplified from carla_ackermann_control_node.py)
# -----------------------------------------------------------------------------
class AckermannController:
    """
    Simplified Ackermann controller replicating the logic from
    carla_ackermann_control_node.py
    """

    def __init__(self, speed_kp=0.05, speed_ki=0.0, speed_kd=0.5,
                 accel_kp=0.05, accel_ki=0.0, accel_kd=0.05,
                 control_loop_rate=0.05):

        self.control_loop_rate = control_loop_rate

        # PID controllers (matching default parameters)
        self.speed_controller = PID(
            Kp=speed_kp, Ki=speed_ki, Kd=speed_kd,
            sample_time=control_loop_rate,
            output_limits=(-1.0, 1.0)
        )

        self.accel_controller = PID(
            Kp=accel_kp, Ki=accel_ki, Kd=accel_kd,
            sample_time=control_loop_rate,
            output_limits=(-1.0, 1.0)
        )

        # State
        self.vehicle_info = VehicleInfo()
        self.vehicle_status = VehicleStatus()
        self.restrictions = ControlRestrictions()
        self.target = ControlTarget()
        self.current = ControlCurrent()
        self.status = ControlStatus()
        self.output = ControlOutput()

        self.current.time_sec = time.time()

    def set_target_speed(self, target_speed: float):
        """Set target speed in m/s"""
        self.target.speed = np.clip(target_speed, -self.restrictions.max_speed,
                                    self.restrictions.max_speed)
        self.target.speed_abs = abs(self.target.speed)

        # When setting speed, use low acceleration (triggers speed controller)
        self.target.accel = 0.0

    def update_vehicle_state(self, velocity: float):
        """Update current vehicle state from CARLA"""
        current_time = time.time()
        delta_time = current_time - self.current.time_sec

        if delta_time > 0:
            delta_speed = velocity - self.current.speed
            current_accel = delta_speed / delta_time
            # Average filter (matching original code)
            self.current.accel = (self.current.accel * 4 + current_accel) / 5

        self.current.time_sec = current_time
        self.current.speed = velocity
        self.current.speed_abs = abs(velocity)
        self.vehicle_status.velocity = velocity

    def run_speed_control_loop(self):
        """Run speed PID controller"""
        epsilon = 0.00001
        target_accel_abs = abs(self.target.accel)

        # Activation logic (from original)
        if target_accel_abs < self.restrictions.min_accel:
            if self.status.speed_control_activation_count < 5:
                self.status.speed_control_activation_count += 1
        else:
            if self.status.speed_control_activation_count > 0:
                self.status.speed_control_activation_count -= 1

        self.speed_controller.auto_mode = self.status.speed_control_activation_count >= 5

        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.target.speed_abs
            self.status.speed_control_accel_delta = float(
                self.speed_controller(self.current.speed_abs)
            )

            # Clipping
            clipping_lower = -target_accel_abs if target_accel_abs >= epsilon else -self.restrictions.max_decel
            clipping_upper = target_accel_abs if target_accel_abs >= epsilon else self.restrictions.max_accel

            self.status.speed_control_accel_target = np.clip(
                self.status.speed_control_accel_target + self.status.speed_control_accel_delta,
                clipping_lower, clipping_upper
            )
        else:
            self.status.speed_control_accel_delta = 0.0
            self.status.speed_control_accel_target = self.target.accel

    def run_accel_control_loop(self):
        """Run acceleration PID controller"""
        self.accel_controller.setpoint = self.status.speed_control_accel_target
        self.status.accel_control_pedal_delta = float(
            self.accel_controller(self.current.accel)
        )
        self.status.accel_control_pedal_target = np.clip(
            self.status.accel_control_pedal_target + self.status.accel_control_pedal_delta,
            -self.restrictions.max_pedal, self.restrictions.max_pedal
        )

    def update_drive_command(self):
        """Convert pedal target to throttle/brake commands"""
        # Driving impedance border
        self.status.throttle_lower_border = get_vehicle_driving_impedance_acceleration(
            self.vehicle_info, self.vehicle_status, self.output.reverse
        )

        # Engine braking border
        self.status.brake_upper_border = (
            self.status.throttle_lower_border +
            get_vehicle_lay_off_engine_acceleration(self.vehicle_info)
        )

        pedal = self.status.accel_control_pedal_target

        if pedal > self.status.throttle_lower_border:
            self.status.status = "accelerating"
            self.output.brake = 0.0
            self.output.throttle = (pedal - self.status.throttle_lower_border) / self.restrictions.max_pedal
        elif pedal > self.status.brake_upper_border:
            self.status.status = "coasting"
            self.output.brake = 0.0
            self.output.throttle = 0.0
        else:
            self.status.status = "braking"
            self.output.throttle = 0.0
            self.output.brake = (self.status.brake_upper_border - pedal) / self.restrictions.max_pedal

        # Clip final outputs
        self.output.throttle = np.clip(self.output.throttle, 0.0, 1.0)
        self.output.brake = np.clip(self.output.brake, 0.0, 1.0)

    def control_stop_and_reverse(self):
        """Handle stopping and reverse"""
        standing_still_epsilon = 0.1
        full_stop_epsilon = 0.00001

        self.output.hand_brake = False

        if self.current.speed_abs < standing_still_epsilon:
            self.status.status = "standing"
            if self.target.speed_abs < full_stop_epsilon:
                self.status.status = "full stop"
                self.output.hand_brake = True
                self.output.brake = 1.0
                self.output.throttle = 0.0

    def compute_control(self, velocity: float) -> carla.VehicleControl:
        """
        Main control computation - call this each control cycle.
        Returns CARLA VehicleControl command.
        """
        self.update_vehicle_state(velocity)
        self.control_stop_and_reverse()

        if not self.output.hand_brake:
            self.run_speed_control_loop()
            self.run_accel_control_loop()
            self.update_drive_command()

        control = carla.VehicleControl()
        control.throttle = self.output.throttle
        control.brake = self.output.brake
        control.steer = self.output.steer
        control.reverse = self.output.reverse
        control.hand_brake = self.output.hand_brake

        return control


# -----------------------------------------------------------------------------
# Experiment Runner
# -----------------------------------------------------------------------------
class SpeedTrackingExperiment:
    """
    Runs the speed tracking experiment in CARLA.
    """

    def __init__(self, host='localhost', port=2000):
        self.host = host
        self.port = port
        self.client = None
        self.world = None
        self.vehicle = None
        self.controller = None
        self.data_log: List[DataPoint] = []

    def connect(self):
        """Connect to CARLA server"""
        print(f"Connecting to CARLA at {self.host}:{self.port}...")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        print(f"Connected! Map: {self.world.get_map().name}")

    def setup_synchronous_mode(self, delta_seconds=0.05):
        """Enable synchronous mode for deterministic simulation"""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = delta_seconds
        self.world.apply_settings(settings)
        print(f"Synchronous mode enabled (dt={delta_seconds}s)")

    def spawn_vehicle(self):
        """Spawn ego vehicle on a straight road section"""
        blueprint_library = self.world.get_blueprint_library()

        # Use a standard sedan
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

        # Find a good spawn point (preferably on a straight road)
        spawn_points = self.world.get_map().get_spawn_points()

        # Try to find a spawn point that's on a straight section
        spawn_point = spawn_points[0]  # Default

        for sp in spawn_points:
            # Check if there's a long straight ahead
            waypoint = self.world.get_map().get_waypoint(sp.location)
            next_wps = waypoint.next(100.0)  # Check 100m ahead
            if next_wps:
                spawn_point = sp
                break

        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle: {self.vehicle.type_id}")

        # Get vehicle physics for mass info
        physics = self.vehicle.get_physics_control()
        print(f"Vehicle mass: {physics.mass} kg")

        return physics.mass

    def get_vehicle_speed(self) -> float:
        """Get current vehicle speed in m/s"""
        velocity = self.vehicle.get_velocity()
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def run_test(self, target_speed_kmh: float, duration_sec: float = 30.0,
                 control_rate: float = 0.05):
        """
        Run speed tracking test at specified target speed.

        Args:
            target_speed_kmh: Target speed in km/h
            duration_sec: Test duration in seconds
            control_rate: Control loop rate in seconds
        """
        target_speed_ms = target_speed_kmh / 3.6
        print(f"\n{'='*60}")
        print(f"Running test: Target speed = {target_speed_kmh} km/h ({target_speed_ms:.2f} m/s)")
        print(f"Duration: {duration_sec}s, Control rate: {control_rate}s")
        print(f"{'='*60}")

        # Initialize controller with default PID parameters
        self.controller = AckermannController(control_loop_rate=control_rate)

        # Update controller with actual vehicle mass
        physics = self.vehicle.get_physics_control()
        self.controller.vehicle_info.mass = physics.mass

        # Set target speed
        self.controller.set_target_speed(target_speed_ms)

        # Reset data log for this test
        test_data = []
        start_time = time.time()

        # Run control loop
        num_steps = int(duration_sec / control_rate)

        for step in range(num_steps):
            # Get current speed
            current_speed = self.get_vehicle_speed()

            # Compute control
            control = self.controller.compute_control(current_speed)

            # Apply control
            self.vehicle.apply_control(control)

            # Tick simulation
            self.world.tick()

            # Log data
            elapsed = time.time() - start_time
            speed_error = target_speed_ms - current_speed

            data_point = DataPoint(
                timestamp=elapsed,
                target_speed=target_speed_ms,
                actual_speed=current_speed,
                speed_error=speed_error,
                throttle=control.throttle,
                brake=control.brake,
                accel=self.controller.current.accel,
                status=self.controller.status.status
            )
            test_data.append(data_point)

            # Progress output every second
            if step % int(1.0 / control_rate) == 0:
                print(f"  t={elapsed:5.1f}s | target={target_speed_ms*3.6:5.1f} km/h | "
                      f"actual={current_speed*3.6:5.1f} km/h | error={speed_error*3.6:+5.1f} km/h | "
                      f"thr={control.throttle:.2f} brk={control.brake:.2f} | {self.controller.status.status}")

        # Add to main log
        self.data_log.extend(test_data)

        # Analyze results for this test
        self.analyze_test(test_data, target_speed_kmh)

        return test_data

    def analyze_test(self, test_data: List[DataPoint], target_speed_kmh: float):
        """Analyze test results for instability indicators"""
        # Skip first 5 seconds (acceleration phase)
        steady_state_data = [d for d in test_data if d.timestamp > 5.0]

        if not steady_state_data:
            print("  Not enough data for analysis")
            return

        speeds = [d.actual_speed for d in steady_state_data]
        errors = [d.speed_error for d in steady_state_data]

        mean_speed = np.mean(speeds)
        std_speed = np.std(speeds)
        max_error = max(abs(e) for e in errors)
        mean_error = np.mean(errors)

        # Calculate oscillation metrics
        # Count zero crossings in error (indicator of oscillation)
        zero_crossings = 0
        for i in range(1, len(errors)):
            if errors[i-1] * errors[i] < 0:
                zero_crossings += 1

        oscillation_freq = zero_crossings / (steady_state_data[-1].timestamp - steady_state_data[0].timestamp)

        print(f"\n  Analysis (steady-state, t > 5s):")
        print(f"    Mean speed: {mean_speed*3.6:.2f} km/h")
        print(f"    Speed std dev: {std_speed*3.6:.3f} km/h")
        print(f"    Mean error: {mean_error*3.6:+.3f} km/h")
        print(f"    Max |error|: {max_error*3.6:.3f} km/h")
        print(f"    Oscillation freq: {oscillation_freq:.2f} Hz")

        # Instability indicators
        is_unstable = False
        if std_speed * 3.6 > 1.0:  # More than 1 km/h std dev
            print(f"    WARNING: High speed variance detected!")
            is_unstable = True
        if oscillation_freq > 1.0:  # More than 1 Hz oscillation
            print(f"    WARNING: High frequency oscillations detected!")
            is_unstable = True
        if max_error * 3.6 > 3.0:  # More than 3 km/h max error
            print(f"    WARNING: Large tracking error detected!")
            is_unstable = True

        if is_unstable:
            print(f"    >>> INSTABILITY CONFIRMED at {target_speed_kmh} km/h <<<")
        else:
            print(f"    Speed tracking appears stable at {target_speed_kmh} km/h")

    def save_results(self, filename: str):
        """Save logged data to CSV file"""
        if not self.data_log:
            print("No data to save")
            return

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'target_speed_ms', 'actual_speed_ms',
                           'speed_error_ms', 'throttle', 'brake', 'accel', 'status'])
            for d in self.data_log:
                writer.writerow([d.timestamp, d.target_speed, d.actual_speed,
                               d.speed_error, d.throttle, d.brake, d.accel, d.status])

        print(f"\nResults saved to: {filename}")

    def cleanup(self):
        """Clean up CARLA resources"""
        if self.vehicle:
            self.vehicle.destroy()
            print("Vehicle destroyed")

        # Restore async mode
        if self.world:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)


def main():
    parser = argparse.ArgumentParser(description='Speed tracking instability experiment')
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--duration', type=float, default=30.0, help='Test duration per speed (seconds)')
    args = parser.parse_args()

    experiment = SpeedTrackingExperiment(host=args.host, port=args.port)

    try:
        experiment.connect()
        experiment.setup_synchronous_mode(delta_seconds=0.05)
        experiment.spawn_vehicle()

        # Test speeds around the reported instability range (30-40 km/h)
        # Also test outside this range for comparison
        test_speeds = [20, 30, 35, 40, 50]  # km/h

        for speed in test_speeds:
            # Reset vehicle position for each test
            spawn_points = experiment.world.get_map().get_spawn_points()
            experiment.vehicle.set_transform(spawn_points[0])
            experiment.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

            # Wait for vehicle to settle
            for _ in range(20):
                experiment.world.tick()

            experiment.run_test(target_speed_kmh=speed, duration_sec=args.duration)

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment.save_results(f"speed_tracking_results_{timestamp}.csv")

        print("\n" + "="*60)
        print("EXPERIMENT COMPLETE")
        print("="*60)

    except KeyboardInterrupt:
        print("\nExperiment interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        raise
    finally:
        experiment.cleanup()


if __name__ == "__main__":
    main()
