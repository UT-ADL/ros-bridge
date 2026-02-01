# Speed Tracking Instability Experiment

This directory contains experiments to validate and analyze speed tracking instability
in the `carla_ackermann_control` node at speeds around 30-40 km/h.

## Files

| File | Description |
|------|-------------|
| `speed_tracking_test.py` | Full CARLA experiment - tests speed tracking at various speeds |
| `pid_simulation_test.py` | Lightweight simulation - analyzes PID behavior without CARLA |
| `run_experiment.sh` | Automated setup script - downloads CARLA and runs experiment |
| `pid_analysis.png` | Output plot from PID simulation |

## Quick Start

### Option 1: Lightweight Simulation (No CARLA Required)

Test the PID controller behavior in a simplified vehicle model:

```bash
pip install simple-pid numpy matplotlib
python3 pid_simulation_test.py
```

This generates `pid_analysis.png` showing speed tracking at 20, 30, 35, 40, 50 km/h.

### Option 2: Full CARLA Experiment

#### Prerequisites
- 20GB free disk space (for CARLA download)
- NVIDIA GPU with Vulkan support (required even for headless mode)
- NVIDIA drivers installed (`nvidia-smi` should work)
- Linux x86_64 system

#### Automated Setup
```bash
./run_experiment.sh
```

This script will:
1. Install Python dependencies (carla, simple-pid, numpy)
2. Download CARLA 0.9.15 (~15GB) if not present
3. Start CARLA in headless mode
4. Run speed tracking tests at 20, 30, 35, 40, 50 km/h
5. Generate results CSV file

#### Manual Setup

1. **Install CARLA 0.9.15:**
   ```bash
   wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.15.tar.gz
   tar -xzf CARLA_0.9.15.tar.gz -C ./carla_0.9.15
   ```

2. **Start CARLA Server (headless):**
   ```bash
   cd carla_0.9.15
   ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000 -quality-level=Low
   ```

3. **Run Experiment (in another terminal):**
   ```bash
   pip install carla==0.9.15 simple-pid numpy
   python3 speed_tracking_test.py --duration 30
   ```

## Test Parameters

The experiment tests with default PID parameters from `carla_ackermann_control`:

| Parameter | Value |
|-----------|-------|
| speed_Kp | 0.05 |
| speed_Ki | 0.0 |
| speed_Kd | 0.5 |
| accel_Kp | 0.05 |
| accel_Ki | 0.0 |
| accel_Kd | 0.05 |
| min_accel | 1.0 m/s² |
| control_loop_rate | 0.05s (20 Hz) |

## Instability Indicators

The analysis flags instability when:
- Speed standard deviation > 1.0 km/h
- Oscillation frequency > 1.0 Hz
- Maximum tracking error > 3.0 km/h

## Expected Output

### PID Simulation Results (Idealized Model)
```
   20 km/h: stable   | std=0.254 km/h | osc=0.00 Hz
   30 km/h: stable   | std=0.383 km/h | osc=0.00 Hz
   35 km/h: stable   | std=0.450 km/h | osc=0.00 Hz
   40 km/h: stable   | std=0.518 km/h | osc=0.00 Hz
   50 km/h: stable   | std=0.656 km/h | osc=0.00 Hz
```

Note: The simplified simulation shows stability, suggesting any real instability
comes from CARLA-specific vehicle dynamics, not the PID parameters themselves.

### CARLA Test Output
The experiment outputs:
1. Real-time progress showing target vs actual speed
2. Analysis of each speed test (variance, oscillation frequency, max error)
3. CSV file with complete time series data

## Interpreting Results

If instability is observed at 30-40 km/h in CARLA but not in simulation,
potential causes include:

1. **Vehicle-specific dynamics** - CARLA's physics model has non-linear torque curves
2. **Simulation timing** - Fixed vs variable timesteps can cause control issues
3. **Sensor delays** - Latency in velocity feedback affects PID response
4. **Driving impedance model** - The aerodynamic drag calculation may be inaccurate

## Tuning Recommendations

If instability is confirmed, consider:

1. **Reduce derivative gains** - High Kd amplifies sensor noise
2. **Add integral gain** - Helps with steady-state error
3. **Increase min_accel threshold** - Delays speed controller activation
4. **Tune per vehicle type** - Different vehicles have different dynamics
