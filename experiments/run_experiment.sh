#!/bin/bash
#
# Speed Tracking Instability Experiment Setup and Runner
#
# This script:
# 1. Downloads CARLA 0.9.15 if not present
# 2. Installs Python dependencies
# 3. Starts CARLA in headless mode
# 4. Runs the speed tracking experiment
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_VERSION="0.9.15"
CARLA_DIR="${SCRIPT_DIR}/carla_${CARLA_VERSION}"
CARLA_DOWNLOAD_URL="https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_${CARLA_VERSION}.tar.gz"

echo "=============================================="
echo "Speed Tracking Instability Experiment"
echo "=============================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [ ! -z "$CARLA_PID" ]; then
        echo "Stopping CARLA server (PID: $CARLA_PID)..."
        kill $CARLA_PID 2>/dev/null || true
        wait $CARLA_PID 2>/dev/null || true
    fi
    echo "Done."
}
trap cleanup EXIT

# Step 1: Install Python dependencies
echo "[1/4] Installing Python dependencies..."
pip3 install --quiet carla==0.9.15 simple-pid numpy 2>/dev/null || {
    echo "Installing dependencies with pip..."
    pip3 install carla==0.9.15 simple-pid numpy
}
echo "  Dependencies installed."

# Step 2: Download CARLA if needed
echo ""
echo "[2/4] Checking CARLA installation..."
if [ ! -d "$CARLA_DIR" ]; then
    echo "  CARLA ${CARLA_VERSION} not found. Downloading..."
    echo "  URL: ${CARLA_DOWNLOAD_URL}"
    echo "  This may take a while (~15GB)..."

    mkdir -p "$CARLA_DIR"
    cd "$CARLA_DIR"

    # Download with progress
    wget -q --show-progress "$CARLA_DOWNLOAD_URL" -O carla.tar.gz || {
        echo "  Trying alternative download with curl..."
        curl -L -# "$CARLA_DOWNLOAD_URL" -o carla.tar.gz
    }

    echo "  Extracting..."
    tar -xzf carla.tar.gz
    rm carla.tar.gz

    echo "  CARLA downloaded and extracted to: $CARLA_DIR"
else
    echo "  CARLA ${CARLA_VERSION} found at: $CARLA_DIR"
fi

# Step 3: Start CARLA in headless mode
echo ""
echo "[3/4] Starting CARLA server in headless mode..."
cd "$CARLA_DIR"

# Check for required display-less execution
export SDL_VIDEODRIVER=offscreen

# Start CARLA with headless rendering
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000 -quality-level=Low &
CARLA_PID=$!

echo "  CARLA starting (PID: $CARLA_PID)..."
echo "  Waiting for server to be ready..."

# Wait for CARLA to be ready (check if port is open)
MAX_WAIT=120
WAIT_TIME=0
while ! python3 -c "import carla; carla.Client('localhost', 2000).set_timeout(1.0); carla.Client('localhost', 2000).get_server_version()" 2>/dev/null; do
    sleep 2
    WAIT_TIME=$((WAIT_TIME + 2))
    if [ $WAIT_TIME -ge $MAX_WAIT ]; then
        echo "  ERROR: CARLA server failed to start within ${MAX_WAIT}s"
        exit 1
    fi
    echo "  Waiting... (${WAIT_TIME}s / ${MAX_WAIT}s)"
done

echo "  CARLA server is ready!"

# Step 4: Run the experiment
echo ""
echo "[4/4] Running speed tracking experiment..."
echo ""
cd "$SCRIPT_DIR"
python3 speed_tracking_test.py --duration 30

echo ""
echo "=============================================="
echo "Experiment completed successfully!"
echo "=============================================="
