#!/bin/bash

# Piper Inference Startup Script
# This script sets up ROS2 environment and starts the Piper inference system

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting Piper Inference System${NC}"

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found. Please install ROS2 first.${NC}"
    exit 1
fi

# Source ROS2 setup
echo -e "${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash

# Check if Piper ROS package is available
PIPER_ROS_PATH="/home/rosota/Github/Piper_Ros-Tcp"
if [ -d "$PIPER_ROS_PATH" ]; then
    echo -e "${YELLOW}Sourcing Piper ROS workspace...${NC}"
    if [ -f "$PIPER_ROS_PATH/install/setup.bash" ]; then
        source "$PIPER_ROS_PATH/install/setup.bash"
    else
        echo -e "${YELLOW}Warning: Piper ROS workspace not built. Building now...${NC}"
        cd "$PIPER_ROS_PATH"
        colcon build
        source install/setup.bash
        cd - > /dev/null
    fi
else
    echo -e "${YELLOW}Warning: Piper ROS package not found at $PIPER_ROS_PATH${NC}"
fi

# Default parameters
POLICY_PATH="outputs/train/act_piper_xr_test/checkpoints/100000/pretrained_model"
DATASET_REPO_ID="ujin/piper-direct-mode-test_xr_2"
EPISODE_TIME=30
FPS=30
VERBOSE=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --policy_path)
            POLICY_PATH="$2"
            shift 2
            ;;
        --dataset_repo_id)
            DATASET_REPO_ID="$2"
            shift 2
            ;;
        --episode_time_s)
            EPISODE_TIME="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --quiet)
            VERBOSE=false
            shift
            ;;
        --start-piper-node)
            START_PIPER_NODE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --policy_path PATH          Path to trained policy model"
            echo "  --dataset_repo_id ID        Dataset repo ID for features"
            echo "  --episode_time_s TIME       Inference time in seconds (default: 30)"
            echo "  --fps FPS                   Control frequency (default: 30)"
            echo "  --quiet                     Reduce output verbosity"
            echo "  --start-piper-node          Also start Piper ROS node"
            echo "  -h, --help                  Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    if [ ! -z "$PIPER_NODE_PID" ]; then
        echo "Killing Piper node (PID: $PIPER_NODE_PID)"
        kill $PIPER_NODE_PID 2>/dev/null || true
    fi
    if [ ! -z "$INFERENCE_PID" ]; then
        echo "Killing inference process (PID: $INFERENCE_PID)"
        kill $INFERENCE_PID 2>/dev/null || true
    fi
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start Piper ROS node if requested
if [ "$START_PIPER_NODE" = true ]; then
    echo -e "${YELLOW}Starting Piper ROS node...${NC}"
    ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=true gripper_exist:=true &
    PIPER_NODE_PID=$!
    echo "Piper node started with PID: $PIPER_NODE_PID"
    sleep 5  # Give the node more time to start
fi

# Check if policy path exists
if [ ! -d "$POLICY_PATH" ]; then
    echo -e "${RED}Error: Policy path not found: $POLICY_PATH${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}Configuration:${NC}"
echo "  Policy Path: $POLICY_PATH"
echo "  Dataset Repo ID: $DATASET_REPO_ID"
echo "  Episode Time: ${EPISODE_TIME}s"
echo "  FPS: $FPS"
echo "  Verbose: $VERBOSE"

# Start the inference
echo -e "${GREEN}Starting Piper inference...${NC}"
python -m lerobot.infer_piper \
    --policy_path="$POLICY_PATH" \
    --dataset_repo_id="$DATASET_REPO_ID" \
    --episode_time_s="$EPISODE_TIME" \
    --fps="$FPS" \
    --verbose="$VERBOSE" &

INFERENCE_PID=$!
echo "Inference started with PID: $INFERENCE_PID"

# Wait for inference to complete
wait $INFERENCE_PID

echo -e "${GREEN}Inference completed successfully!${NC}"
cleanup 