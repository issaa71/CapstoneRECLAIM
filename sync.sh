#!/usr/bin/env bash
# sync.sh — rsync wrapper for pushing local workspace to MIC-711
# Usage:
#   ./sync.sh                    # Sync entire workspace
#   ./sync.sh <package_name>     # Sync only a specific package
#   ./sync.sh --build <pkg>      # Sync + build a specific package
#   ./sync.sh --run <pkg> <launch>  # Sync + build + launch
set -euo pipefail

LOCAL_WS="$(cd "$(dirname "$0")" && pwd)/reclaim_ws"
REMOTE_WS="~/reclaim_ws"
REMOTE="mic"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[sync]${NC} $1"; }
warn()  { echo -e "${YELLOW}[sync]${NC} $1"; }
error() { echo -e "${RED}[sync]${NC} $1"; exit 1; }

# Check local workspace exists
if [ ! -d "$LOCAL_WS" ]; then
    error "Local workspace not found at $LOCAL_WS"
fi

# Check SSH connectivity (fast timeout)
if ! ssh -o ConnectTimeout=3 "$REMOTE" "true" 2>/dev/null; then
    error "Cannot reach MIC-711. Is it powered on and connected?"
fi

sync_workspace() {
    local pkg="${1:-}"
    if [ -n "$pkg" ]; then
        local pkg_path="$LOCAL_WS/src/$pkg"
        if [ ! -d "$pkg_path" ]; then
            error "Package '$pkg' not found at $pkg_path"
        fi
        info "Syncing package: $pkg"
        rsync -avz --delete \
            --exclude='build/' \
            --exclude='install/' \
            --exclude='log/' \
            --exclude='.git/' \
            --exclude='__pycache__/' \
            --exclude='*.pyc' \
            "$pkg_path/" "${REMOTE}:${REMOTE_WS}/src/${pkg}/"
    else
        info "Syncing entire workspace..."
        warn "NOTE: Using --delete. Files only on MIC-711 will be removed."
        warn "Excluding: URDF files, tests/, models/, maps/ (shared with others)"
        rsync -avz --delete \
            --exclude='build/' \
            --exclude='install/' \
            --exclude='log/' \
            --exclude='.git/' \
            --exclude='__pycache__/' \
            --exclude='*.pyc' \
            --exclude='*.urdf' \
            --exclude='**/urdf/' \
            --exclude='**/tests/' \
            --exclude='**/test/' \
            --exclude='**/models/' \
            --exclude='**/maps/' \
            "$LOCAL_WS/" "${REMOTE}:${REMOTE_WS}/"
    fi
    info "Sync complete."
}

build_package() {
    local pkg="$1"
    info "Building package: $pkg on MIC-711..."
    ssh "$REMOTE" "cd ${REMOTE_WS} && conda activate ros_env && colcon build --packages-select ${pkg} --symlink-install 2>&1"
    info "Build complete."
}

run_launch() {
    local pkg="$1"
    local launch="$2"
    info "Launching: ros2 launch $pkg $launch"
    ssh -t "$REMOTE" "cd ${REMOTE_WS} && conda activate ros_env && source install/setup.bash && ros2 launch ${pkg} ${launch}"
}

case "${1:-}" in
    --build)
        [ -z "${2:-}" ] && error "Usage: ./sync.sh --build <package_name>"
        sync_workspace "$2"
        build_package "$2"
        ;;
    --run)
        [ -z "${2:-}" ] && error "Usage: ./sync.sh --run <package_name> <launch_file>"
        [ -z "${3:-}" ] && error "Usage: ./sync.sh --run <package_name> <launch_file>"
        sync_workspace "$2"
        build_package "$2"
        run_launch "$2" "$3"
        ;;
    --help|-h)
        echo "Usage:"
        echo "  ./sync.sh                          Sync entire workspace"
        echo "  ./sync.sh <pkg>                    Sync specific package"
        echo "  ./sync.sh --build <pkg>            Sync + build package"
        echo "  ./sync.sh --run <pkg> <launch>     Sync + build + launch"
        echo "  ./sync.sh --help                   Show this help"
        ;;
    *)
        sync_workspace "${1:-}"
        ;;
esac
