#!/usr/bin/env bash
set -euo pipefail

LEROBOT_TAG="v0.6.0"
LEROBOT_COMMIT_PREFIX="30da8e6"
LEROBOT_DIR="${LEROBOT_DIR:-$HOME/lerobot-v060}"
CONDA_ENV="${CONDA_ENV:-lerobot-v060}"

echo "[1/6] Checking Jetson/Linux environment"
uname -a
if [[ "$(uname -s)" != "Linux" ]]; then
  echo "ERROR: this setup script must run on the Jetson Linux host." >&2
  exit 1
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "WARNING: expected Jetson aarch64, found $(uname -m)."
fi

echo "[2/6] Checking conda"
if ! command -v conda >/dev/null 2>&1; then
  echo "ERROR: conda/miniforge is required. Install it first, then rerun." >&2
  exit 1
fi
eval "$(conda shell.bash hook)"

echo "[3/6] Creating isolated Python 3.12 environment"
if ! conda env list | awk '{print $1}' | grep -qx "$CONDA_ENV"; then
  conda create -n "$CONDA_ENV" python=3.12 -y
fi
conda activate "$CONDA_ENV"
python -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version'

echo "[4/6] Fetching pinned LeRobot $LEROBOT_TAG"
if [[ ! -d "$LEROBOT_DIR/.git" ]]; then
  git clone --branch "$LEROBOT_TAG" --depth 1 \
    https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
fi
git -C "$LEROBOT_DIR" fetch --depth 1 origin "refs/tags/$LEROBOT_TAG:refs/tags/$LEROBOT_TAG"
git -C "$LEROBOT_DIR" checkout --detach "$LEROBOT_TAG"
actual_commit="$(git -C "$LEROBOT_DIR" rev-parse HEAD)"
if [[ "$actual_commit" != "$LEROBOT_COMMIT_PREFIX"* ]]; then
  echo "ERROR: $LEROBOT_TAG resolved to unexpected commit $actual_commit" >&2
  exit 1
fi

echo "[5/6] Installing only the LeKiwi extra"
python -m pip install --upgrade pip
python -m pip install -e "$LEROBOT_DIR[lekiwi]"

echo "[6/6] Recording the reproducible environment"
python -m pip freeze > "$LEROBOT_DIR/requirements-fcr-phase1.txt"
printf '%s\n' "$actual_commit" > "$LEROBOT_DIR/fcr-phase1-commit.txt"

echo
echo "READY"
echo "  conda env: $CONDA_ENV"
echo "  LeRobot:   $LEROBOT_DIR ($actual_commit)"
echo "Next: connect and power the Feetech board, then run lerobot-find-port."
