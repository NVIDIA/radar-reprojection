# Sourceable env setup for the radar_reprojection Bazel build.
#
# Usage:
#   source scripts/env.sh
#   bazel build //...
#   bazel test  //...
#
# Sets:
#   PATH         — prepends ~/.local/bin so bazelisk is on PATH
#   CONDA_PREFIX — points at the radar_reprojection conda env (Eigen headers for
#                  the algorithm build; the env is also used by
#                  scripts/download_sample_clip.py for the physical_ai_av PyPI pkg).

export PATH="$HOME/.local/bin:$PATH"

if command -v conda >/dev/null 2>&1; then
    export CONDA_PREFIX="$(conda info --base)/envs/radar_reprojection"
elif [ -d "$HOME/miniconda3/envs/radar_reprojection" ]; then
    export CONDA_PREFIX="$HOME/miniconda3/envs/radar_reprojection"
elif [ -d "$HOME/anaconda3/envs/radar_reprojection" ]; then
    export CONDA_PREFIX="$HOME/anaconda3/envs/radar_reprojection"
else
    echo "warning: radar_reprojection conda env not found; run:" >&2
    echo "  conda create -n radar_reprojection python=3.11 -y" >&2
    echo "  conda activate radar_reprojection" >&2
    echo "  pip install physical_ai_av" >&2
    echo "  conda install -c conda-forge eigen=3.4 -y" >&2
fi

if [ ! -d "$CONDA_PREFIX/include/eigen3" ]; then
    echo "warning: eigen3 headers not found at \$CONDA_PREFIX. Make sure 'conda install -c conda-forge eigen=3.4' was run in the env." >&2
fi
