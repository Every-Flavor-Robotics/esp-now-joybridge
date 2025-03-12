#!/usr/bin/env bash
set -e  # Exit on error

PLUGIN_REPO="https://github.com/Every-Flavor-Robotics/esp-now-joybridge.git"
PLUGIN_SUBDIR="joybridge_host"  # the subfolder with setup.py
TMP_DIR="$(mktemp -d)"

echo "==> Installing JoyBridge plugin from ${PLUGIN_REPO}/${PLUGIN_SUBDIR}"

# 1) Create and enter a temporary directory
cd "${TMP_DIR}"

# 2) Initialize an empty git repo
git init

# 3) Add the remote
git remote add origin "${PLUGIN_REPO}"

# 4) Enable sparse checkout
git config core.sparseCheckout true

# 5) For newer versions of Git (>= 2.25), you can do:
#    git sparse-checkout set "${PLUGIN_SUBDIR}"
#    Otherwise, manually create the .git/info/sparse-checkout file:
echo "${PLUGIN_SUBDIR}/*" >> .git/info/sparse-checkout

# 6) Pull only the specified subfolder (shallow clone, depth 1 to reduce data)
git pull origin main --depth=1

# 7) Enter the plugin subdir
cd "${PLUGIN_SUBDIR}"

# 8) Install the plugin (non-editable mode, upgrade pip if desired)
uv pip install --upgrade pip
uv pip install .

echo "==> Plugin efr-test successfully installed!"

# 9) Cleanup: remove the temporary directory
cd
rm -rf "${TMP_DIR}"

# 10) Optionally remove this installer script if you like
# rm -- "$0"

echo "==> Cleanup complete."