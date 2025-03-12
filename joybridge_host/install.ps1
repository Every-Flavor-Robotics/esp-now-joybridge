# Requires PowerShell 7+

$ErrorActionPreference = "Stop"  # Exit on error

$PLUGIN_REPO = "https://github.com/Every-Flavor-Robotics/esp-now-joybridge.git"
$PLUGIN_SUBDIR = "joybridge_host"  # The subfolder with setup.py or pyproject.toml
$TMP_DIR = New-Item -ItemType Directory -Path (Join-Path $env:TEMP (New-Guid))

Write-Host "==> Installing joybridge plugin from $PLUGIN_REPO/$PLUGIN_SUBDIR"

# 1) Navigate to the temporary directory
Set-Location $TMP_DIR

# 2) Initialize an empty git repository
git init

# 3) Add the remote repository
git remote add origin $PLUGIN_REPO

# 4) Enable sparse checkout
git config core.sparseCheckout true

# 5) For Git >= 2.25, use sparse-checkout set; otherwise, manually configure sparse-checkout
Add-Content -Path .git/info/sparse-checkout -Value "$PLUGIN_SUBDIR/*"

# 6) Pull only the specified subfolder (shallow clone, depth 1 to reduce data)
git pull origin main --depth=1

# 7) Enter the plugin subdirectory
Set-Location "$TMP_DIR\$PLUGIN_SUBDIR"

# 8) Install the plugin (non-editable mode, upgrade pip if needed)
python -m pip install --upgrade pip
python -m pip install .

Write-Host "==> Plugin joybridge successfully installed!"

# 9) Cleanup: Remove the temporary directory
Set-Location $env:TEMP
Remove-Item -Recurse -Force $TMP_DIR

Write-Host "==> Cleanup complete."