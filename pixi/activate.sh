# Sourced by `pixi shell` / `pixi run` (see [activation] in pixi.toml).
# Overlays the colcon workspace once it has been built. Safe to run before the
# first `pixi run build` - it just does nothing.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.sh" ]; then
  source "$PIXI_PROJECT_ROOT/install/setup.sh"
fi
