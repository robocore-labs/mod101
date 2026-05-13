# Configurator

`configurator/` is a self-contained static page + Python backend for sizing and tooling the arm. It edits `src/mod101_description/urdf/mod101.xacro` in place and shows a live three.js view (`urdf-loader`) that re-renders on every save.

## Running

```bash
cd ~/Work/mod101
python3 configurator/server.py
# open http://localhost:8000/
```

Backend (`server.py`) is stdlib-only Python — no Flask, no virtualenv.

## What you can change

- **Extrusion lengths** — `shoulder_ext_length` and `elbow_ext_length` xacro properties. Reach, total mass, and payload (continuous + stall, at full extension and at 70 % reach) recompute live for both BASE and PRO servo configs.
- **Tool** — dropdown lists every `mod101_tool_*` package discovered under `src/`; selecting one rewrites the `tool` xacro arg's `default` and reloads the viewer with the new end-effector attached.

After changing anything, rebuild the affected packages (`mod101_description` for lengths, `mod101_tool_<name>` if you also added/edited a tool) to pick up the changes in sim.

You can also open `configurator/index.html` directly via `file://` for a read-only preview; Save needs the server.

## Endpoints

| Method + path | Purpose |
|---|---|
| `GET /load` / `POST /save` | shoulder + elbow extrusion lengths |
| `GET /tool` / `POST /tool` | active tool + discovered tools |
| `GET /urdf` | runs `xacro` and rewrites mesh URIs (`package://` and `file://`) to `/pkg/<pkgname>/meshes/<file>` |
| `GET /pkg/<pkgname>/meshes/<file>` | serves binary meshes from any package's `meshes/` dir |
