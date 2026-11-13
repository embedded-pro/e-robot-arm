# tools

Host-side developer tools — CLI utilities, simulators, and bridges that help you
develop and validate the firmware. They build only for the host platform.

```
tools/
├── arm_console/         # e-robot-arm-console: text commands to the arm
├── arm_host/            # the arm on its own thread with its target; shared by console and simulator
├── simulated_arm/       # six simulated e-foc joints on an in-process CAN bus
└── simulator/           # the Qt simulator: view models (no Qt) and the app
```

The tools link the same `core/` libraries the firmware will, so they share code with the controller instead of
duplicating it:

```
e-robot-arm-console --bus simulated
```

## Qt GUI tools

The simulator builds with the `host-qt` preset (Qt 6 with Qt Quick 3D); see `documentation/design/tool-simulator.md`
and the README's "The simulator" section.

## Add a new tool

1. Create `tools/<your_tool>/` with its `CMakeLists.txt`, sources and a `test/` folder, as `arm_console/` does.
2. Keep its logic in a library tested without I/O, and its `main` thin.
3. Add `add_subdirectory(<your_tool>)` to this file.
