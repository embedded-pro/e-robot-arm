# tools

Host-side developer tools — CLI utilities, simulators, and bridges that help you
develop and validate the firmware. They build only for the host platform.

```
tools/
└── example_tool/        # Worked example: a CLI that reuses a core library
    └── Main.cpp
```

`example_tool` links the same `core/` library the firmware uses, demonstrating
how tools share code with the application instead of duplicating it:

```
embedded_scaffold.tool.example 2 3 4   # -> total = 9
```

## Qt GUI tools

The devcontainer ships Qt6, so GUI tools (e.g. a simulator) are supported. See
the commented `find_package(Qt6 ...)` block in `CMakeLists.txt` for how to enable
them, and the README's "How to Run the Simulator" section for X-server setup.

## Add a new tool

1. Copy `example_tool/` to `tools/<your_tool>/`.
2. Update the executable name and sources.
3. Add `add_subdirectory(<your_tool>)` to this file.
