---
description: "embedded-scaffold C++ coding rules for embedded/real-time code: no heap allocation, bounded containers, real-time determinism, embedded compiler optimizations, Allman brace style, PascalCase naming, SOLID principles, const correctness, documentation alignment."
applyTo: core/**,targets/**
---

# embedded-scaffold C++ Rules

> **Scope**: These rules apply to embedded and real-time code paths (`core/`
> runtime libraries, `targets/`). Host-side tooling (`tools/`) and test code may use
> standard-library heap allocation where appropriate for a host environment.

This repository is a template for resource-constrained embedded C++ projects.
Follow these rules strictly within the scoped paths.

## Memory — No Heap Allocation

Never use `new`, `delete`, `malloc`, `free`, `std::make_unique`, or `std::make_shared`.

Avoid `virtual ~ClassName() = 0` (pure virtual destructors) — they pull in
`__cxa_pure_virtual` and vtable overhead, increasing flash/RAM usage significantly.
The default is **no pure virtual destructor**; use a protected non-virtual
destructor on interfaces. Only add one when there is a proven, documented need.

Replace standard containers:
- `std::vector<T>` → `infra::BoundedVector<T>::WithMaxSize<N>`
- `std::string` → `infra::BoundedString::WithStorage<N>`
- `std::deque<T>` → `infra::BoundedDeque<T>::WithMaxSize<N>`
- `std::list<T>` → `infra::BoundedList<T>::WithMaxSize<N>`
- Use `std::array<T, N>` for fixed-size arrays
- No recursion — stack usage must be predictable

## Real-Time — Hot-Path Rules

Hot paths (control loops, ISR-reachable code) run under tight cycle budgets.

- No virtual dispatch in hot paths
- No blocking calls in any ISR-reachable code
- State an explicit cycle budget for each timing-critical loop and verify it
- Prefer lookup tables / fixed-point over expensive transcendental calls

Implementation files with hot-path code should enable aggressive optimization:

```cpp
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
```

Apply `OPTIMIZE_FOR_SPEED` (from `numerical/math/CompilerOptimizations.hpp`) on
hot-path methods.

## Naming

- Classes/Methods: `PascalCase` (e.g., `AccumulatorImpl`, `Add()`)
- Member variables: `camelCase` (e.g., `total`, `initialized`)
- Namespaces: lowercase (`example`, `platform`)

## Style

- Allman braces (opening brace on new line), 4-space indent
- Functions ~30 lines max (hard limit ~50); delegate to focused helpers
- Self-documenting code — avoid unnecessary comments
- No implementation in headers — only templated classes may have method bodies in
  the header; all other method bodies go in `.cpp` files
- `const` on all non-mutating methods, `constexpr` where possible
- Fixed-size types: `uint8_t`, `int32_t`, etc.
- Prefer `{}` initialization over `()` (e.g., `float x{ 0.0f }`, `std::size_t n{ 0 }`)

## Design

- SOLID principles — constructor injection, one class = one concern
- Platform dependencies injected via constructor and the platform `Board`/factory
- Reuse `infra/numerical-toolbox/` for PID, filters, and math — do not duplicate
- RAII for resource management

## Documentation — MANDATORY

**Documentation-first rule**: If a change alters any component's observable
behaviour, the corresponding architecture or design document in `documentation/`
must be updated **before or alongside** the code. If no document exists for the
affected component, create one from `documentation/templates/`.

Within architecture and design documents, all visuals must be Mermaid code blocks
or ASCII art — external image references are not allowed.

Full details: [copilot-instructions.md](../copilot-instructions.md),
[CLAUDE.md](../../CLAUDE.md)
