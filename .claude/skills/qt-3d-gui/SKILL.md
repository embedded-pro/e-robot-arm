---
name: qt-3d-gui
description: Building a professional Qt 6 desktop GUI with physically based 3-D rendering for e-robot-arm's simulator — Qt Quick 3D scenes (PBR materials, image-based lighting, shadow-casting lights, ambient occlusion, tone mapping, antialiasing), embedding a 3-D view in a Qt Widgets application built on ui-cpp, feeding it live joint data, picking, camera control, performance budgets, and visual polish (typography, icons, spacing, dark and light themes). Use when creating or changing the simulator's 3-D arm view, its scene, lighting or materials, or the overall look of the application; pair it with ux-ui-expert for layout, safety and flow decisions.
---

# Professional Qt GUI with lit, shadowed 3-D

The simulator should look like a professional robotics tool:

- a calm, dense, readable interface;
- an arm rendered with real materials, soft shadows and studio lighting;
- a smooth frame rate with the control loop running.

`ux-ui-expert` decides **what** the screen shows and how it behaves. This skill decides **how it is built and how
it looks**. Load both for any simulator work.

## Architecture

| Layer                                                             | Technology                                                                                                    | Why                                                                                                                                                                                                              |
|-------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Application shell, docks, toolbars, forms, tables, scopes, charts | Qt 6 Widgets with **ui-cpp** (`ui::shell`, `ui::model`, `ui::scope`, `ui::charts`, `ui::theme` via `QtTheme`) | Dense engineering UI, keyboard-first, already themed and tested without Qt                                                                                                                                       |
| 3-D arm view                                                      | **Qt Quick 3D** (`View3D` in a `QQuickWidget`), scene in QML, data from C++                                   | GPU rendering through RHI: PBR materials, image-based lighting, shadow maps, SSAO, MSAA, tone mapping. ui-cpp's `ui::stage` is flat-shaded painter's-algorithm drawing with no shadows (its own limitation list) |
| Scene description                                                 | C++ arm-scene model: links, joints, meshes, materials, poses, taken from the arm profile and arm model        | One description, two renderers: Qt Quick 3D in the simulator, `ui::stage` for tests and lightweight views                                                                                                        |

- **The 3-D scene is a function of data.**
  - C++ exposes one `QObject` (the arm-scene model) to QML. Its properties carry joint values, the ghost
    (commanded) values, the selected joint, the tool trail and status colours.
  - QML binds node rotations and materials to those properties.
  - No arm logic lives in QML. QML only declares the scene.
- **Update at display rate, not control rate.**
  - The model samples the arm's readings on a ≤ 60 Hz timer (`QTimer`, or `QQuickWindow::beforeSynchronizing`)
    and emits one change signal per frame.
  - Never emit per CAN frame or per control period.
- **Ownership:**
  - the QML engine owns the scene items;
  - C++ owns the arm-scene model, which outlives the `QQuickWidget`;
  - nothing in QML holds a raw pointer into `core/`.
- **A missing piece goes upstream.** A Qt Quick 3D renderer for `ui::stage` scenes would serve every toolbox, so
  propose it to ui-cpp as a Tier 3 backend (`ui/backend/qtquick3d`) instead of growing it privately here.

## Scene recipe: industrial robot look

### Environment (`SceneEnvironment`)

- `backgroundMode: SceneEnvironment.Color`, with a neutral background from the theme: dark grey for the
  instrument theme, light grey for the light theme. Alternatively a softly blurred skybox
  (`SkyBox` + `skyboxBlurAmount: 0.6–0.9`) when an HDR studio probe is used.
- **Image-based lighting:**
  - `lightProbe: Texture { source: "studio_small.hdr" }`, a small neutral studio HDRI of 1–2k, shipped as a
    resource with its licence recorded;
  - `probeExposure` around 0.6–1.0 so the probe fills the shadows without flattening the key light.
- **Tone mapping:** `tonemapMode: SceneEnvironment.TonemapModeFilmic`, or `ACES` where available. Never `None` with
  HDR lighting, because highlights clip.
- **Antialiasing:**
  - `antialiasingMode: SceneEnvironment.MSAA` with `antialiasingQuality: SceneEnvironment.High`;
  - `ProgressiveAA` only for still captures, because it smears motion;
  - `specularAAEnabled: true` against shimmering edges on metal.
- **Ambient occlusion:**
  - `aoEnabled: true` with `aoStrength` 40–70, `aoDistance` scaled to the arm (a few centimetres in scene units),
    `aoSoftness` 30–50, `aoDither: true`;
  - AO is what makes joints sit in their housings.
- `depthPrePassEnabled: true` when the scene has many overlapping parts.
- From Qt 6.5 on, `ExtendedSceneEnvironment` adds exposure, glow, vignette and colour grading. Use these sparingly
  (exposure and a faint vignette at most). Guard their use by Qt version: this container has Qt 6.4.

### Lights: three-point plus image-based lighting

| Light | Type                              | Placement                                          | Shadows             | Brightness      |
|-------|-----------------------------------|----------------------------------------------------|---------------------|-----------------|
| Key   | `DirectionalLight`                | 40–55° elevation, 30–45° to the side of the camera | `castsShadow: true` | 1.0 (reference) |
| Fill  | `DirectionalLight`                | Opposite side, low                                 | none                | 0.25–0.4        |
| Rim   | `DirectionalLight` or `SpotLight` | Behind the arm, high                               | none                | 0.4–0.6         |

- **Key-light shadows:**
  - `shadowMapQuality: Light.ShadowMapQualityVeryHigh` (High on weak GPUs);
  - `shadowMapFar` just past the scene bounds, because tight bounds give sharp shadows;
  - `shadowFactor` 60–80, so the shadow does not go pure black;
  - `shadowBias` tuned until there is no acne and no peter-panning: from Qt 6.5 on it is a distance (start around
    5–10 in scene units of 1 cm), but in 6.4 it is a fraction in [−1, 1] and a value like 8 clamps to 1 and removes
    the shadow entirely; leave it at its default unless the version is known;
  - `shadowFilter` 5–15 for soft PCF edges.
  - With Qt 6.8 or later, prefer cascaded shadow maps (`csmNumSplits`) and the soft-shadow quality settings.
- **Floor:**
  - a large `#Rectangle` model under the base with a matte, light-grey `PrincipledMaterial` (roughness 0.9,
    metalness 0) that **receives** shadows;
  - optionally a subtle grid drawn as a second, unlit, slightly transparent model, or with `InfiniteGrid` from
    Qt 6.5 on.
- **No more than four shadow-casting lights.** Usually only one is needed.

### Materials (`PrincipledMaterial`)

| Part                                | baseColor                               | metalness | roughness | Notes                                                                                                                            |
|-------------------------------------|-----------------------------------------|-----------|-----------|----------------------------------------------------------------------------------------------------------------------------------|
| Link shells (painted)               | Theme accent or industrial orange/white | 0.0       | 0.35–0.5  | `clearcoatAmount` 0.3 for a painted look (6.5+)                                                                                  |
| Joint housings (anodized aluminium) | #9aa0a6                                 | 1.0       | 0.3–0.4   |                                                                                                                                  |
| Motor caps, cables (rubber)         | #1d1f21                                 | 0.0       | 0.8       |                                                                                                                                  |
| Base (cast)                         | #3c4043                                 | 0.6       | 0.55      |                                                                                                                                  |
| Ghost (commanded) arm               | Accent colour                           | 0.0       | 0.6       | `opacity` 0.25–0.35, `alphaMode: PrincipledMaterial.Blend`, `depthDrawMode: Material.NeverDepthDraw`                             |
| Selected joint                      | Same as its part                        | —         | —         | Raise `emissiveFactor` slightly in the accent colour; never change geometry                                                      |
| Fault                               | —                                       | —         | —         | Emissive pulse on the faulted joint's housing in the theme's fault colour, plus the text status: colour is never the only signal |

Normal maps and occlusion maps are optional polish. Prefer good geometry and AO over texture work.

### Geometry

- **Meshes:**
  - convert CAD (STEP → STL/glTF) offline with `balsam`, Qt's asset importer, into `.mesh` files with
    generated normals;
  - or load glTF at run time with `RuntimeLoader`;
  - keep each link a few thousand triangles. Decimate CAD meshes.
- **Without CAD,** build links from built-in primitives (`#Cylinder`, `#Cube`, `#Sphere`) scaled per link, with
  housings as short wide cylinders at each joint.
- **Units and axes:**
  - one scene unit = 1 cm keeps shadow and AO parameters intuitive;
  - Qt Quick 3D is Y-up, while the arm model is Z-up, so convert once at the scene root, never per joint;
  - document the convention.
- The kinematic chain is a `Node` hierarchy, one `Node` per joint with `eulerRotation` on the joint axis bound to
  the model. The link geometry is a child with its fixed offset.

### Camera and interaction

- `PerspectiveCamera` with a 35–45° field of view. `clipNear` and `clipFar` should be tight around the scene, for
  depth precision.
- Orbit, pan and zoom come from `OrbitCameraController` (`QtQuick3D.Helpers`) or a custom controller.
- Camera presets (front, side, top, iso) are bound to number keys. "Frame arm" on double-click or `F`.
  Transitions animate over 200–300 ms with easing, never instantly jumping.
- **Picking:** `View3D.pick(x, y)` returns a model. Map the model to its joint through a property, select that
  joint in the joint table, and keep the selection in sync both ways.
- **Overlays:**
  - tool-frame triad;
  - joint-limit arcs on the selected joint;
  - target pose marker;
  - tool trail as a line strip (`Model` with a custom `Geometry`, primitive `LineStrip`), with a capacity bound.

## Embedding and the Widgets application

- Host the scene in a `QQuickWidget` with `setResizeMode(QQuickWidget::SizeRootObjectToView)`, docked like any
  other panel.
  - Expose the arm-scene model with `rootContext()->setContextProperty`, or better, a registered QML singleton or
    required property.
  - Use `QQuickView` with `QWidget::createWindowContainer` only if profiling shows the `QQuickWidget` copy costs
    too much. It complicates stacking and focus.
- Select the graphics backend explicitly at start-up (`QQuickWindow::setGraphicsApi`): OpenGL on Linux by default,
  Direct3D 11 on Windows. Log what was chosen in the About dialog.
- Set the surface format before the application object exists:
  `QSurfaceFormat::setDefaultFormat` with depth 24 and stencil 8, and samples matching MSAA.
- **HiDPI:** `Qt::AA_ShareOpenGLContexts` where needed, devicePixelRatio-aware overlays, SVG icons.

## Look and feel of the whole application

- **Theme:**
  - ui-cpp's themes only (`QtTheme`): the instrument (dark) theme by default for the operator station, and the
    light theme for documentation screenshots;
  - the 3-D background, floor and accent colours come from the same theme roles;
  - no stylesheet outside ui-cpp's `QtTheme`.
- **Typography:**
  - one family, the platform UI font, unless the theme names one;
  - three sizes: body, small and heading;
  - tabular (monospaced) digits for every numeric read-out, so values don't jitter.
- **Spacing:** a 4 px grid, with 8 px between related controls and 16 px between groups. Labels are aligned and
  units sit in suffixes.
- **Icons:**
  - one SVG set with a consistent stroke weight, recoloured by the theme;
  - no mixed icon families and no bitmap icons;
  - every icon button has a tooltip and an accessible name.
- **Motion design:**
  - 150–250 ms ease-out for panel and state transitions;
  - no animation on safety controls, which must respond instantly;
  - camera moves are animated, values are not.
- **Docking:**
  - default layout: 3-D view centre, joint table and status right, scopes bottom, configuration as tabs on the
    left;
  - "Reset layout" is always available;
  - saved per user (`QMainWindow::saveState`).

## Performance budget

- **Target:** 60 fps at 1080p on an integrated GPU, with the control loop at 100 Hz and four scopes running.
- **Measure:**
  - `QSG_RENDERER_DEBUG=render` and `QSG_RHI_DEBUG_LAYER=1` during development;
  - Qt Quick 3D's `DebugView` (render stats, 6.5+) behind a developer toggle;
  - frame time logged once per second in debug builds.
- **When over budget, act in this order:**
  1. reduce mesh triangles;
  2. lower the shadow-map quality one step;
  3. tighten `shadowMapFar`;
  4. reduce AO;
  5. MSAA High → Medium;
  6. never drop below 30 fps, and never drop the shadows entirely without a user toggle.
- **The control loop must never wait for rendering.** The arm runs on its own event loop and thread, as in the
  console. The GUI reads a snapshot, and exchanges data through a lock-free or mutex-protected snapshot struct.

## Pitfalls seen with Qt 6.4

- `SceneEnvironment.aoEnabled` arrives in 6.5. In 6.4 ambient occlusion is on whenever `aoStrength` is above 0; an
  unknown property makes the whole QML file fail to load.
- A `QML_SINGLETON` class that is default-constructible is constructed by the engine, which then ignores the static
  `create()` returning the application's instance. The scene binds to an object nobody updates and shows the zero
  pose forever. Give the class no default constructor.
- `QQuickWidget::grabFramebuffer()` forces a render, so a captured frame proves nothing about live updates. Count
  `QQuickWindow::afterRendering` while the arm moves; zero means the scene does not see the data.
- Qt Quick renders only on change: an arm at rest renders no frames, so measure frame rate during motion.
- A QML module on a static backing library gets a generated plugin that calls into the backing library (Qt 6.10
  emits `qInitResources_qmlcache_*` there). Link the plugin **before** the backing library, or a single-pass linker
  leaves the symbol undefined; Qt 6.4 does not need it, so the mistake only shows on a newer Qt.
- `qmllint` resolves a C++ type only through a QML module (`qt_add_qml_module`); a context property is invisible to
  it. `Connections` needs `import QtQml`. Set `OUTPUT_DIRECTORY` to a path ending in the module URI.

## Testing

- **Scene logic** (the arm-scene model, joint mapping, pick → joint, trail bounds, colours by status) is plain
  C++ and unit-tested without Qt Quick.
- **QML scene:** `qmllint` in CI. A smoke test loads the QML with `QQmlComponent` under
  `QT_QPA_PLATFORM=offscreen` and checks that the root items exist.
- **Rendering:**
  - on CI without a GPU, run with Mesa's software rasteriser (`LIBGL_ALWAYS_SOFTWARE=1`, OpenGL RHI);
  - capture a frame with `QQuickWidget::grabFramebuffer()` at a fixed camera;
  - compare against a reference image with a tolerance, and store the image as a CI artifact for review;
  - keep reference images few and deliberate.
- **Dependencies:** Qt Quick 3D needs `qt6-quick3d-dev`, plus `qt6-declarative-dev` and `qt6-shadertools-dev`.
  The dev container and the CI image must provide them; record the version used.

## Checklist

- [ ] The scene is in QML and the data comes from one C++ model, updated at ≤ 60 Hz.
- [ ] Image-based lighting, a key light with soft shadows, fill and rim lights, a shadow-receiving floor.
- [ ] PBR materials from the table; the ghost arm is translucent and does not write depth.
- [ ] MSAA, filmic tone mapping, AO and specular antialiasing are on.
- [ ] Theme roles drive the background, floor, accents and status colours.
- [ ] Picking, camera presets and frame-arm; the selection stays in sync with the joint table.
- [ ] 60 fps budget measured, and the measurement recorded in `documentation/design/tool-simulator.md`.
- [ ] The scene model is unit-tested and the QML is linted. Frame captures run on CI with a tolerance.
