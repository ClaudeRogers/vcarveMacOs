# ClaudeCarve

A macOS-native CNC design and toolpath generation application built with Swift and SwiftUI. ClaudeCarve enables you to create 2D vector designs, generate CNC toolpaths (including the signature V-carving algorithm), and export G-code for a wide range of CNC routers and machines.

## Features

### Vector Design
- **Drawing tools**: Line, rectangle, circle, ellipse, polygon, arc
- **Interactive canvas** with zoom, pan, grid, snap, and crosshair cursor
- **Node editing**: Move, add, delete, and convert control points (smooth/corner/symmetric handles)
- **Text creation**: TrueType/OpenType font rendering via CoreText glyph extraction, text on curve
- **Vector transforms**: Scale, rotate, mirror, shear, array (linear/circular/grid), align, distribute
- **Vector operations**: Welding, trimming, filleting, chamfering, path joining, validation
- **Image tracing**: Bitmap-to-vector conversion (threshold, contour detection, Bezier fitting)
- **Snap system**: 9 snap types (grid, endpoint, midpoint, center, intersection, nearest, guideline, tangent, perpendicular)
- **Layer management** with visibility and lock controls
- **File import**: SVG, DXF, STL (binary/ASCII), OBJ (with heightmap generation)
- **Boolean operations**: Union, intersection, difference (Weiler-Atherton algorithm)

### Toolpath Generation
- **Profile (contour)** — Inside/outside/on-line cutting with tabs, lead-in/out arcs, and ramping
- **Pocket** — Interior clearing with offset, raster, and spiral strategies
- **V-Carve** — Medial axis variable-depth cuts for carved lettering and designs
- **Drilling** — Single and peck drilling with configurable retract and dwell
- **Fluting** — Tapered grooves with Hermite interpolation ramp-in/out
- **Texture** — Surface textures (crosshatch, diamond, wave, random, basket weave)
- **Prism** — Raised lettering/prism carving with angle-based depth
- **Thread milling** — Internal/external threads with helical interpolation
- **3D roughing/finishing** — Raster, offset, adaptive, waterline, spiral, and pencil strategies
- **PhotoVCarve** — Photo engraving with 6 strategies (horizontal, vertical, crosshatch, diagonal, stipple, spiral)
- **Auto-inlay** — Standard + backer inlay generation with V-bit pocket, plug, and registration
- **Toolpath tiling** — Grid, zigzag, spiral tiling for oversized jobs with overlap
- **Toolpath merging** — Nearest-neighbor rapid reordering and time estimation
- **Production plate engraving** — CSV merge with single-stroke Hershey font for batch work

### Advanced Machining
- **Rotary axis** — Flat-to-cylinder coordinate wrapping for rotary CNC
- **Double-sided machining** — Flip axis support with registration holes and alignment marks
- **Toolpath simulation** — Material removal heightfield visualization

### G-Code Output
- **11 post-processor definitions**: GRBL, Mach3/4, LinuxCNC, UCCNC, Carbide Motion, Fanuc, Haas, ShopBot, and generic G-code
- **Custom post-processor editor** — Template variables, modal optimization, and presets
- **Plugin/gadget system** — JSON-based extensibility with parameter types and built-in plugins
- Modal G-code optimization (only outputs changed commands)
- Multi-toolpath export with automatic tool changes

### Output & Export
- **G-code export** with configurable precision, line numbering, and comments
- **PDF job setup sheets** — Material info, tool list, toolpath summary (CGContext/CoreText rendering)
- **3D model import** — STL binary/ASCII and OBJ with ray-triangle heightmap generation
- Copy to clipboard or save to file

### 3D Preview
- SceneKit-based 3D visualization with material block and toolpath rendering
- Interactive camera controls (rotate, zoom, pan)
- Material color presets (wood, MDF, acrylic, aluminum, brass, HDPE)

### Material/Job Setup
- Configurable material dimensions and thickness
- Z-zero reference (material surface or machine bed)
- XY datum position (center, corners)
- Safe Z height and home position
- Unit support (millimeters and inches)
- Single-sided, double-sided, and rotary job types

### Tool Database
- Comprehensive tool definitions: end mill, ball nose, V-bit, bull nose, engraving, drill, chamfer, thread mill, tapered ball nose
- Full cutting parameter management: diameter, flute length, spindle speed, feed rate, plunge rate, depth per pass, stepover
- Preset tools for quick setup
- Tool geometry calculations (V-bit depth/width relationships)

## Architecture

ClaudeCarve is built as a modular Swift Package with 7 libraries and 1 executable:

```
ClaudeCarveApp (executable)
  └── ClaudeCarveUI (SwiftUI views, document handling)
        ├── ClaudeCarveCore (data models, types)
        ├── ClaudeCarveGeometry (computational geometry)
        ├── ClaudeCarveToolpath (toolpath algorithms)
        ├── ClaudeCarveGCode (G-code generation)
        └── ClaudeCarveIO (file import/export)
```

| Module | Purpose |
|--------|---------|
| **ClaudeCarveCore** | Vector2D/3D, BoundingBox, VectorPath, Tool, MaterialSetup, ToolpathConfig, ComputedToolpath, ClaudeCarveDocument |
| **ClaudeCarveGeometry** | PolygonOffset, MedialAxis, PolygonBoolean, VoronoiDiagram, ShapeNesting, NodeEditor, TextCreation, VectorTransforms, VectorOperations, ImageTracing, SnapSystem |
| **ClaudeCarveToolpath** | Profile, Pocket, VCarve, Drill, Fluting, Texture, Prism, ThreadMilling, 3D, PhotoVCarve, Inlay, Tiling/Merging, Rotary/DoubleSided, ProductionPlate |
| **ClaudeCarveGCode** | GCodeGenerator, PostProcessor (11 dialects), PluginSystem, CustomPostProcessor |
| **ClaudeCarveIO** | SVGImporter, DXFImporter, ModelImporter (STL/OBJ), JobSetupSheet (PDF) |
| **ClaudeCarveUI** | ClaudeCarveDocumentView, DesignCanvasView (NSView), MaterialSetupView, ToolDatabaseView, GCodeExportView, ToolpathPreview3DView |
| **ClaudeCarveApp** | Main app entry point, menus, settings |

## Key Algorithms

### V-Carving (Medial Axis Transform)

The signature algorithm computes variable-depth toolpaths for V-bit carving:

1. **Flatten** the vector boundary into dense point samples
2. **Compute distance field** on a grid — each interior point stores its distance to the nearest boundary
3. **Detect ridges** — local maxima of the distance field form the medial axis (skeleton)
4. **Map radius to depth** — at each medial axis point, `depth = radius / tan(halfAngle)` where `halfAngle` is half the V-bit tip angle
5. **Generate smooth toolpath** following the medial axis with continuously varying Z depth
6. **Optional flat-bottom clearing** — for wide regions, use a secondary flat-bottom tool

### Polygon Offsetting

Uses the vertex normal bisector method:
- Compute edge normals for adjacent segments
- At each vertex, compute the angle bisector
- Offset along the bisector by `tool_radius / cos(halfAngle)`
- Handle degenerate cases (self-intersections, zero-area results)

### Pocket Clearing

Two strategies:
- **Offset**: Generate concentric inward polygon offsets at stepover intervals
- **Raster**: Horizontal scan-line intersection with polygon boundary, zigzag passes

## Building

### Requirements
- macOS 14.0+
- Xcode 16+ or Swift 6.0+
- No external dependencies

### Build & Run

```bash
# Build
swift build

# Run the app
swift run ClaudeCarve

# Run tests
swift test

# Open in Xcode
open Package.swift
```

### Project Structure

```
ClaudeCarveMacOs/
├── Package.swift                          # Swift Package manifest
├── README.md                              # This file
├── ARCHITECTURE.md                        # Detailed architecture documentation
├── Resources/
│   ├── Info.plist                         # macOS app metadata & document types
│   └── ClaudeCarve.entitlements           # App sandbox entitlements
├── Sources/
│   ├── ClaudeCarveApp/main.swift          # App entry point, menus, settings
│   ├── ClaudeCarveCore/                   # Core data models
│   │   ├── Vector2D.swift                 # 2D vector/point type
│   │   ├── Vector3D.swift                 # 3D vector/point type
│   │   ├── BoundingBox.swift              # Axis-aligned bounding boxes
│   │   ├── Path.swift                     # Vector paths (line, arc, Bézier)
│   │   ├── Tool.swift                     # CNC tool definitions
│   │   ├── Material.swift                 # Job/material setup
│   │   ├── Toolpath.swift                 # Computed toolpath moves
│   │   ├── ToolpathConfig.swift           # Toolpath operation parameters
│   │   └── Document.swift                 # Top-level document model
│   ├── ClaudeCarveGeometry/               # Computational geometry (11 modules)
│   │   ├── PolygonOffset.swift            # Polygon inward/outward offset
│   │   ├── MedialAxis.swift               # Medial axis transform
│   │   ├── PolygonBoolean.swift           # Boolean operations (Weiler-Atherton)
│   │   ├── VoronoiDiagram.swift           # Fortune's sweep line Voronoi
│   │   ├── ShapeNesting.swift             # Bottom-Left Fill nesting
│   │   ├── NodeEditor.swift               # Vector node editing
│   │   ├── TextCreation.swift             # CoreText glyph extraction, text on curve
│   │   ├── VectorTransforms.swift         # Affine transforms, array, align, distribute
│   │   ├── VectorOperations.swift         # Weld, trim, fillet, chamfer, join
│   │   ├── ImageTracing.swift             # Bitmap-to-vector (Potrace-style)
│   │   └── SnapSystem.swift               # 9 snap types + guidelines
│   ├── ClaudeCarveToolpath/               # Toolpath generators (14 modules)
│   │   ├── ProfileToolpath.swift          # Profile/contour cutting
│   │   ├── PocketToolpath.swift           # Pocket clearing
│   │   ├── VCarveToolpath.swift           # V-carving (medial axis)
│   │   ├── DrillToolpath.swift            # Drilling
│   │   ├── FlutingToolpath.swift          # Fluting (tapered grooves)
│   │   ├── TextureToolpath.swift          # Surface textures
│   │   ├── PrismToolpath.swift            # Prism / raised lettering
│   │   ├── ThreadMillingToolpath.swift    # Thread milling (helical)
│   │   ├── ThreeDToolpath.swift           # 3D roughing/finishing
│   │   ├── PhotoVCarveToolpath.swift      # Photo engraving (6 strategies)
│   │   ├── InlayToolpath.swift            # Auto-inlay generation
│   │   ├── ToolpathTilingMerging.swift    # Tiling + merge optimization
│   │   ├── RotaryDoubleSided.swift        # Rotary axis + double-sided
│   │   └── ProductionPlateEngraving.swift # CSV merge batch engraving
│   ├── ClaudeCarveGCode/                  # G-code output
│   │   ├── GCodeGenerator.swift           # Toolpath → G-code conversion
│   │   ├── PostProcessor.swift            # Machine-specific dialects
│   │   └── PluginSystem.swift             # Plugin system + post-processor editor
│   ├── ClaudeCarveIO/                     # File I/O
│   │   ├── SVGImporter.swift              # SVG file parser
│   │   ├── DXFImporter.swift              # DXF file parser
│   │   ├── ModelImporter.swift            # STL/OBJ 3D model import
│   │   └── JobSetupSheet.swift            # PDF setup sheet generation
│   └── ClaudeCarveUI/                     # SwiftUI interface
│       ├── ClaudeCarveDocumentView.swift   # Main workspace
│       ├── ClaudeCarveAppDocument.swift    # Document model for SwiftUI
│       ├── DesignCanvasView.swift          # Interactive NSView canvas
│       ├── MaterialSetupView.swift         # Job setup dialog
│       ├── ToolDatabaseView.swift          # Tool management
│       ├── ToolpathPreview3DView.swift     # 3D SceneKit preview
│       └── GCodeExportView.swift           # G-code export dialog
└── Tests/
    ├── ClaudeCarveCoreTests/              # Core type tests
    ├── ClaudeCarveGeometryTests/          # Geometry algorithm tests
    └── ClaudeCarveToolpathTests/          # Toolpath generation tests
```

## Supported File Formats

### Import
| Format | Extension | Elements Supported |
|--------|-----------|-------------------|
| SVG | .svg | `<path>` (M, L, H, V, C, Q, Z), `<rect>`, `<circle>`, `<line>`, `<polyline>`, `<polygon>` |
| DXF | .dxf | LINE, CIRCLE, ARC, LWPOLYLINE |
| STL | .stl | Binary and ASCII triangle mesh (converted to heightmap) |
| OBJ | .obj | Wavefront OBJ with ray-triangle intersection heightmap |

### Export
| Format | Extension | Description |
|--------|-----------|-------------|
| G-code | .gcode, .nc, .tap, .ngc | CNC machine instructions |
| PDF | .pdf | Job setup sheets (material, tools, toolpaths) |
| ClaudeCarve | .claudecarve | Native project format (JSON) |

## Supported CNC Controllers

| Controller | Post Processor | File Extension |
|-----------|---------------|----------------|
| GRBL | grbl / grblMM / grblInch | .gcode |
| Mach3 / Mach4 | mach3 | .tap |
| LinuxCNC | linuxCNC | .ngc |
| UCCNC | uccnc | .nc |
| Carbide Motion | carbideMotion | .gcode |
| Fanuc | fanuc | .nc |
| Haas | haas | .nc |
| ShopBot | shopbot | .sbp |
| Generic | generic | .gcode |

## Document Format

ClaudeCarve documents (`.claudecarve`) are JSON files containing:
- Material/job setup parameters
- Layer structure with visibility and lock state
- All vector paths (with Bézier curves, arcs, line segments)
- Tool database
- Toolpath configurations

This makes documents human-readable, version-control friendly, and easy to parse with external tools.

## License

This project is provided as-is for educational and personal use.

## Contributing

Contributions are welcome. See ARCHITECTURE.md for detailed implementation notes and the roadmap of planned features.
