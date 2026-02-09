# ClaudeCarve

A macOS-native CNC design and toolpath generation application built with Swift and SwiftUI. ClaudeCarve enables you to create 2D vector designs, generate CNC toolpaths (including the signature V-carving algorithm), and export G-code for a wide range of CNC routers and machines.

## Features

### Vector Design
- **Drawing tools**: Line, rectangle, circle, ellipse, polygon, arc
- **Interactive canvas** with zoom, pan, grid, snap, and crosshair cursor
- **Layer management** with visibility and lock controls
- **File import**: SVG (full path data parser) and DXF (LINE, CIRCLE, ARC, LWPOLYLINE)
- **Vector operations**: Bézier curves, arcs, and polylines with adaptive flattening

### Toolpath Generation
- **Profile (contour)** — Cut along or around vectors with inside/outside/on-line options, tabs, lead-in/lead-out arcs, and ramping (straight, helical, profile ramp)
- **Pocket** — Clear interior regions using offset (concentric) or raster (zigzag) strategies with configurable stepover
- **V-Carve** — The signature algorithm. Uses a medial axis transform to compute variable-depth cuts with a V-bit, producing beautiful carved lettering and designs. Supports flat-bottom V-carving with secondary clearing tools
- **Drilling** — Single and peck drilling with configurable retract height and dwell time
- **Fluting** — Tapered groove cutting with smooth ramp-in/ramp-out using Hermite interpolation
- **Texture** — Decorative surface textures (parallel, crosshatch, stipple, wave) with randomized depth variation

### G-Code Output
- **11 post-processor definitions**: GRBL, Mach3/4, LinuxCNC, UCCNC, Carbide Motion, Fanuc, Haas, ShopBot, and generic G-code
- Modal G-code optimization (only outputs changed commands)
- Configurable decimal precision, line numbering, and comments
- Multi-toolpath export with automatic tool changes
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
| **ClaudeCarveGeometry** | PolygonOffset, MedialAxis (distance field ridge detection), PolygonBoolean (Sutherland-Hodgman clipping) |
| **ClaudeCarveToolpath** | ProfileToolpath, PocketToolpath, VCarveToolpath, DrillToolpath, FlutingToolpath, TextureToolpath generators |
| **ClaudeCarveGCode** | GCodeGenerator (with modal optimization), PostProcessor (11 machine dialects) |
| **ClaudeCarveIO** | SVGImporter (paths + shapes), DXFImporter (entities) |
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
- Xcode 15+ or Swift 5.9+
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
│   ├── ClaudeCarveGeometry/               # Computational geometry
│   │   ├── PolygonOffset.swift            # Polygon inward/outward offset
│   │   ├── MedialAxis.swift               # Medial axis transform
│   │   └── PolygonBoolean.swift           # Boolean operations on polygons
│   ├── ClaudeCarveToolpath/               # Toolpath generators
│   │   ├── ProfileToolpath.swift          # Profile/contour cutting
│   │   ├── PocketToolpath.swift           # Pocket clearing
│   │   ├── VCarveToolpath.swift           # V-carving (medial axis)
│   │   ├── DrillToolpath.swift            # Drilling
│   │   ├── FlutingToolpath.swift          # Fluting (tapered grooves)
│   │   └── TextureToolpath.swift          # Surface textures
│   ├── ClaudeCarveGCode/                  # G-code output
│   │   ├── GCodeGenerator.swift           # Toolpath → G-code conversion
│   │   └── PostProcessor.swift            # Machine-specific dialects
│   ├── ClaudeCarveIO/                     # File I/O
│   │   ├── SVGImporter.swift              # SVG file parser
│   │   └── DXFImporter.swift              # DXF file parser
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

### Export
| Format | Extension | Description |
|--------|-----------|-------------|
| G-code | .gcode, .nc, .tap, .ngc | CNC machine instructions |
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
