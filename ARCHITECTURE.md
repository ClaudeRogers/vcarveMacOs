# ClaudeCarve for macOS — Architecture & Implementation Plan

## Overview

This is a macOS-native implementation of CNC design and toolpath generation software,
inspired by Vectric's VCarve Pro. The application enables users to create 2D vector designs,
generate CNC toolpaths, and export G-code for CNC routers.

## Technology Stack

- **Language:** Swift 6.0 (strict concurrency)
- **UI Framework:** SwiftUI (macOS 14+)
- **3D Rendering:** SceneKit (toolpath preview), Metal (2D canvas)
- **Build System:** Swift Package Manager
- **Architecture:** Document-based app with modular library structure

## Module Architecture

```
ClaudeCarveApp (executable)
  └── ClaudeCarveUI (SwiftUI views, document handling)
        ├── ClaudeCarveCore (data models, types)
        ├── ClaudeCarveGeometry (computational geometry)
        ├── ClaudeCarveToolpath (toolpath algorithms)
        ├── ClaudeCarveGCode (G-code generation)
        └── ClaudeCarveIO (file import/export)
```

### ClaudeCarveCore
Core data models shared across all modules:
- `Vector2D`, `Vector3D` — Geometric primitives
- `BoundingBox2D`, `BoundingBox3D` — Axis-aligned bounding boxes
- `VectorPath`, `PathSegment` — 2D vector paths with lines, arcs, Beziers
- `Tool`, `ToolType` — CNC tool definitions
- `MaterialSetup` — Job/material configuration
- `ToolpathConfig` — Toolpath operation parameters
- `ComputedToolpath`, `ToolpathMove` — Generated toolpath data
- `ClaudeCarveDocument` — Top-level document model

### ClaudeCarveGeometry
Computational geometry algorithms:
- `PolygonOffset` — Inward/outward polygon offsetting with self-intersection resolution
- `MedialAxis` — Medial axis transform via distance field ridge detection (for V-carving)
- `PolygonBoolean` — Union, intersection, difference (Weiler-Atherton algorithm)
- `VoronoiDiagram` — Fortune's sweep line Voronoi computation
- `ShapeNesting` — Bottom-Left Fill nesting for material optimization
- `NodeEditor` — Vector node editing (move, add, delete, convert control points)
- `TextCreation` — Text-to-vector via CoreText glyph outlines, text on curve
- `VectorTransforms` — Affine transforms, scale/rotate/mirror/array/align/distribute
- `VectorOperations` — Welding, trimming, filleting, chamfering, path joining, validation
- `ImageTracing` — Bitmap-to-vector tracing (threshold, contour, Bezier fitting)
- `SnapSystem` — Grid, endpoint, midpoint, center, intersection, guideline snapping

### ClaudeCarveToolpath
Toolpath generation algorithms:
- `ProfileToolpathGenerator` — Profile cutting (inside/outside/on-line) with tabs, lead-in/out, ramping
- `PocketToolpathGenerator` — Interior clearing with offset, raster, spiral strategies
- `VCarveToolpathGenerator` — V-carving via medial axis with variable-depth cuts
- `DrillToolpathGenerator` — Drilling with peck drilling support

### ClaudeCarveGCode
G-code output:
- `GCodeGenerator` — Converts `ComputedToolpath` to G-code string
- `PostProcessor` — Machine-specific dialects (GRBL, Mach3, LinuxCNC, etc.)

### ClaudeCarveIO
File import/export:
- `SVGImporter` — Full SVG path data parser (M, L, C, Q, H, V, Z + shapes)
- `DXFImporter` — DXF entity parser (LINE, CIRCLE, ARC, LWPOLYLINE)

### ClaudeCarveUI
SwiftUI interface:
- `ClaudeCarveDocumentView` — Main workspace (canvas, layer panel, toolpath list, properties)
- `MaterialSetupView` — Job setup dialog
- `ToolDatabaseView` — Tool management
- `ToolpathPreview3DView` — 3D SceneKit preview

## Feature Implementation Status

### Phase 1 — Foundation (Current)
- [x] Core data models (Vector2D/3D, Path, Tool, Material, Toolpath)
- [x] Project structure with 7 modules
- [x] Basic polygon offset algorithm
- [x] Medial axis computation (distance field approach)
- [x] Profile toolpath generation with tabs, lead-in/out, ramping
- [x] Pocket toolpath generation (offset and raster strategies)
- [x] V-Carve toolpath generation via medial axis
- [x] Drilling toolpath with peck drilling
- [x] G-code generator with modal optimization
- [x] 11 post-processor definitions (GRBL, Mach3, LinuxCNC, etc.)
- [x] SVG importer (paths, rects, circles, lines, polylines, polygons)
- [x] DXF importer (LINE, CIRCLE, ARC, LWPOLYLINE)
- [x] Document-based SwiftUI app structure
- [x] Layer management UI
- [x] 2D canvas with grid and material bounds
- [x] Material Setup dialog
- [x] Tool Database management
- [x] 3D preview placeholder with SceneKit
- [x] Unit tests for core types and toolpath generation

### Phase 2 — Enhanced Algorithms
- [x] Voronoi diagram computation (Fortune's sweep line algorithm)
- [x] Robust polygon boolean operations (Weiler-Atherton)
- [x] Improved polygon offset (handle self-intersections properly)
- [x] True shape nesting for material optimization (Bottom-Left Fill)
- [x] Toolpath simulation (material removal heightfield)
- [x] Fluting toolpath
- [x] Texture toolpath
- [x] Prism (raised lettering) toolpath
- [x] Thread milling toolpath
- [x] 3D roughing/finishing toolpaths (raster, offset, adaptive, waterline, spiral, pencil)

### Phase 3 — Design Tools
- [x] Interactive vector drawing (mouse-based creation)
- [x] Node editing (move, add, delete, convert control points)
- [x] Text creation with TrueType/OpenType fonts (CoreText glyph extraction)
- [x] Text on curve
- [x] Vector transform tools (affine transforms, scale, rotate, mirror, array, align, distribute)
- [x] Vector welding, trimming, filleting, chamfering
- [x] Vector validation (self-intersection, overlap, winding order detection)
- [x] Image tracing (bitmap to vector conversion with Potrace-style algorithm)
- [x] Snap to grid, guidelines, object snapping (9 snap types)

### Phase 4 — Advanced Features
- [ ] PhotoVCarve (photograph to depth-mapped engraving)
- [ ] Auto-inlay generation
- [ ] Toolpath tiling for oversized jobs
- [ ] Toolpath merging and optimization
- [ ] Wrapped rotary axis support
- [ ] Double-sided machining
- [ ] Production plate engraving (CSV merge)
- [ ] Job setup sheets (PDF export)
- [ ] 3D model import (STL, OBJ)
- [ ] Gadget/plugin extensibility system
- [ ] Custom post-processor editor

## Key Algorithms

### V-Carving (Medial Axis Transform)
The signature algorithm. For each closed vector shape:
1. Compute the medial axis (skeleton) — the locus of inscribed circle centers
2. At each point, depth = inscribed_radius / tan(half_angle)
3. The tool follows the medial axis with smoothly varying Z depth
4. Wide regions can use flat-bottom clearing with a secondary tool

### Profile Offsetting
Uses vertex normal bisector method:
- Compute edge normals for each polygon segment
- At each vertex, compute the bisector of adjacent normals
- Offset each vertex along its bisector by tool_radius
- Handle acute angles with miter limiting

### Pocket Clearing
Offset strategy: Generate concentric inward offsets at stepover intervals
Raster strategy: Scan-line intersection with polygon, zigzag passes

## File Format

Documents are saved as JSON with the `.claudecarve` extension, containing:
- Material/job setup parameters
- Layer structure
- All vector paths (with Bezier curves, arcs)
- Tool database
- Toolpath configurations

## Build & Run

```bash
# Build
swift build

# Run tests
swift test

# Generate Xcode project
swift package generate-xcodeproj

# Or open in Xcode directly
open Package.swift
```
