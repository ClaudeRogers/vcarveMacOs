# ClaudeCarve — Project Guide

## What This Is

ClaudeCarve is a macOS-native CNC design and toolpath generation app (inspired by Vectric's VCarve Pro). It lets users create 2D vector designs, generate CNC toolpaths, and export G-code for CNC routers.

## Build & Test

```bash
# Build via Xcode (preferred — produces .app bundle)
xcodebuild -project ClaudeCarve.xcodeproj -scheme ClaudeCarve build

# Run tests via SPM (fast, no Xcode needed)
swift test

# Open in Xcode
open ClaudeCarve.xcodeproj
```

- **58 tests** across 5 test targets, **zero warnings** required
- Swift 6.0 strict concurrency is enabled — all code must be concurrency-safe
- No external dependencies — pure Swift with system frameworks only

## Project Structure

The app uses a **hybrid architecture**: Xcode project for the app target, SPM Package.swift for library modules.

```
ClaudeCarve.xcodeproj    — Xcode project (macOS App target)
Package.swift            — SPM libraries + test targets (no executable)
Sources/
  ClaudeCarveApp/        — App entry point (@main, SwiftUI App, menus) — built by Xcode target
  ClaudeCarveCore/       — Data models: Vector2D/3D, VectorPath, Tool, MaterialSetup, Toolpath, Document
  ClaudeCarveGeometry/   — 11 geometry algorithms (offset, medial axis, booleans, Voronoi, text, etc.)
  ClaudeCarveToolpath/   — 14 toolpath generators (profile, pocket, V-carve, drill, 3D, photo, inlay, etc.)
  ClaudeCarveGCode/      — G-code generator, 11 post-processors, plugin system
  ClaudeCarveIO/         — SVG/DXF/STL/OBJ import, PDF job setup sheet export
  ClaudeCarveUI/         — SwiftUI views (document, canvas, material setup, tool database, 3D preview)
Tests/                   — 5 test targets matching library modules
Resources/
  Info.plist             — App bundle config (bundle ID: com.claudecarve.macos)
  ClaudeCarve.entitlements
  Assets.xcassets/       — App icon (empty slots), accent color
```

### Module Dependency Graph

```
ClaudeCarveApp (Xcode target)
  └── ClaudeCarveUI
        ├── ClaudeCarveCore (no dependencies)
        ├── ClaudeCarveGeometry → Core
        ├── ClaudeCarveToolpath → Core, Geometry
        ├── ClaudeCarveGCode → Core, Toolpath
        └── ClaudeCarveIO → Core, Geometry
```

## Critical API Patterns

**Always read the actual type definitions before writing code that uses them.** The APIs have been wrong in every case where they were guessed.

### ToolpathMove (Sources/ClaudeCarveCore/Toolpath.swift)
```swift
ToolpathMove(type: .rapid/.linear, position: Vector3D, feedRate: Double?)
// Convenience constructors:
.rapid(to: Vector3D)
.linear(to: Vector3D, feed: Double)
.plunge(to: Double, at: Vector2D, feed: Double)
```
- `MotionType` enum: `.rapid`, `.linear`, `.cwArc`, `.ccwArc` — NOT `.rapidMove`/`.linearMove`
- `feedRate` is `Double?` (optional) — always unwrap with `?? defaultValue`

### ComputedToolpath
```swift
ComputedToolpath(configID: UUID, toolID: UUID, moves: [ToolpathMove])
```

### ToolType (Sources/ClaudeCarveCore/Tool.swift)
`.endMill`, `.ballNose`, `.vBit`, `.bullNose`, `.engraving`, `.drill`, `.taperedBallNose`, `.chamfer`, `.threadMill`, `.custom`

### MaterialSetup (Sources/ClaudeCarveCore/Material.swift)
- Uses `xyDatumPosition` (NOT `xyOrigin`)
- Has `zZeroPosition`, `unit`, `width`, `height`, `thickness`

## Concurrency Rules

- Swift 6.0 strict concurrency is enforced project-wide
- UI-related protocols/classes need `@MainActor` (e.g., `CanvasDelegate`, `Coordinator` in DesignCanvasView)
- All core types conform to `Sendable` and `Codable`

## Common Pitfalls

1. **Never guess API shapes** — always `Read` the actual type definition first
2. **Don't name files `main.swift`** if using `@main` attribute — Swift treats `main.swift` as top-level code
3. **When removing unused variables**, check ALL references in the same scope first
4. **Background agents often get API shapes wrong** — always verify and fix their output
5. **`swift test` tests libraries only** — the app target is built by Xcode, not SPM

## Key Files for Reference

| What | Where |
|------|-------|
| Core types | `Sources/ClaudeCarveCore/Toolpath.swift`, `Tool.swift`, `Material.swift`, `Path.swift` |
| App entry | `Sources/ClaudeCarveApp/ClaudeCarveApp.swift` |
| Main UI | `Sources/ClaudeCarveUI/ClaudeCarveDocumentView.swift` |
| Canvas | `Sources/ClaudeCarveUI/DesignCanvasView.swift` |
| Architecture | `ARCHITECTURE.md` |

## Git Conventions

- Commit messages: short summary line, optional body with bullet points
- Always include `Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>`
- Push to `main` branch on GitHub (github.com/ClaudeRogers/vcarveMacOs)
