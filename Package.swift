// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "ClaudeCarveMacOs",
    platforms: [
        .macOS(.v14)
    ],
    products: [
        .executable(name: "ClaudeCarve", targets: ["ClaudeCarveApp"]),
        .library(name: "ClaudeCarveCore", targets: ["ClaudeCarveCore"]),
        .library(name: "ClaudeCarveGeometry", targets: ["ClaudeCarveGeometry"]),
        .library(name: "ClaudeCarveToolpath", targets: ["ClaudeCarveToolpath"]),
        .library(name: "ClaudeCarveGCode", targets: ["ClaudeCarveGCode"]),
        .library(name: "ClaudeCarveUI", targets: ["ClaudeCarveUI"]),
        .library(name: "ClaudeCarveIO", targets: ["ClaudeCarveIO"]),
    ],
    targets: [
        // Core data models and types
        .target(
            name: "ClaudeCarveCore",
            dependencies: []
        ),
        // Computational geometry: Voronoi, medial axis, polygon ops
        .target(
            name: "ClaudeCarveGeometry",
            dependencies: ["ClaudeCarveCore"]
        ),
        // Toolpath generation algorithms
        .target(
            name: "ClaudeCarveToolpath",
            dependencies: ["ClaudeCarveCore", "ClaudeCarveGeometry"]
        ),
        // G-code generation and post-processors
        .target(
            name: "ClaudeCarveGCode",
            dependencies: ["ClaudeCarveCore", "ClaudeCarveToolpath"]
        ),
        // File import/export (SVG, DXF, STL)
        .target(
            name: "ClaudeCarveIO",
            dependencies: ["ClaudeCarveCore", "ClaudeCarveGeometry"]
        ),
        // SwiftUI-based macOS UI
        .target(
            name: "ClaudeCarveUI",
            dependencies: [
                "ClaudeCarveCore",
                "ClaudeCarveGeometry",
                "ClaudeCarveToolpath",
                "ClaudeCarveGCode",
                "ClaudeCarveIO",
            ]
        ),
        // Main app executable
        .executableTarget(
            name: "ClaudeCarveApp",
            dependencies: ["ClaudeCarveUI"]
        ),
        // Tests
        .testTarget(
            name: "ClaudeCarveCoreTests",
            dependencies: ["ClaudeCarveCore"]
        ),
        .testTarget(
            name: "ClaudeCarveGeometryTests",
            dependencies: ["ClaudeCarveGeometry"]
        ),
        .testTarget(
            name: "ClaudeCarveToolpathTests",
            dependencies: ["ClaudeCarveToolpath"]
        ),
        .testTarget(
            name: "ClaudeCarveGCodeTests",
            dependencies: ["ClaudeCarveGCode"]
        ),
        .testTarget(
            name: "ClaudeCarveIOTests",
            dependencies: ["ClaudeCarveIO"]
        ),
    ]
)
