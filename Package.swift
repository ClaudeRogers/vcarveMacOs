// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "VCarveMacOs",
    platforms: [
        .macOS(.v14)
    ],
    products: [
        .executable(name: "VCarve", targets: ["VCarveApp"]),
        .library(name: "VCarveCore", targets: ["VCarveCore"]),
        .library(name: "VCarveGeometry", targets: ["VCarveGeometry"]),
        .library(name: "VCarveToolpath", targets: ["VCarveToolpath"]),
        .library(name: "VCarveGCode", targets: ["VCarveGCode"]),
        .library(name: "VCarveUI", targets: ["VCarveUI"]),
        .library(name: "VCarveIO", targets: ["VCarveIO"]),
    ],
    targets: [
        // Core data models and types
        .target(
            name: "VCarveCore",
            dependencies: []
        ),
        // Computational geometry: Voronoi, medial axis, polygon ops
        .target(
            name: "VCarveGeometry",
            dependencies: ["VCarveCore"]
        ),
        // Toolpath generation algorithms
        .target(
            name: "VCarveToolpath",
            dependencies: ["VCarveCore", "VCarveGeometry"]
        ),
        // G-code generation and post-processors
        .target(
            name: "VCarveGCode",
            dependencies: ["VCarveCore", "VCarveToolpath"]
        ),
        // File import/export (SVG, DXF, STL)
        .target(
            name: "VCarveIO",
            dependencies: ["VCarveCore", "VCarveGeometry"]
        ),
        // SwiftUI-based macOS UI
        .target(
            name: "VCarveUI",
            dependencies: [
                "VCarveCore",
                "VCarveGeometry",
                "VCarveToolpath",
                "VCarveGCode",
                "VCarveIO",
            ]
        ),
        // Main app executable
        .executableTarget(
            name: "VCarveApp",
            dependencies: ["VCarveUI"]
        ),
        // Tests
        .testTarget(
            name: "VCarveCoreTests",
            dependencies: ["VCarveCore"]
        ),
        .testTarget(
            name: "VCarveGeometryTests",
            dependencies: ["VCarveGeometry"]
        ),
        .testTarget(
            name: "VCarveToolpathTests",
            dependencies: ["VCarveToolpath"]
        ),
    ]
)
