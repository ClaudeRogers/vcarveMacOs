import Foundation
import ClaudeCarveCore

/// Generates texture toolpaths — creating a hand-carved or textured surface appearance.
/// Uses randomized parallel passes with varying depth to simulate hand-carved textures.
public struct TextureToolpathGenerator {

    /// Texture style options.
    public enum TextureStyle: String, Codable, Sendable {
        case parallel       // Parallel lines with randomness
        case crosshatch     // Two sets of parallel lines at angles
        case stipple        // Random dots/plunges
        case wave           // Sinusoidal wave pattern
    }

    /// Generate a texture toolpath within the given closed boundary paths.
    public static func generate(
        boundaryPaths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        material: MaterialSetup,
        style: TextureStyle = .parallel,
        angle: Double = 0,
        spacing: Double = 2.0,
        depthVariation: Double = 0.3,
        randomSeed: UInt64 = 42
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let maxDepth = config.cutDepth

        for path in boundaryPaths {
            guard path.isClosed else { continue }
            let boundary = path.flattenedPoints(tolerance: 0.1)
            guard boundary.count >= 3 else { continue }

            let bb = BoundingBox2D.enclosing(boundary)

            switch style {
            case .parallel:
                let textureMoves = generateParallelTexture(
                    boundary: boundary, bb: bb,
                    angle: angle, spacing: spacing,
                    maxDepth: maxDepth, depthVariation: depthVariation,
                    safeZ: safeZ, tool: tool, seed: randomSeed
                )
                moves.append(contentsOf: textureMoves)

            case .crosshatch:
                let textureMoves1 = generateParallelTexture(
                    boundary: boundary, bb: bb,
                    angle: angle, spacing: spacing,
                    maxDepth: maxDepth * 0.7, depthVariation: depthVariation,
                    safeZ: safeZ, tool: tool, seed: randomSeed
                )
                let textureMoves2 = generateParallelTexture(
                    boundary: boundary, bb: bb,
                    angle: angle + .pi / 2, spacing: spacing,
                    maxDepth: maxDepth * 0.7, depthVariation: depthVariation,
                    safeZ: safeZ, tool: tool, seed: randomSeed + 1000
                )
                moves.append(contentsOf: textureMoves1)
                moves.append(contentsOf: textureMoves2)

            case .stipple:
                let textureMoves = generateStippleTexture(
                    boundary: boundary, bb: bb,
                    spacing: spacing, maxDepth: maxDepth,
                    depthVariation: depthVariation,
                    safeZ: safeZ, tool: tool, seed: randomSeed
                )
                moves.append(contentsOf: textureMoves)

            case .wave:
                let textureMoves = generateWaveTexture(
                    boundary: boundary, bb: bb,
                    angle: angle, spacing: spacing,
                    maxDepth: maxDepth, depthVariation: depthVariation,
                    safeZ: safeZ, tool: tool
                )
                moves.append(contentsOf: textureMoves)
            }
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Parallel texture

    private static func generateParallelTexture(
        boundary: [Vector2D], bb: BoundingBox2D,
        angle: Double, spacing: Double,
        maxDepth: Double, depthVariation: Double,
        safeZ: Double, tool: Tool, seed: UInt64
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        var rng = SimpleRNG(seed: seed)

        // Rotate bounding box to align with angle
        let cos_a = cos(angle)
        let sin_a = sin(angle)
        let diagonal = sqrt(bb.width * bb.width + bb.height * bb.height)
        let center = bb.center

        var y = -diagonal / 2
        var leftToRight = true

        while y <= diagonal / 2 {
            // Create scan line in rotated space
            let lineStart = Vector2D(
                center.x + (-diagonal / 2) * cos_a - y * sin_a,
                center.y + (-diagonal / 2) * sin_a + y * cos_a
            )
            let lineEnd = Vector2D(
                center.x + (diagonal / 2) * cos_a - y * sin_a,
                center.y + (diagonal / 2) * sin_a + y * cos_a
            )

            // Find intersections with boundary
            let intersections = findLinePolygonIntersections(
                lineStart: lineStart, lineEnd: lineEnd, polygon: boundary
            )

            let sorted = intersections.sorted()

            var i = 0
            while i + 1 < sorted.count {
                let t1 = sorted[i]
                let t2 = sorted[i + 1]

                let dir = lineEnd - lineStart
                let p1 = lineStart + dir * t1
                let p2 = lineStart + dir * t2

                let start = leftToRight ? p1 : p2
                let end = leftToRight ? p2 : p1

                // Random depth variation
                let depthOffset = (rng.nextDouble() - 0.5) * 2 * depthVariation * maxDepth
                let lineDepth = max(0.1, maxDepth + depthOffset)

                moves.append(.rapid(to: Vector3D(start, z: safeZ)))
                moves.append(.linear(to: Vector3D(start, z: -lineDepth), feed: tool.plungeRate))
                moves.append(.linear(to: Vector3D(end, z: -lineDepth), feed: tool.feedRate))
                moves.append(.rapid(to: Vector3D(end, z: safeZ)))

                i += 2
            }

            y += spacing + (rng.nextDouble() - 0.5) * spacing * 0.3
            leftToRight = !leftToRight
        }

        return moves
    }

    // MARK: - Stipple texture

    private static func generateStippleTexture(
        boundary: [Vector2D], bb: BoundingBox2D,
        spacing: Double, maxDepth: Double,
        depthVariation: Double,
        safeZ: Double, tool: Tool, seed: UInt64
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        var rng = SimpleRNG(seed: seed)

        let nx = Int(bb.width / spacing)
        let ny = Int(bb.height / spacing)

        for iy in 0..<ny {
            for ix in 0..<nx {
                let x = bb.min.x + (Double(ix) + rng.nextDouble()) * spacing
                let y = bb.min.y + (Double(iy) + rng.nextDouble()) * spacing
                let point = Vector2D(x, y)

                if isPointInsidePolygon(point, boundary) {
                    let depth = maxDepth * (0.5 + rng.nextDouble() * 0.5)

                    moves.append(.rapid(to: Vector3D(point, z: safeZ)))
                    moves.append(.linear(to: Vector3D(point, z: -depth), feed: tool.plungeRate))
                    moves.append(.rapid(to: Vector3D(point, z: safeZ)))
                }
            }
        }

        return moves
    }

    // MARK: - Wave texture

    private static func generateWaveTexture(
        boundary: [Vector2D], bb: BoundingBox2D,
        angle: Double, spacing: Double,
        maxDepth: Double, depthVariation: Double,
        safeZ: Double, tool: Tool
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let wavelength = spacing * 4
        let amplitude = spacing * 0.5

        var y = bb.min.y
        var leftToRight = true

        while y <= bb.max.y {
            let startX = leftToRight ? bb.min.x : bb.max.x
            let endX = leftToRight ? bb.max.x : bb.min.x
            let step = leftToRight ? 0.5 : -0.5

            var firstPoint = true
            var x = startX

            while (leftToRight && x <= endX) || (!leftToRight && x >= endX) {
                let waveOffset = amplitude * sin(2 * .pi * x / wavelength)
                let point = Vector2D(x, y + waveOffset)

                if isPointInsidePolygon(point, boundary) {
                    let depthVar = 0.5 + 0.5 * sin(2 * .pi * x / (wavelength * 1.7))
                    let depth = maxDepth * depthVar

                    if firstPoint {
                        moves.append(.rapid(to: Vector3D(point, z: safeZ)))
                        moves.append(.linear(to: Vector3D(point, z: -depth), feed: tool.plungeRate))
                        firstPoint = false
                    } else {
                        moves.append(.linear(to: Vector3D(point, z: -depth), feed: tool.feedRate))
                    }
                } else if !firstPoint {
                    moves.append(.rapid(to: Vector3D(point, z: safeZ)))
                    firstPoint = true
                }

                x += step
            }

            if !firstPoint {
                moves.append(.rapid(to: Vector3D(Vector2D(x, y), z: safeZ)))
            }

            y += spacing
            leftToRight = !leftToRight
        }

        return moves
    }

    // MARK: - Helpers

    private static func findLinePolygonIntersections(
        lineStart: Vector2D, lineEnd: Vector2D, polygon: [Vector2D]
    ) -> [Double] {
        var intersections: [Double] = []
        let dir = lineEnd - lineStart
        let dirLength = dir.length

        guard dirLength > 1e-10 else { return [] }

        for i in 0..<polygon.count {
            let a = polygon[i]
            let b = polygon[(i + 1) % polygon.count]

            let edgeDir = b - a
            let cross = dir.cross(edgeDir)

            guard abs(cross) > 1e-10 else { continue }

            let diff = a - lineStart
            let t = diff.cross(edgeDir) / cross
            let u = diff.cross(dir) / cross

            if t >= 0 && t <= 1 && u >= 0 && u <= 1 {
                intersections.append(t)
            }
        }

        return intersections
    }

    private static func isPointInsidePolygon(_ point: Vector2D, _ polygon: [Vector2D]) -> Bool {
        var inside = false
        let n = polygon.count
        var j = n - 1
        for i in 0..<n {
            if (polygon[i].y > point.y) != (polygon[j].y > point.y) {
                let xInt = polygon[i].x + (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) * (polygon[j].x - polygon[i].x)
                if point.x < xInt { inside = !inside }
            }
            j = i
        }
        return inside
    }
}

/// Simple deterministic random number generator for reproducible textures.
struct SimpleRNG {
    var state: UInt64

    init(seed: UInt64) {
        state = seed
    }

    mutating func next() -> UInt64 {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return state
    }

    mutating func nextDouble() -> Double {
        Double(next() >> 11) / Double(1 << 53)
    }
}
