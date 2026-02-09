import Foundation
import VCarveCore
import VCarveGeometry

/// Generates pocket toolpaths — clearing interior areas of closed vectors.
/// Supports offset, raster, and spiral clearing strategies.
public struct PocketToolpathGenerator {

    /// Generate a pocket toolpath for the given closed paths.
    public static func generate(
        paths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let depthPerPass = tool.depthPerPass
        let totalDepth = config.cutDepth
        let stepover = tool.stepoverDistance

        for path in paths {
            guard path.isClosed else { continue }
            let contour = path.flattenedPoints(tolerance: 0.01)
            guard contour.count >= 3 else { continue }

            let passes = computeDepthPasses(totalDepth: totalDepth, depthPerPass: depthPerPass)

            for passDepth in passes {
                let z = -passDepth

                switch config.pocketStrategy {
                case .offset:
                    let passMoves = generateOffsetPocket(
                        contour: contour,
                        toolRadius: tool.radius,
                        stepover: stepover,
                        z: z,
                        safeZ: safeZ,
                        feedRate: tool.feedRate,
                        plungeRate: tool.plungeRate,
                        rampType: config.rampType
                    )
                    moves.append(contentsOf: passMoves)

                case .raster:
                    let passMoves = generateRasterPocket(
                        contour: contour,
                        toolRadius: tool.radius,
                        stepover: stepover,
                        z: z,
                        safeZ: safeZ,
                        feedRate: tool.feedRate,
                        plungeRate: tool.plungeRate,
                        angle: 0 // Could be configurable
                    )
                    moves.append(contentsOf: passMoves)

                case .spiral:
                    let passMoves = generateOffsetPocket(
                        contour: contour,
                        toolRadius: tool.radius,
                        stepover: stepover,
                        z: z,
                        safeZ: safeZ,
                        feedRate: tool.feedRate,
                        plungeRate: tool.plungeRate,
                        rampType: config.rampType
                    )
                    moves.append(contentsOf: passMoves)
                }
            }
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Offset pocket

    private static func generateOffsetPocket(
        contour: [Vector2D],
        toolRadius: Double,
        stepover: Double,
        z: Double,
        safeZ: Double,
        feedRate: Double,
        plungeRate: Double,
        rampType: RampType
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        // Generate concentric offset rings
        let firstOffset = PolygonOffset.offset(polygon: contour, distance: -toolRadius)
        guard let outerRing = firstOffset.first, outerRing.count >= 3 else { return moves }

        let rings = PolygonOffset.concentricOffsets(polygon: outerRing, stepover: stepover, inward: true)
        let allRings = [outerRing] + rings

        for ring in allRings {
            guard ring.count >= 2 else { continue }

            // Rapid to start position above safe height
            moves.append(.rapid(to: Vector3D(ring[0], z: safeZ)))

            // Plunge to depth
            moves.append(.plunge(to: z, at: ring[0], feed: plungeRate))

            // Cut around the ring
            for i in 1..<ring.count {
                moves.append(.linear(to: Vector3D(ring[i], z: z), feed: feedRate))
            }

            // Close the ring
            moves.append(.linear(to: Vector3D(ring[0], z: z), feed: feedRate))

            // Retract
            moves.append(.rapid(to: Vector3D(ring.last ?? ring[0], z: safeZ)))
        }

        return moves
    }

    // MARK: - Raster pocket

    private static func generateRasterPocket(
        contour: [Vector2D],
        toolRadius: Double,
        stepover: Double,
        z: Double,
        safeZ: Double,
        feedRate: Double,
        plungeRate: Double,
        angle: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        // Get the bounding box of the contour
        let bb = BoundingBox2D.enclosing(contour)
        let offsetBB = bb.expanded(by: -toolRadius)

        guard offsetBB.width > 0 && offsetBB.height > 0 else { return moves }

        // Generate raster lines
        var y = offsetBB.min.y
        var leftToRight = true

        while y <= offsetBB.max.y {
            // Find intersections of horizontal scan line with polygon
            let intersections = scanLineIntersections(y: y, polygon: contour)

            // Sort intersections by x
            let sorted = intersections.sorted()

            // Process pairs of intersections (entry/exit points)
            var i = 0
            while i + 1 < sorted.count {
                let x1 = sorted[i] + toolRadius
                let x2 = sorted[i + 1] - toolRadius

                if x2 > x1 {
                    let startX = leftToRight ? x1 : x2
                    let endX = leftToRight ? x2 : x1

                    // Rapid to start
                    moves.append(.rapid(to: Vector3D(startX, y, safeZ)))
                    moves.append(.plunge(to: z, at: Vector2D(startX, y), feed: plungeRate))

                    // Cut across
                    moves.append(.linear(to: Vector3D(endX, y, z), feed: feedRate))

                    // Retract
                    moves.append(.rapid(to: Vector3D(endX, y, safeZ)))
                }

                i += 2
            }

            y += stepover
            leftToRight = !leftToRight
        }

        return moves
    }

    /// Find x-coordinates where a horizontal scan line intersects a polygon.
    private static func scanLineIntersections(y: Double, polygon: [Vector2D]) -> [Double] {
        var intersections: [Double] = []
        let n = polygon.count

        for i in 0..<n {
            let p1 = polygon[i]
            let p2 = polygon[(i + 1) % n]

            // Check if scan line crosses this edge
            if (p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y) {
                let t = (y - p1.y) / (p2.y - p1.y)
                let x = p1.x + t * (p2.x - p1.x)
                intersections.append(x)
            }
        }

        return intersections
    }

    // MARK: - Depth passes

    private static func computeDepthPasses(totalDepth: Double, depthPerPass: Double) -> [Double] {
        guard totalDepth > 0 && depthPerPass > 0 else { return [totalDepth] }
        var passes: [Double] = []
        var depth = depthPerPass
        while depth < totalDepth {
            passes.append(depth)
            depth += depthPerPass
        }
        passes.append(totalDepth)
        return passes
    }
}
