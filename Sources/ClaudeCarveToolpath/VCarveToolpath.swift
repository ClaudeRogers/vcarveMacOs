import Foundation
import ClaudeCarveCore
import ClaudeCarveGeometry

/// Generates V-Carve toolpaths — the signature feature.
/// Uses the medial axis transform to compute variable-depth cuts with a V-bit,
/// producing beautiful carved lettering and designs.
///
/// Algorithm:
/// 1. Compute medial axis (skeleton) of the shape
/// 2. At each point on the medial axis, the inscribed circle radius determines the cut depth
/// 3. Depth = radius / tan(halfAngle) where halfAngle is half the V-bit angle
/// 4. Generate smooth toolpath following the medial axis with varying Z
/// 5. Optionally add flat-bottom clearing for wide areas
public struct VCarveToolpathGenerator {

    /// Generate a V-Carve toolpath for the given closed paths.
    public static func generate(
        paths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        secondaryTool: Tool? = nil,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        guard tool.type == .vBit || tool.type == .engraving || tool.type == .chamfer else {
            // V-carving requires a V-bit
            return ComputedToolpath(configID: config.id, toolID: tool.id)
        }

        for path in paths {
            guard path.isClosed else { continue }
            let contour = path.flattenedPoints(tolerance: 0.1)
            guard contour.count >= 3 else { continue }

            // Compute medial axis
            let medialAxis = MedialAxis.compute(for: contour, resolution: 0.3)

            if medialAxis.segments.isEmpty {
                // Fallback: simple centerline V-carve
                let centerLineMoves = generateSimpleVCarve(
                    contour: contour,
                    tool: tool,
                    safeZ: safeZ,
                    maxDepth: config.cutDepth
                )
                moves.append(contentsOf: centerLineMoves)
                continue
            }

            // Generate V-carve moves along the medial axis
            let vcarveMoves = generateMedialAxisVCarve(
                medialAxis: medialAxis,
                tool: tool,
                safeZ: safeZ,
                startDepth: config.startDepth,
                maxDepth: config.cutDepth,
                flatBottomDepth: config.flatBottomDepth
            )
            moves.append(contentsOf: vcarveMoves)

            // Generate flat-bottom clearing if enabled
            if config.useFlatBottomTool, let clearingTool = secondaryTool {
                let clearingMoves = generateFlatBottomClearing(
                    contour: contour,
                    medialAxis: medialAxis,
                    vBit: tool,
                    clearingTool: clearingTool,
                    flatDepth: config.flatBottomDepth,
                    safeZ: safeZ
                )
                moves.append(contentsOf: clearingMoves)
            }
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Medial axis based V-carving

    private static func generateMedialAxisVCarve(
        medialAxis: MedialAxis,
        tool: Tool,
        safeZ: Double,
        startDepth: Double,
        maxDepth: Double,
        flatBottomDepth: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        for segment in medialAxis.segments {
            // Sample the segment at high resolution
            let numSamples = max(10, Int(segment.start.position.distance(to: segment.end.position) / 0.2))

            // Rapid to start
            let startPoint = segment.start
            let startZ = computeVCarveDepth(
                radius: startPoint.radius,
                tool: tool,
                startDepth: startDepth,
                maxDepth: maxDepth,
                flatBottomDepth: flatBottomDepth
            )

            moves.append(.rapid(to: Vector3D(startPoint.position, z: safeZ)))
            moves.append(.linear(
                to: Vector3D(startPoint.position, z: startZ),
                feed: tool.plungeRate
            ))

            // Follow the medial axis with smoothly varying depth
            for i in 1...numSamples {
                let t = Double(i) / Double(numSamples)
                let point = segment.interpolate(t: t)

                let z = computeVCarveDepth(
                    radius: point.radius,
                    tool: tool,
                    startDepth: startDepth,
                    maxDepth: maxDepth,
                    flatBottomDepth: flatBottomDepth
                )

                moves.append(.linear(
                    to: Vector3D(point.position, z: z),
                    feed: tool.feedRate
                ))
            }

            // Retract
            moves.append(.rapid(to: Vector3D(segment.end.position, z: safeZ)))
        }

        return moves
    }

    /// Calculate the V-carve depth at a point given its inscribed circle radius.
    /// depth = radius / tan(halfAngle)
    private static func computeVCarveDepth(
        radius: Double,
        tool: Tool,
        startDepth: Double,
        maxDepth: Double,
        flatBottomDepth: Double
    ) -> Double {
        let halfAngle = tool.halfAngleRadians
        guard halfAngle > 0 else { return -maxDepth }

        let naturalDepth = radius / tan(halfAngle)
        let clampedDepth = min(naturalDepth, maxDepth)

        // For flat-bottom V-carving, limit depth and use flat bottom
        if flatBottomDepth > 0 && naturalDepth > flatBottomDepth {
            return -(startDepth + flatBottomDepth)
        }

        return -(startDepth + clampedDepth)
    }

    // MARK: - Simple V-carve fallback

    /// Simple V-carve using the geometric center approach (for when medial axis fails).
    private static func generateSimpleVCarve(
        contour: [Vector2D],
        tool: Tool,
        safeZ: Double,
        maxDepth: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let bb = BoundingBox2D.enclosing(contour)

        // Use the center of the bounding box as a simple centerline
        let centerY = bb.center.y
        let startX = bb.min.x + tool.radius
        let endX = bb.max.x - tool.radius

        guard endX > startX else { return moves }

        let halfWidth = bb.height / 2
        let depth = min(tool.vBitDepthForWidth(halfWidth * 2), maxDepth)

        moves.append(.rapid(to: Vector3D(startX, centerY, safeZ)))
        moves.append(.linear(to: Vector3D(startX, centerY, -depth), feed: tool.plungeRate))
        moves.append(.linear(to: Vector3D(endX, centerY, -depth), feed: tool.feedRate))
        moves.append(.rapid(to: Vector3D(endX, centerY, safeZ)))

        return moves
    }

    // MARK: - Flat bottom clearing

    /// Generate pocket clearing moves for flat-bottom V-carving.
    private static func generateFlatBottomClearing(
        contour: [Vector2D],
        medialAxis: MedialAxis,
        vBit: Tool,
        clearingTool: Tool,
        flatDepth: Double,
        safeZ: Double
    ) -> [ToolpathMove] {
        // Find regions where the natural V-carve depth exceeds flatDepth
        // and clear them with the flat-bottom tool
        var moves: [ToolpathMove] = []

        // Offset the contour inward by the V-bit width at flatDepth
        let vBitWidthAtFlat = vBit.vBitWidthAtDepth(flatDepth)
        let innerOffset = PolygonOffset.offset(polygon: contour, distance: -(vBitWidthAtFlat / 2))

        for region in innerOffset {
            guard region.count >= 3 else { continue }

            // Simple offset clearing
            let toolPath = region
            moves.append(.rapid(to: Vector3D(toolPath[0], z: safeZ)))
            moves.append(.plunge(to: -flatDepth, at: toolPath[0], feed: clearingTool.plungeRate))

            for i in 1..<toolPath.count {
                moves.append(.linear(to: Vector3D(toolPath[i], z: -flatDepth), feed: clearingTool.feedRate))
            }

            moves.append(.linear(to: Vector3D(toolPath[0], z: -flatDepth), feed: clearingTool.feedRate))
            moves.append(.rapid(to: Vector3D(toolPath[0], z: safeZ)))
        }

        return moves
    }
}
