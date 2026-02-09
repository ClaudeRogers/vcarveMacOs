import Foundation
import ClaudeCarveCore

/// Generates drilling toolpaths with support for peck drilling and dwell.
public struct DrillToolpathGenerator {

    /// Generate a drilling toolpath for circular paths (detected as drill points).
    public static func generate(
        paths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let totalDepth = config.cutDepth

        // Extract drill positions from paths (centers of small circles or single points)
        let drillPositions = extractDrillPositions(from: paths)

        for position in drillPositions {
            // Rapid to position
            moves.append(.rapid(to: Vector3D(position, z: safeZ)))

            if config.peckDrilling {
                // Peck drilling cycle
                let peckMoves = generatePeckDrilling(
                    position: position,
                    totalDepth: totalDepth,
                    peckDepth: config.peckDepth,
                    retractHeight: config.retractHeight,
                    safeZ: safeZ,
                    feedRate: tool.plungeRate,
                    dwellTime: config.dwellTime
                )
                moves.append(contentsOf: peckMoves)
            } else {
                // Simple drilling
                moves.append(.linear(
                    to: Vector3D(position, z: -totalDepth),
                    feed: tool.plungeRate
                ))

                // Dwell if specified
                // (Dwell is handled in G-code output, not in toolpath moves)

                // Retract
                moves.append(.rapid(to: Vector3D(position, z: safeZ)))
            }
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    /// Extract drill positions from vector paths.
    /// Small circles are treated as drill points (using their center).
    /// Single points are used directly.
    private static func extractDrillPositions(from paths: [VectorPath]) -> [Vector2D] {
        var positions: [Vector2D] = []

        for path in paths {
            if path.isClosed {
                // Use the centroid of the path as the drill position
                let points = path.flattenedPoints(tolerance: 0.1)
                if points.count >= 3 {
                    let centroid = points.reduce(Vector2D.zero, +) / Double(points.count)
                    positions.append(centroid)
                }
            } else if path.segments.isEmpty {
                // Single point
                positions.append(path.startPoint)
            } else {
                // Use start point of open paths
                positions.append(path.startPoint)
            }
        }

        return positions
    }

    /// Generate peck drilling moves.
    private static func generatePeckDrilling(
        position: Vector2D,
        totalDepth: Double,
        peckDepth: Double,
        retractHeight: Double,
        safeZ: Double,
        feedRate: Double,
        dwellTime: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        var currentDepth = 0.0

        while currentDepth < totalDepth {
            let nextDepth = min(currentDepth + peckDepth, totalDepth)

            // Plunge to next peck depth
            moves.append(.linear(
                to: Vector3D(position, z: -nextDepth),
                feed: feedRate
            ))

            currentDepth = nextDepth

            if currentDepth < totalDepth {
                // Retract for chip clearing
                moves.append(.rapid(to: Vector3D(position, z: retractHeight)))

                // Rapid back to just above previous depth
                moves.append(.rapid(to: Vector3D(position, z: -(currentDepth - 0.5))))
            }
        }

        // Final retract
        moves.append(.rapid(to: Vector3D(position, z: safeZ)))

        return moves
    }
}
