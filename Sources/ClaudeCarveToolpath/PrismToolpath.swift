import Foundation
import ClaudeCarveCore
import ClaudeCarveGeometry

/// Generates prism (raised lettering/chamfer) toolpaths.
/// A prism toolpath uses the side angle of a V-bit to create raised,
/// chamfered shapes — the inverse of V-carving. The tool cuts at an angle
/// around the outside of shapes, leaving a raised prismatic form.
public struct PrismToolpathGenerator {

    /// Generate a prism toolpath for the given closed paths.
    public static func generate(
        paths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let cutDepth = config.cutDepth

        guard tool.type == .vBit || tool.type == .chamfer else {
            return ComputedToolpath(configID: config.id, toolID: tool.id)
        }

        for path in paths {
            guard path.isClosed else { continue }
            let contour = path.flattenedPoints(tolerance: 0.05)
            guard contour.count >= 3 else { continue }

            // For a prism, we cut around the OUTSIDE of the shape
            // The tool rides at an angle, so the outer edge is at full depth
            // and the inner edge (touching the shape) is at the surface

            // Generate multiple offset contours radiating outward
            let maxOffset = tool.vBitWidthAtDepth(cutDepth) / 2
            let stepover = tool.stepoverDistance
            let numPasses = max(1, Int(maxOffset / stepover))

            for passIndex in 0..<numPasses {
                let offsetDist = Double(passIndex + 1) * stepover
                let passDepth = tool.vBitDepthForWidth(offsetDist * 2)
                let clampedDepth = min(passDepth, cutDepth)

                let offsetContours = PolygonOffset.offset(polygon: contour, distance: offsetDist)

                for offsetContour in offsetContours {
                    guard offsetContour.count >= 3 else { continue }

                    // Rapid to start
                    moves.append(.rapid(to: Vector3D(offsetContour[0], z: safeZ)))

                    // Plunge
                    moves.append(.linear(
                        to: Vector3D(offsetContour[0], z: -clampedDepth),
                        feed: tool.plungeRate
                    ))

                    // Cut around the contour
                    for i in 1..<offsetContour.count {
                        moves.append(.linear(
                            to: Vector3D(offsetContour[i], z: -clampedDepth),
                            feed: tool.feedRate
                        ))
                    }

                    // Close
                    moves.append(.linear(
                        to: Vector3D(offsetContour[0], z: -clampedDepth),
                        feed: tool.feedRate
                    ))

                    // Retract
                    moves.append(.rapid(to: Vector3D(offsetContour[0], z: safeZ)))
                }
            }

            // Final cleanup pass right on the vector boundary at full depth
            moves.append(.rapid(to: Vector3D(contour[0], z: safeZ)))
            moves.append(.linear(
                to: Vector3D(contour[0], z: -cutDepth),
                feed: tool.plungeRate
            ))

            for i in 1..<contour.count {
                moves.append(.linear(
                    to: Vector3D(contour[i], z: -cutDepth),
                    feed: tool.feedRate
                ))
            }

            moves.append(.linear(
                to: Vector3D(contour[0], z: -cutDepth),
                feed: tool.feedRate
            ))
            moves.append(.rapid(to: Vector3D(contour[0], z: safeZ)))
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }
}
