import Foundation
import ClaudeCarveCore

/// Generates fluting toolpaths — tapered groove cutting along open or closed vectors.
/// Fluting creates decorative grooves that can ramp in/out at the ends.
public struct FlutingToolpathGenerator {

    /// Generate a fluting toolpath along the given paths.
    public static func generate(
        paths: [VectorPath],
        config: ToolpathConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let cutDepth = config.cutDepth
        let rampInLength = config.rampInLength
        let rampOutLength = config.rampOutLength

        for path in paths {
            let points = path.flattenedPoints(tolerance: 0.05)
            guard points.count >= 2 else { continue }

            // Calculate cumulative distances along the path
            var cumulativeDistances: [Double] = [0]
            for i in 1..<points.count {
                let d = points[i - 1].distance(to: points[i])
                cumulativeDistances.append(cumulativeDistances.last! + d)
            }
            let totalLength = cumulativeDistances.last!

            guard totalLength > 0 else { continue }

            // Rapid to start position
            moves.append(.rapid(to: Vector3D(points[0], z: safeZ)))

            // Generate fluting passes
            let depthPerPass = tool.depthPerPass
            let passes = computeDepthPasses(totalDepth: cutDepth, depthPerPass: depthPerPass)

            for passDepth in passes {
                // Move to start at safe height
                moves.append(.rapid(to: Vector3D(points[0], z: safeZ)))

                // Cut along the path with ramped depth
                for i in 0..<points.count {
                    let dist = cumulativeDistances[i]

                    // Calculate depth with ramp in/out
                    let depth = computeFlutingDepth(
                        distance: dist,
                        totalLength: totalLength,
                        maxDepth: passDepth,
                        rampInLength: rampInLength,
                        rampOutLength: rampOutLength
                    )

                    let z = -depth

                    if i == 0 {
                        // Plunge to start depth (may be 0 if ramping)
                        moves.append(.linear(
                            to: Vector3D(points[i], z: z),
                            feed: tool.plungeRate
                        ))
                    } else {
                        moves.append(.linear(
                            to: Vector3D(points[i], z: z),
                            feed: tool.feedRate
                        ))
                    }
                }

                // Retract
                moves.append(.rapid(to: Vector3D(points.last!, z: safeZ)))
            }
        }

        var toolpath = ComputedToolpath(configID: config.id, toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    /// Compute the fluting depth at a given distance along the path.
    /// Ramps from 0 to maxDepth over rampInLength, holds at maxDepth,
    /// then ramps back to 0 over rampOutLength.
    private static func computeFlutingDepth(
        distance: Double,
        totalLength: Double,
        maxDepth: Double,
        rampInLength: Double,
        rampOutLength: Double
    ) -> Double {
        let rampOutStart = totalLength - rampOutLength

        if distance < rampInLength {
            // Ramping in
            let t = distance / rampInLength
            return maxDepth * smoothStep(t)
        } else if distance > rampOutStart {
            // Ramping out
            let t = (totalLength - distance) / rampOutLength
            return maxDepth * smoothStep(t)
        } else {
            // Full depth
            return maxDepth
        }
    }

    /// Smooth step function for natural-looking ramps (hermite interpolation).
    private static func smoothStep(_ t: Double) -> Double {
        let clamped = max(0, min(1, t))
        return clamped * clamped * (3 - 2 * clamped)
    }

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
