import Foundation
import ClaudeCarveCore
import ClaudeCarveGeometry

/// Generates profile (contour) toolpaths — cutting along or around vector paths.
/// Supports inside, outside, and on-line profiling with tabs, lead-in/out, and ramping.
public struct ProfileToolpathGenerator {

    /// Generate a profile toolpath for the given paths and configuration.
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

        for path in paths {
            let contour = path.flattenedPoints(tolerance: 0.01)
            guard contour.count >= 2 else { continue }

            // Compute the offset contour based on profile side
            let offsetContours = computeOffsetContour(
                contour: contour,
                side: config.profileSide,
                toolRadius: tool.radius,
                allowance: config.allowanceOffset,
                isClosed: path.isClosed
            )

            for offsetContour in offsetContours {
                guard offsetContour.count >= 2 else { continue }

                // Generate passes at increasing depth
                var currentDepth = 0.0
                let passes = computePasses(totalDepth: totalDepth, depthPerPass: depthPerPass)

                for passDepth in passes {
                    currentDepth = passDepth
                    let z = -currentDepth

                    // Rapid to safe height above start
                    let startPoint = offsetContour[0]
                    moves.append(.rapid(to: Vector3D(startPoint, z: safeZ)))

                    // Lead-in
                    if config.leadIn.enabled && path.isClosed {
                        let leadInMoves = generateLeadIn(
                            startPoint: startPoint,
                            direction: offsetContour.count > 1 ? (offsetContour[1] - offsetContour[0]).normalized : Vector2D(1, 0),
                            radius: config.leadIn.radius,
                            z: z,
                            safeZ: safeZ,
                            feedRate: tool.feedRate,
                            plungeRate: tool.plungeRate
                        )
                        moves.append(contentsOf: leadInMoves)
                    } else {
                        // Simple plunge or ramp
                        switch config.rampType {
                        case .straight:
                            moves.append(.plunge(to: z, at: startPoint, feed: tool.plungeRate))
                        case .ramp:
                            let rampMoves = generateRampEntry(
                                along: offsetContour,
                                targetZ: z,
                                rampDistance: config.rampDistance,
                                feedRate: tool.feedRate,
                                plungeRate: tool.plungeRate
                            )
                            moves.append(contentsOf: rampMoves)
                        case .helical:
                            let helicalMoves = generateHelicalEntry(
                                center: startPoint,
                                radius: tool.radius * 0.8,
                                targetZ: z,
                                feedRate: tool.feedRate,
                                plungeRate: tool.plungeRate
                            )
                            moves.append(contentsOf: helicalMoves)
                        case .profile:
                            let rampMoves = generateRampEntry(
                                along: offsetContour,
                                targetZ: z,
                                rampDistance: config.rampDistance,
                                feedRate: tool.feedRate,
                                plungeRate: tool.plungeRate
                            )
                            moves.append(contentsOf: rampMoves)
                        }
                    }

                    // Cut along the contour
                    for i in 1..<offsetContour.count {
                        // Check for tabs
                        if config.addTabs && path.isClosed {
                            let tabZ = z + config.tabHeight
                            let tabMoves = generateMovesWithTabs(
                                from: offsetContour[i - 1],
                                to: offsetContour[i],
                                normalZ: z,
                                tabZ: tabZ,
                                tabWidth: config.tabWidth,
                                tabSpacing: config.tabSpacing,
                                feedRate: tool.feedRate
                            )
                            moves.append(contentsOf: tabMoves)
                        } else {
                            moves.append(.linear(to: Vector3D(offsetContour[i], z: z), feed: tool.feedRate))
                        }
                    }

                    // Close the path if needed
                    if path.isClosed {
                        moves.append(.linear(to: Vector3D(offsetContour[0], z: z), feed: tool.feedRate))
                    }

                    // Lead-out
                    if config.leadOut.enabled && path.isClosed {
                        // Simple retract for now
                        moves.append(.rapid(to: Vector3D(offsetContour.last ?? startPoint, z: safeZ)))
                    } else {
                        moves.append(.rapid(to: Vector3D(offsetContour.last ?? startPoint, z: safeZ)))
                    }
                }
            }
        }

        var toolpath = ComputedToolpath(
            configID: config.id,
            toolID: tool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Offset computation

    private static func computeOffsetContour(
        contour: [Vector2D],
        side: ProfileSide,
        toolRadius: Double,
        allowance: Double,
        isClosed: Bool
    ) -> [[Vector2D]] {
        switch side {
        case .onLine:
            return [contour]
        case .outside:
            return PolygonOffset.offset(polygon: contour, distance: toolRadius + allowance)
        case .inside:
            return PolygonOffset.offset(polygon: contour, distance: -(toolRadius + allowance))
        }
    }

    // MARK: - Pass computation

    private static func computePasses(totalDepth: Double, depthPerPass: Double) -> [Double] {
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

    // MARK: - Entry moves

    private static func generateLeadIn(
        startPoint: Vector2D,
        direction: Vector2D,
        radius: Double,
        z: Double,
        safeZ: Double,
        feedRate: Double,
        plungeRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let normal = direction.perpendicular
        let arcCenter = startPoint - normal * radius
        let arcStart = arcCenter - direction * radius

        moves.append(.rapid(to: Vector3D(arcStart, z: safeZ)))
        moves.append(.plunge(to: z, at: arcStart, feed: plungeRate))

        // Arc from lead-in point to start point
        let arcMoves = generateArc(
            from: arcStart,
            to: startPoint,
            center: arcCenter,
            z: z,
            clockwise: false,
            feedRate: feedRate
        )
        moves.append(contentsOf: arcMoves)

        return moves
    }

    private static func generateRampEntry(
        along contour: [Vector2D],
        targetZ: Double,
        rampDistance: Double,
        feedRate: Double,
        plungeRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        var distanceTraveled = 0.0
        var previousZ = 0.0

        for i in 0..<contour.count {
            if i > 0 {
                distanceTraveled += contour[i - 1].distance(to: contour[i])
            }

            let rampProgress = min(1.0, distanceTraveled / rampDistance)
            let z = previousZ + (targetZ - previousZ) * rampProgress

            if i == 0 {
                moves.append(.linear(to: Vector3D(contour[i], z: 0), feed: feedRate))
                previousZ = 0
            } else {
                moves.append(.linear(to: Vector3D(contour[i], z: z), feed: feedRate))
            }

            if rampProgress >= 1.0 { break }
        }

        return moves
    }

    private static func generateHelicalEntry(
        center: Vector2D,
        radius: Double,
        targetZ: Double,
        feedRate: Double,
        plungeRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let totalRevolutions = max(1, Int(abs(targetZ) / 2.0))
        let steps = totalRevolutions * 36 // 10° per step
        let zPerStep = targetZ / Double(steps)

        for i in 0...steps {
            let angle = Double(i) * (2 * .pi / 36)
            let x = center.x + radius * cos(angle)
            let y = center.y + radius * sin(angle)
            let z = Double(i) * zPerStep

            moves.append(.linear(to: Vector3D(x, y, z), feed: feedRate))
        }

        return moves
    }

    // MARK: - Arc generation

    private static func generateArc(
        from start: Vector2D,
        to end: Vector2D,
        center: Vector2D,
        z: Double,
        clockwise: Bool,
        feedRate: Double,
        steps: Int = 16
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let startAngle = (start - center).angle
        let endAngle = (end - center).angle
        let radius = start.distance(to: center)

        var sweep = endAngle - startAngle
        if clockwise && sweep > 0 { sweep -= 2 * .pi }
        if !clockwise && sweep < 0 { sweep += 2 * .pi }

        for i in 1...steps {
            let t = Double(i) / Double(steps)
            let angle = startAngle + sweep * t
            let p = center + Vector2D.fromAngle(angle) * radius
            moves.append(.linear(to: Vector3D(p, z: z), feed: feedRate))
        }

        return moves
    }

    // MARK: - Tab generation

    private static func generateMovesWithTabs(
        from start: Vector2D,
        to end: Vector2D,
        normalZ: Double,
        tabZ: Double,
        tabWidth: Double,
        tabSpacing: Double,
        feedRate: Double
    ) -> [ToolpathMove] {
        let segLength = start.distance(to: end)
        if segLength < tabWidth {
            return [.linear(to: Vector3D(end, z: normalZ), feed: feedRate)]
        }

        // Simple tab placement - one tab per segment if long enough
        let dir = (end - start).normalized
        let midpoint = start.lerp(to: end, t: 0.5)
        let halfTab = tabWidth / 2

        if segLength > tabSpacing {
            let tabStart = midpoint - dir * halfTab
            let tabEnd = midpoint + dir * halfTab

            return [
                .linear(to: Vector3D(tabStart, z: normalZ), feed: feedRate),
                .linear(to: Vector3D(tabStart, z: tabZ), feed: feedRate),
                .linear(to: Vector3D(tabEnd, z: tabZ), feed: feedRate),
                .linear(to: Vector3D(tabEnd, z: normalZ), feed: feedRate),
                .linear(to: Vector3D(end, z: normalZ), feed: feedRate),
            ]
        }

        return [.linear(to: Vector3D(end, z: normalZ), feed: feedRate)]
    }
}
