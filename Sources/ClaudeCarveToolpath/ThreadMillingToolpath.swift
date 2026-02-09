import Foundation
import ClaudeCarveCore

// MARK: - Thread Type Definitions

/// The standard thread form to be milled.
public enum ThreadType: String, Codable, Sendable, CaseIterable {
    case metric     // ISO metric (M-series), pitch in mm
    case imperial   // Unified inch threads (UNC/UNF), pitch as TPI
    case acme       // Trapezoidal ACME threads, 29-degree included angle
    case buttress   // Asymmetric buttress threads, 45/7-degree flanks
}

/// The hand (rotation direction) of the thread helix.
public enum ThreadDirection: String, Codable, Sendable, CaseIterable {
    case rightHand  // Standard right-hand thread (clockwise engagement)
    case leftHand   // Left-hand thread (counter-clockwise engagement)
}

// MARK: - Thread Configuration

/// Complete specification for a thread milling operation.
public struct ThreadConfig: Codable, Sendable {
    /// The thread standard/form.
    public var threadType: ThreadType

    /// Helix direction of the thread.
    public var direction: ThreadDirection

    /// Outer (nominal) diameter of the thread in mm.
    public var majorDiameter: Double

    /// Thread pitch: millimeters per revolution for metric/acme/buttress,
    /// or threads-per-inch (TPI) for imperial.
    public var pitch: Double

    /// Total depth of the threaded section along the Z axis in mm.
    public var depth: Double

    /// True for internal threads (tapped holes); false for external threads (bolts).
    public var isInternal: Bool

    /// Number of radial infeed passes. Each pass cuts deeper toward the final
    /// thread profile. More passes reduce tool load in hard materials.
    public var numberOfPasses: Int

    public init(
        threadType: ThreadType = .metric,
        direction: ThreadDirection = .rightHand,
        majorDiameter: Double = 10.0,
        pitch: Double = 1.5,
        depth: Double = 15.0,
        isInternal: Bool = true,
        numberOfPasses: Int = 1
    ) {
        self.threadType = threadType
        self.direction = direction
        self.majorDiameter = majorDiameter
        self.pitch = pitch
        self.depth = depth
        self.isInternal = isInternal
        self.numberOfPasses = max(1, numberOfPasses)
    }

    // MARK: - Computed Properties

    /// The pitch expressed in mm regardless of thread type.
    /// For imperial threads this converts TPI to mm/rev.
    public var pitchMM: Double {
        switch threadType {
        case .metric, .acme, .buttress:
            return pitch
        case .imperial:
            guard pitch > 0 else { return 0 }
            return 25.4 / pitch
        }
    }

    /// Radial depth of the thread profile on one side, derived from the thread form.
    /// This is the height of the thread tooth measured perpendicular to the axis.
    public var threadDepthPerSide: Double {
        let p = pitchMM
        switch threadType {
        case .metric:
            // ISO metric: H = 0.5 * sqrt(3) * P; actual depth = 5/8 * H
            let h = 0.5 * sqrt(3.0) * p
            return 5.0 / 8.0 * h
        case .imperial:
            // Unified threads share the ISO 60-degree form
            let h = 0.5 * sqrt(3.0) * p
            return 5.0 / 8.0 * h
        case .acme:
            // ACME 29-degree included angle: depth = P / 2
            return p / 2.0
        case .buttress:
            // Buttress thread: depth ~ 0.6 * P
            return 0.6 * p
        }
    }

    /// Inner diameter of the thread (root diameter for external threads,
    /// bore diameter for internal threads before threading).
    public var minorDiameter: Double {
        majorDiameter - 2.0 * threadDepthPerSide
    }
}

// MARK: - Thread Milling Toolpath Generator

/// Generates CNC toolpaths for thread milling operations.
///
/// Thread milling uses a thread mill tool that orbits the thread center
/// in a helical path. Each full revolution around the center descends
/// (or ascends) by exactly one pitch, producing one thread revolution.
///
/// Multi-pass strategies are supported: each successive pass increases
/// the radial depth of cut until the full thread profile is reached,
/// reducing cutting forces and improving surface finish.
public struct ThreadMillingToolpathGenerator: Sendable {

    /// Number of linear segments used to approximate each full helical revolution.
    /// Higher values produce smoother toolpaths at the cost of larger G-code files.
    /// 72 segments = 5-degree angular resolution, a good balance for most applications.
    private static let segmentsPerRevolution: Int = 72

    /// Angle in radians for the lead-in / lead-out ramp arc (quarter circle).
    private static let rampArcAngle: Double = .pi / 2.0

    /// Generate a complete thread milling toolpath.
    ///
    /// - Parameters:
    ///   - config: Thread specification (type, diameter, pitch, depth, etc.)
    ///   - tool: The thread mill tool to use.
    ///   - material: Material/job setup providing safe Z and dimensions.
    ///   - center: The XY center of the thread hole or cylinder.
    /// - Returns: A computed toolpath ready for G-code generation.
    public static func generate(
        config: ThreadConfig,
        tool: Tool,
        material: MaterialSetup,
        center: Vector2D
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        let pitchMM = config.pitchMM
        guard pitchMM > 0, config.majorDiameter > tool.diameter else {
            // Invalid configuration: return empty toolpath
            return ComputedToolpath(configID: UUID(), toolID: tool.id, moves: [])
        }

        // Calculate the orbit radius: distance from thread center to tool center.
        // For internal threads the tool orbits inside the bore.
        // For external threads the tool orbits outside the cylinder.
        let fullThreadDepthPerSide = config.threadDepthPerSide
        let nominalRadius = config.majorDiameter / 2.0
        let toolRadius = tool.radius

        // Number of full thread revolutions that fit in the specified depth
        let totalRevolutions = config.depth / pitchMM

        // Determine the Z travel direction.
        // Conventional: mill from top (Z=0) downward.
        // The helical path descends by one pitch per revolution.
        let threadStartZ = 0.0
        let threadEndZ = -(config.depth)

        // Multi-pass radial infeed: divide the full thread depth across passes.
        let passes = config.numberOfPasses

        for passIndex in 0..<passes {
            let passFraction = Double(passIndex + 1) / Double(passes)
            let currentThreadDepth = fullThreadDepthPerSide * passFraction

            // Orbit radius for this pass
            let orbitRadius: Double
            if config.isInternal {
                // Internal thread: tool center orbits at (minor radius + current depth - tool radius)
                // At full depth the tool center is at (major_radius - tool_radius)
                let threadCrestRadius = nominalRadius - fullThreadDepthPerSide + currentThreadDepth
                orbitRadius = threadCrestRadius - toolRadius
            } else {
                // External thread: tool center orbits outside
                let threadCrestRadius = nominalRadius + fullThreadDepthPerSide - currentThreadDepth
                orbitRadius = threadCrestRadius + toolRadius
            }

            guard orbitRadius > 0 else { continue }

            // Determine the helical direction based on thread hand and internal/external.
            // Right-hand internal threads are conventionally climb-milled with G3 (CCW).
            // Left-hand reverses the orbit direction.
            let climbMilling = true // prefer climb milling for better finish
            var orbitCW: Bool
            if config.isInternal {
                orbitCW = !climbMilling
            } else {
                orbitCW = climbMilling
            }
            if config.direction == .leftHand {
                orbitCW = !orbitCW
            }

            // Starting angle: approach from the +X direction (0 radians)
            let startAngle = 0.0

            // --- Step 1: Rapid to safe Z above thread center ---
            moves.append(.rapid(to: Vector3D(center, z: safeZ)))

            // --- Step 2: Rapid down to the thread start Z at center ---
            // Position at the bottom of the thread first (climb milling goes bottom-up)
            // For conventional top-down milling, start at Z=0.
            let helicalStartZ: Double
            let helicalEndZ: Double
            let zPerRevolution: Double

            // Thread milling typically starts at the bottom and spirals upward
            // so the single-point cutter enters at the bottom of the bore.
            helicalStartZ = threadEndZ
            helicalEndZ = threadStartZ
            zPerRevolution = pitchMM  // ascending

            // Rapid to the thread start Z at center (no tool engagement yet)
            moves.append(.rapid(to: Vector3D(center, z: helicalStartZ)))

            // --- Step 3: Lead-in arc to the orbit radius ---
            // Ramp from center outward to the orbit radius using a quarter-circle arc.
            // This avoids a straight plunge into the material at full radial depth.
            let leadInMoves = generateLeadIn(
                center: center,
                orbitRadius: orbitRadius,
                startAngle: startAngle,
                z: helicalStartZ,
                clockwise: orbitCW,
                feedRate: tool.plungeRate
            )
            moves.append(contentsOf: leadInMoves)

            // --- Step 4: Helical interpolation around the thread ---
            let helicalMoves = generateHelicalPath(
                center: center,
                radius: orbitRadius,
                startAngle: startAngle,
                startZ: helicalStartZ,
                endZ: helicalEndZ,
                zPerRevolution: zPerRevolution,
                clockwise: orbitCW,
                feedRate: tool.feedRate
            )
            moves.append(contentsOf: helicalMoves)

            // --- Step 5: Lead-out arc back to center ---
            let currentAngle = computeExitAngle(
                startAngle: startAngle,
                totalRevolutions: totalRevolutions,
                clockwise: orbitCW
            )
            let leadOutMoves = generateLeadOut(
                center: center,
                orbitRadius: orbitRadius,
                exitAngle: currentAngle,
                z: helicalEndZ,
                clockwise: orbitCW,
                feedRate: tool.plungeRate
            )
            moves.append(contentsOf: leadOutMoves)

            // --- Step 6: Rapid retract to safe Z ---
            moves.append(.rapid(to: Vector3D(center, z: safeZ)))
        }

        var toolpath = ComputedToolpath(configID: UUID(), toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Lead-In

    /// Generate a smooth lead-in arc from the thread center to the orbit radius.
    /// The tool moves radially outward along a quarter-circle so it reaches the
    /// cutting radius tangentially, avoiding shock loading.
    private static func generateLeadIn(
        center: Vector2D,
        orbitRadius: Double,
        startAngle: Double,
        z: Double,
        clockwise: Bool,
        feedRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let leadInRadius = orbitRadius / 2.0
        let steps = 16

        // The lead-in arc center is midway between the thread center and the orbit.
        // The tool traces a semicircular path from center to orbit tangent point.
        let arcCenter = Vector2D(
            center.x + leadInRadius * cos(startAngle),
            center.y + leadInRadius * sin(startAngle)
        )

        // Trace the lead-in arc from center to orbit point
        let arcStartAngle = atan2(center.y - arcCenter.y, center.x - arcCenter.x)
        let arcEndAngle = atan2(
            (center.y + orbitRadius * sin(startAngle)) - arcCenter.y,
            (center.x + orbitRadius * cos(startAngle)) - arcCenter.x
        )

        var sweep = arcEndAngle - arcStartAngle
        if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
        if !clockwise && sweep < 0 { sweep += 2.0 * .pi }

        // Linearize the lead-in arc for reliable motion
        for i in 1...steps {
            let t = Double(i) / Double(steps)
            let angle = arcStartAngle + sweep * t
            let px = arcCenter.x + leadInRadius * cos(angle)
            let py = arcCenter.y + leadInRadius * sin(angle)
            moves.append(.linear(to: Vector3D(px, py, z), feed: feedRate))
        }

        return moves
    }

    // MARK: - Helical Path

    /// Generate the main helical milling path: circular XY motion combined with
    /// linear Z interpolation. Each revolution advances Z by one pitch.
    private static func generateHelicalPath(
        center: Vector2D,
        radius: Double,
        startAngle: Double,
        startZ: Double,
        endZ: Double,
        zPerRevolution: Double,
        clockwise: Bool,
        feedRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        let totalZTravel = abs(endZ - startZ)
        let totalRevolutions = totalZTravel / zPerRevolution
        let totalSegments = Int(totalRevolutions * Double(segmentsPerRevolution))

        guard totalSegments > 0 else { return moves }

        let zDirection = endZ > startZ ? 1.0 : -1.0
        let angleDirection = clockwise ? -1.0 : 1.0
        let anglePerSegment = angleDirection * (2.0 * .pi) / Double(segmentsPerRevolution)
        let zPerSegment = (zDirection * zPerRevolution) / Double(segmentsPerRevolution)

        // Generate arc segments. Each segment is a small arc from the current
        // angular position to the next, with a simultaneous Z step.
        // We use native G2/G3 arc commands with the I,J center offset for
        // accurate circular interpolation on the CNC controller.
        var currentAngle = startAngle
        var currentZ = startZ

        for _ in 0..<totalSegments {
            let nextAngle = currentAngle + anglePerSegment
            let nextZ = currentZ + zPerSegment

            // Clamp Z to prevent overshooting
            let clampedZ: Double
            if zDirection > 0 {
                clampedZ = min(nextZ, endZ)
            } else {
                clampedZ = max(nextZ, endZ)
            }

            let nextX = center.x + radius * cos(nextAngle)
            let nextY = center.y + radius * sin(nextAngle)

            // Current position (for computing incremental I,J)
            let currentX = center.x + radius * cos(currentAngle)
            let currentY = center.y + radius * sin(currentAngle)

            // I, J are incremental offsets from the current position to the arc center
            let iOffset = center.x - currentX
            let jOffset = center.y - currentY

            let motionType: MotionType = clockwise ? .cwArc : .ccwArc
            moves.append(ToolpathMove(
                type: motionType,
                position: Vector3D(nextX, nextY, clampedZ),
                feedRate: feedRate,
                arcCenter: Vector2D(iOffset, jOffset)
            ))

            currentAngle = nextAngle
            currentZ = clampedZ
        }

        return moves
    }

    // MARK: - Lead-Out

    /// Compute the angular position after all helical revolutions.
    private static func computeExitAngle(
        startAngle: Double,
        totalRevolutions: Double,
        clockwise: Bool
    ) -> Double {
        let totalAngle: Double
        if clockwise {
            totalAngle = startAngle - totalRevolutions * 2.0 * .pi
        } else {
            totalAngle = startAngle + totalRevolutions * 2.0 * .pi
        }
        // Normalize to [0, 2*pi)
        var normalized = totalAngle.truncatingRemainder(dividingBy: 2.0 * .pi)
        if normalized < 0 { normalized += 2.0 * .pi }
        return normalized
    }

    /// Generate a smooth lead-out arc from the orbit radius back to the thread center.
    /// Mirrors the lead-in approach for a clean, tangential exit from the cut.
    private static func generateLeadOut(
        center: Vector2D,
        orbitRadius: Double,
        exitAngle: Double,
        z: Double,
        clockwise: Bool,
        feedRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let leadOutRadius = orbitRadius / 2.0
        let steps = 16

        // Current position on the orbit
        let orbitPoint = Vector2D(
            center.x + orbitRadius * cos(exitAngle),
            center.y + orbitRadius * sin(exitAngle)
        )

        // Arc center is midway between orbit point and thread center
        let arcCenter = Vector2D(
            (orbitPoint.x + center.x) / 2.0,
            (orbitPoint.y + center.y) / 2.0
        )

        let arcStartAngle = atan2(orbitPoint.y - arcCenter.y, orbitPoint.x - arcCenter.x)
        let arcEndAngle = atan2(center.y - arcCenter.y, center.x - arcCenter.x)

        var sweep = arcEndAngle - arcStartAngle
        if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
        if !clockwise && sweep < 0 { sweep += 2.0 * .pi }

        for i in 1...steps {
            let t = Double(i) / Double(steps)
            let angle = arcStartAngle + sweep * t
            let px = arcCenter.x + leadOutRadius * cos(angle)
            let py = arcCenter.y + leadOutRadius * sin(angle)
            moves.append(.linear(to: Vector3D(px, py, z), feed: feedRate))
        }

        return moves
    }
}
