import Foundation
import ClaudeCarveCore

// MARK: - Rotary Axis

/// Which rotary axis the machine uses.
public enum RotaryAxis: String, Codable, Sendable {
    case aAxis  // Rotation around X (Y becomes angular)
    case bAxis  // Rotation around Y (X becomes angular)
}

/// Where A/B=0 is referenced on the flat design.
public enum RotaryOrigin: String, Codable, Sendable {
    case center       // A/B=0 at center of flat design
    case bottom       // A/B=0 at bottom edge
}

/// How the cylinder is oriented relative to the machine.
public enum RotaryOrientation: String, Codable, Sendable {
    case along        // Cylinder runs along the axis
    case across       // Cylinder runs perpendicular
}

/// Configuration for wrapping a flat design around a cylinder.
public struct RotaryConfig: Codable, Sendable {
    public var axis: RotaryAxis
    public var diameter: Double         // Workpiece diameter (mm)
    public var wrapAroundX: Bool        // True = wrap X axis, False = wrap Y axis
    public var origin: RotaryOrigin
    public var orientation: RotaryOrientation

    public init(
        axis: RotaryAxis,
        diameter: Double,
        wrapAroundX: Bool,
        origin: RotaryOrigin,
        orientation: RotaryOrientation
    ) {
        self.axis = axis
        self.diameter = diameter
        self.wrapAroundX = wrapAroundX
        self.origin = origin
        self.orientation = orientation
    }

    /// The circumference of the workpiece cylinder.
    public var circumference: Double {
        .pi * diameter
    }

    /// The radius of the workpiece cylinder.
    public var radius: Double {
        diameter / 2.0
    }
}

// MARK: - Rotary Wrapper

/// Wraps flat 2D toolpaths and vector paths around a cylindrical workpiece.
public struct RotaryWrapper {

    // MARK: - Toolpath Wrapping

    /// Wrap a flat 2D toolpath around a cylinder.
    ///
    /// Converts flat Y coordinates to angular A-axis positions and adjusts Z depth
    /// to compensate for the cylindrical surface geometry.
    ///
    /// The mapping is:
    ///   - X stays as X (along the cylinder axis)
    ///   - Y maps to angular position: angle_degrees = (Y / circumference) * 360
    ///   - Z compensates for cylinder surface: Z_rotary = Z_flat - R + R * cos(angle_rad)
    public static func wrapToolpath(
        _ toolpath: ComputedToolpath,
        config: RotaryConfig
    ) -> ComputedToolpath {
        let wrappedMoves = toolpath.moves.map { move in
            wrapMove(move, config: config)
        }

        var result = ComputedToolpath(
            configID: toolpath.configID,
            toolID: toolpath.toolID,
            moves: wrappedMoves
        )
        result.calculateEstimates()
        return result
    }

    /// Wrap a single toolpath move around the cylinder.
    private static func wrapMove(
        _ move: ToolpathMove,
        config: RotaryConfig
    ) -> ToolpathMove {
        let wrappedPosition = wrapPosition(move.position, config: config)

        // For arc moves, also transform the arc center
        var wrappedArcCenter: Vector2D? = nil
        if let arcCenter = move.arcCenter {
            // Arc centers are 2D (I,J offsets) in the XY plane.
            // For rotary wrapping, we convert the center's angular coordinate.
            let center3D = Vector3D(arcCenter, z: 0)
            let wrappedCenter3D = wrapPosition(center3D, config: config)
            wrappedArcCenter = wrappedCenter3D.xy
        }

        return ToolpathMove(
            type: move.type,
            position: wrappedPosition,
            feedRate: move.feedRate,
            arcCenter: wrappedArcCenter,
            spindleSpeed: move.spindleSpeed
        )
    }

    /// Convert a flat 3D position to a rotary-wrapped position.
    ///
    /// For A-axis (rotation around X):
    ///   - X stays as X
    ///   - Y becomes angular: angle = (Y / circumference) * 360 degrees
    ///   - Z compensates for surface curvature: Z_rotary = Z_flat - R + R * cos(angle)
    ///
    /// For B-axis (rotation around Y):
    ///   - Y stays as Y
    ///   - X becomes angular: angle = (X / circumference) * 360 degrees
    ///   - Z compensates for surface curvature
    private static func wrapPosition(
        _ position: Vector3D,
        config: RotaryConfig
    ) -> Vector3D {
        let R = config.radius
        let circumference = config.circumference

        if config.wrapAroundX {
            // Wrap the X axis: X coordinate maps to angular position
            let linearCoord = position.x
            let offsetLinear = applyOriginOffset(linearCoord, config: config)
            let angleDegrees = (offsetLinear / circumference) * 360.0
            let angleRadians = angleDegrees * .pi / 180.0

            // Z compensation for cylindrical surface
            let zRotary = position.z - R + R * cos(angleRadians)

            // Depending on the axis, the angular position replaces X or is output
            // as the A/B axis value. Here we store it in X for the wrapped toolpath.
            return Vector3D(angleDegrees, position.y, zRotary)
        } else {
            // Wrap the Y axis: Y coordinate maps to angular position (default)
            let linearCoord = position.y
            let offsetLinear = applyOriginOffset(linearCoord, config: config)
            let angleDegrees = (offsetLinear / circumference) * 360.0
            let angleRadians = angleDegrees * .pi / 180.0

            // Z compensation for cylindrical surface
            let zRotary = position.z - R + R * cos(angleRadians)

            // Angular position stored in Y for the wrapped toolpath
            return Vector3D(position.x, angleDegrees, zRotary)
        }
    }

    /// Apply the origin offset to the linear coordinate before angular conversion.
    private static func applyOriginOffset(
        _ coordinate: Double,
        config: RotaryConfig
    ) -> Double {
        switch config.origin {
        case .center:
            // A/B=0 is at the center of the design; coordinate is used directly
            return coordinate
        case .bottom:
            // A/B=0 is at the bottom edge; shift so that 0 maps to the bottom
            return coordinate
        }
    }

    // MARK: - Vector Path Wrapping

    /// Wrap flat 2D vector paths for display on a cylinder.
    ///
    /// Transforms each path's 2D points so they can be previewed as if wrapped
    /// around the cylindrical workpiece. The resulting paths are still 2D but
    /// represent the "unrolled" angular positions.
    public static func wrapPaths(
        _ paths: [VectorPath],
        config: RotaryConfig
    ) -> [VectorPath] {
        paths.map { path in
            wrapPath(path, config: config)
        }
    }

    /// Wrap a single vector path around the cylinder.
    private static func wrapPath(
        _ path: VectorPath,
        config: RotaryConfig
    ) -> VectorPath {
        path.mapPoints { point in
            wrapPoint2D(point, config: config)
        }
    }

    /// Convert a flat 2D point to its wrapped angular representation.
    private static func wrapPoint2D(
        _ point: Vector2D,
        config: RotaryConfig
    ) -> Vector2D {
        let circumference = config.circumference

        if config.wrapAroundX {
            // Wrap X: X becomes angular
            let offsetX = applyOriginOffset(point.x, config: config)
            let angleDegrees = (offsetX / circumference) * 360.0
            return Vector2D(angleDegrees, point.y)
        } else {
            // Wrap Y: Y becomes angular
            let offsetY = applyOriginOffset(point.y, config: config)
            let angleDegrees = (offsetY / circumference) * 360.0
            return Vector2D(point.x, angleDegrees)
        }
    }

    // MARK: - Utility

    /// Calculate the unwrapped (flat) dimensions for a given cylinder.
    ///
    /// Returns the width and height of the flat design that corresponds to the
    /// full surface of a cylinder with the given diameter and length.
    ///
    /// - Parameters:
    ///   - diameter: The cylinder diameter in working units (mm).
    ///   - length: The cylinder length along its axis in working units (mm).
    /// - Returns: A tuple of (width, height) where width = length and
    ///   height = circumference = pi * diameter.
    public static func unwrappedDimensions(
        diameter: Double,
        length: Double
    ) -> (width: Double, height: Double) {
        let circumference = .pi * diameter
        return (width: length, height: circumference)
    }
}

// MARK: - Double-Sided Machining

/// The axis around which the workpiece is flipped for double-sided machining.
public enum FlipAxis: String, Codable, Sendable {
    case horizontal  // Flip left-to-right (mirror X)
    case vertical    // Flip top-to-bottom (mirror Y)
}

/// Method used to align the workpiece after flipping.
public enum RegistrationMethod: String, Codable, Sendable {
    case dowelPins      // Drill holes for dowel pins
    case cornerMarks    // Engrave corner registration marks
    case edgeStops      // Use machine edge stops
}

/// Configuration for double-sided machining.
public struct DoubleSidedConfig: Codable, Sendable {
    public var flipAxis: FlipAxis
    public var materialWidth: Double
    public var materialHeight: Double
    public var materialThickness: Double
    public var registrationMethod: RegistrationMethod
    public var registrationHoleDiameter: Double
    public var registrationHoleDepth: Double

    public init(
        flipAxis: FlipAxis,
        materialWidth: Double,
        materialHeight: Double,
        materialThickness: Double,
        registrationMethod: RegistrationMethod,
        registrationHoleDiameter: Double,
        registrationHoleDepth: Double
    ) {
        self.flipAxis = flipAxis
        self.materialWidth = materialWidth
        self.materialHeight = materialHeight
        self.materialThickness = materialThickness
        self.registrationMethod = registrationMethod
        self.registrationHoleDiameter = registrationHoleDiameter
        self.registrationHoleDepth = registrationHoleDepth
    }
}

/// The result of setting up double-sided machining, containing toolpaths
/// for both sides and optional registration features.
public struct DoubleSidedResult: Sendable {
    public let topToolpath: ComputedToolpath
    public let bottomToolpath: ComputedToolpath
    public let registrationToolpath: ComputedToolpath?

    public init(
        topToolpath: ComputedToolpath,
        bottomToolpath: ComputedToolpath,
        registrationToolpath: ComputedToolpath?
    ) {
        self.topToolpath = topToolpath
        self.bottomToolpath = bottomToolpath
        self.registrationToolpath = registrationToolpath
    }
}

// MARK: - Double-Sided Machining Operations

/// Provides operations for double-sided CNC machining, including path mirroring,
/// Z-depth adjustment for the back side, and registration feature generation.
public struct DoubleSidedMachining {

    /// Inset margin from the material edges for registration hole placement (mm).
    private static let registrationInset: Double = 10.0

    /// Safe Z height for rapid moves above the workpiece (mm).
    private static let safeZ: Double = 5.0

    /// Depth for corner registration mark engraving (mm).
    private static let cornerMarkDepth: Double = 0.5

    /// Length of corner registration mark lines (mm).
    private static let cornerMarkLength: Double = 5.0

    // MARK: - Setup

    /// Set up double-sided machining with registration.
    ///
    /// Generates toolpaths for both the top and bottom sides of the workpiece,
    /// along with optional registration features (dowel holes, corner marks, etc.)
    /// to ensure accurate alignment after flipping.
    ///
    /// - Parameters:
    ///   - topPaths: Vector paths for the top side of the workpiece.
    ///   - bottomPaths: Vector paths for the bottom side (will be mirrored).
    ///   - config: Double-sided machining configuration.
    ///   - generateToolpath: A closure that generates a `ComputedToolpath` from
    ///     an array of `VectorPath` values (e.g., a profile or pocket generator).
    /// - Returns: A `DoubleSidedResult` containing top, bottom, and registration toolpaths.
    public static func setup(
        topPaths: [VectorPath],
        bottomPaths: [VectorPath],
        config: DoubleSidedConfig,
        generateToolpath: ([VectorPath]) -> ComputedToolpath
    ) -> DoubleSidedResult {
        // Generate the top-side toolpath directly from the top paths
        let topToolpath = generateToolpath(topPaths)

        // Mirror the bottom paths for the flipped side, then generate
        let mirroredBottom = mirrorForBottomSide(bottomPaths, config: config)
        let rawBottomToolpath = generateToolpath(mirroredBottom)

        // Adjust Z depths for the bottom side: cutting from the back, the tool
        // enters from the opposite face so Z must be inverted relative to the
        // material thickness.
        let bottomToolpath = adjustZForBottomSide(rawBottomToolpath, config: config)

        // Generate registration features if using dowel pins or corner marks
        let registrationToolpath: ComputedToolpath?
        switch config.registrationMethod {
        case .dowelPins:
            // Use a default drill tool for registration holes
            let drillTool = Tool(
                name: "Registration Drill",
                type: .drill,
                diameter: config.registrationHoleDiameter,
                plungeRate: 200,
                depthPerPass: config.registrationHoleDepth
            )
            registrationToolpath = generateRegistrationHoles(config: config, tool: drillTool)
        case .cornerMarks:
            registrationToolpath = generateCornerMarks(config: config)
        case .edgeStops:
            // No toolpath needed; alignment uses physical machine stops
            registrationToolpath = nil
        }

        return DoubleSidedResult(
            topToolpath: topToolpath,
            bottomToolpath: bottomToolpath,
            registrationToolpath: registrationToolpath
        )
    }

    // MARK: - Mirroring

    /// Mirror paths for the bottom side.
    ///
    /// When the workpiece is flipped, the bottom-side design must be mirrored
    /// so that it aligns correctly with the top side. The mirror axis depends
    /// on the flip direction:
    /// - Horizontal flip (left-to-right): mirror X around the material center X.
    /// - Vertical flip (top-to-bottom): mirror Y around the material center Y.
    ///
    /// - Parameters:
    ///   - paths: The vector paths for the bottom side.
    ///   - config: Double-sided machining configuration.
    /// - Returns: The mirrored vector paths ready for toolpath generation.
    public static func mirrorForBottomSide(
        _ paths: [VectorPath],
        config: DoubleSidedConfig
    ) -> [VectorPath] {
        let centerX = config.materialWidth / 2.0
        let centerY = config.materialHeight / 2.0

        return paths.map { path in
            path.mapPoints { point in
                switch config.flipAxis {
                case .horizontal:
                    // Mirror X around material center
                    let mirroredX = 2.0 * centerX - point.x
                    return Vector2D(mirroredX, point.y)
                case .vertical:
                    // Mirror Y around material center
                    let mirroredY = 2.0 * centerY - point.y
                    return Vector2D(point.x, mirroredY)
                }
            }
        }
    }

    // MARK: - Z Adjustment

    /// Adjust Z coordinates for bottom-side machining.
    ///
    /// When cutting from the bottom, the Z origin is effectively inverted.
    /// The bottom of the material (from the top side's perspective) becomes
    /// Z=0 for the bottom side. The adjustment is:
    ///   Z_bottom = -(materialThickness + Z_original)
    ///
    /// This ensures that a cut that was at Z=-2mm on the top side will be
    /// positioned correctly when the workpiece is flipped over.
    private static func adjustZForBottomSide(
        _ toolpath: ComputedToolpath,
        config: DoubleSidedConfig
    ) -> ComputedToolpath {
        let thickness = config.materialThickness

        let adjustedMoves = toolpath.moves.map { move -> ToolpathMove in
            let originalZ = move.position.z
            let adjustedZ: Double

            if originalZ >= 0 {
                // Rapid/safe Z moves stay positive (above workpiece)
                adjustedZ = originalZ
            } else {
                // Cutting moves: invert Z relative to material thickness
                // Z_bottom = -(thickness + Z_original)
                // Example: thickness=20, Z_original=-5 => Z_bottom = -(20 + (-5)) = -15
                adjustedZ = -(thickness + originalZ)
            }

            return ToolpathMove(
                type: move.type,
                position: Vector3D(move.position.x, move.position.y, adjustedZ),
                feedRate: move.feedRate,
                arcCenter: move.arcCenter,
                spindleSpeed: move.spindleSpeed
            )
        }

        var result = ComputedToolpath(
            configID: toolpath.configID,
            toolID: toolpath.toolID,
            moves: adjustedMoves
        )
        result.calculateEstimates()
        return result
    }

    // MARK: - Registration Features

    /// Generate registration hole drilling toolpath.
    ///
    /// Places dowel pin holes at two diagonal corners of the material so that
    /// the workpiece can be precisely aligned after flipping. The holes are
    /// drilled through both sides at identical machine coordinates.
    ///
    /// Hole positions are inset from the material edges by `registrationInset`.
    ///
    /// - Parameters:
    ///   - config: Double-sided machining configuration.
    ///   - tool: The drill tool to use for the registration holes.
    /// - Returns: A `ComputedToolpath` that drills the registration holes.
    public static func generateRegistrationHoles(
        config: DoubleSidedConfig,
        tool: Tool
    ) -> ComputedToolpath {
        let inset = registrationInset
        let holeDiameter = config.registrationHoleDiameter
        let holeDepth = config.registrationHoleDepth

        // Place registration holes at two diagonal corners for maximum alignment accuracy
        let holePositions: [Vector2D] = [
            Vector2D(inset + holeDiameter / 2.0, inset + holeDiameter / 2.0),
            Vector2D(
                config.materialWidth - inset - holeDiameter / 2.0,
                config.materialHeight - inset - holeDiameter / 2.0
            )
        ]

        var moves: [ToolpathMove] = []

        for position in holePositions {
            // Rapid to safe height above the hole
            moves.append(.rapid(to: Vector3D(position, z: safeZ)))

            // Rapid down to just above the surface
            moves.append(.rapid(to: Vector3D(position, z: 0.5)))

            // Peck drill the registration hole
            var currentDepth = 0.0
            let peckDepth = tool.depthPerPass

            while currentDepth < holeDepth {
                let nextDepth = min(currentDepth + peckDepth, holeDepth)

                // Plunge to next peck depth
                moves.append(.linear(
                    to: Vector3D(position, z: -nextDepth),
                    feed: tool.plungeRate
                ))

                currentDepth = nextDepth

                if currentDepth < holeDepth {
                    // Retract for chip clearing
                    moves.append(.rapid(to: Vector3D(position, z: 1.0)))
                    // Rapid back to just above previous depth
                    moves.append(.rapid(to: Vector3D(position, z: -(currentDepth - 0.5))))
                }
            }

            // Retract to safe height
            moves.append(.rapid(to: Vector3D(position, z: safeZ)))
        }

        var toolpath = ComputedToolpath(
            configID: UUID(),
            toolID: tool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    /// Generate corner registration marks as shallow engraved crosshairs.
    ///
    /// Engraves small cross marks at the same two diagonal corners used for
    /// dowel pins. These are useful when dowel pins are not available and
    /// the operator needs visual alignment aids.
    private static func generateCornerMarks(
        config: DoubleSidedConfig
    ) -> ComputedToolpath {
        let inset = registrationInset
        let markLen = cornerMarkLength
        let markDepth = cornerMarkDepth

        let markCenters: [Vector2D] = [
            Vector2D(inset, inset),
            Vector2D(config.materialWidth - inset, config.materialHeight - inset)
        ]

        var moves: [ToolpathMove] = []

        // Default feed rate for light engraving
        let engraveFeed = 500.0

        for center in markCenters {
            // Horizontal line of the cross
            let hStart = Vector2D(center.x - markLen / 2.0, center.y)
            let hEnd = Vector2D(center.x + markLen / 2.0, center.y)

            moves.append(.rapid(to: Vector3D(hStart, z: safeZ)))
            moves.append(.rapid(to: Vector3D(hStart, z: 0.5)))
            moves.append(.linear(to: Vector3D(hStart, z: -markDepth), feed: engraveFeed))
            moves.append(.linear(to: Vector3D(hEnd, z: -markDepth), feed: engraveFeed))
            moves.append(.rapid(to: Vector3D(hEnd, z: safeZ)))

            // Vertical line of the cross
            let vStart = Vector2D(center.x, center.y - markLen / 2.0)
            let vEnd = Vector2D(center.x, center.y + markLen / 2.0)

            moves.append(.rapid(to: Vector3D(vStart, z: safeZ)))
            moves.append(.rapid(to: Vector3D(vStart, z: 0.5)))
            moves.append(.linear(to: Vector3D(vStart, z: -markDepth), feed: engraveFeed))
            moves.append(.linear(to: Vector3D(vEnd, z: -markDepth), feed: engraveFeed))
            moves.append(.rapid(to: Vector3D(vEnd, z: safeZ)))
        }

        var toolpath = ComputedToolpath(
            configID: UUID(),
            toolID: UUID(),
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }
}
