import Foundation
import ClaudeCarveCore
import ClaudeCarveGeometry

// MARK: - Inlay Types

/// The type of inlay operation to generate.
public enum InlayType: String, Codable, Sendable {
    /// Standard male plug + female pocket cut from the same design.
    /// The plug is cut upside-down so its V-angled walls mate with the pocket.
    case standard

    /// Includes an additional flat backer piece that provides a flush gluing surface.
    /// Used when the inlay needs to sit perfectly flush with the surrounding material.
    case backer
}

/// Configuration for inlay toolpath generation.
///
/// V-bit inlays work by cutting a female pocket (the cavity in the main workpiece)
/// and a male plug (cut upside-down on a separate piece). When the plug is flipped
/// and inserted, the V-angled walls interlock precisely. The glue line gap ensures
/// the plug bottoms out slightly above the pocket floor, leaving room for glue and
/// allowing the excess plug material to be trimmed flush after gluing.
public struct InlayConfig: Codable, Sendable {
    /// Type of inlay operation (standard or with backer).
    public var type: InlayType

    /// Start depth for the pocket, measured from the material surface (mm).
    /// Typically 0.0 unless cutting into an already-recessed area.
    public var startDepth: Double

    /// Total inlay pocket depth (mm). This determines how deep the female pocket
    /// is carved and, combined with the glue line, the depth of the male plug.
    public var inlayDepth: Double

    /// Glue gap at the bottom of the inlay (mm). Typically 0.5-1.0mm.
    /// The male plug is cut deeper by this amount so that when inserted,
    /// the V-walls engage before the plug bottoms out, leaving space for glue.
    public var glueLine: Double

    /// Depth for flat bottom clearing on the backer piece (mm).
    /// Only used when type == .backer. Provides a flat gluing surface.
    public var flatAreaClearance: Double

    /// Fit allowance (mm). Positive values create a looser fit,
    /// negative values create a tighter fit. 0.0 for an exact theoretical fit.
    public var allowance: Double

    /// XY feed rate for cutting moves (mm/min).
    public var feedRate: Double

    /// Z plunge rate (mm/min).
    public var plungeRate: Double

    /// Safe Z height for rapid moves above the workpiece (mm).
    public var safeZ: Double

    public init(
        type: InlayType = .standard,
        startDepth: Double = 0.0,
        inlayDepth: Double = 5.0,
        glueLine: Double = 0.5,
        flatAreaClearance: Double = 0.0,
        allowance: Double = 0.0,
        feedRate: Double = 1000,
        plungeRate: Double = 500,
        safeZ: Double = 5.0
    ) {
        self.type = type
        self.startDepth = startDepth
        self.inlayDepth = inlayDepth
        self.glueLine = glueLine
        self.flatAreaClearance = flatAreaClearance
        self.allowance = allowance
        self.feedRate = feedRate
        self.plungeRate = plungeRate
        self.safeZ = safeZ
    }
}

// MARK: - Inlay Result

/// The result of inlay toolpath generation, containing all required toolpaths.
public struct InlayResult: Sendable {
    /// The female pocket toolpath — carved into the main workpiece.
    /// This is a standard V-carve at the configured inlay depth.
    public let pocketToolpath: ComputedToolpath

    /// The male plug toolpath — cut on a separate piece, to be flipped and inserted.
    /// The paths are mirrored about the X axis (for workpiece flipping) and cut
    /// deeper by the glue line amount.
    public let plugToolpath: ComputedToolpath

    /// Optional flat backer toolpath. Only present when config.type == .backer.
    /// Provides a flat-bottomed clearing pass for a flush gluing surface.
    public let backerToolpath: ComputedToolpath?

    public init(
        pocketToolpath: ComputedToolpath,
        plugToolpath: ComputedToolpath,
        backerToolpath: ComputedToolpath? = nil
    ) {
        self.pocketToolpath = pocketToolpath
        self.plugToolpath = plugToolpath
        self.backerToolpath = backerToolpath
    }
}

// MARK: - Inlay Toolpath Generator

/// Generates toolpaths for V-bit inlay operations.
///
/// V-bit inlays are a woodworking technique where a design is carved into two pieces:
/// 1. A **female pocket** in the main workpiece (the cavity)
/// 2. A **male plug** on a contrasting piece (cut upside-down)
///
/// The plug is flipped and glued into the pocket. Because both pieces are cut with
/// the same V-bit angle, the tapered walls interlock precisely. The glue line gap
/// ensures the plug engages on the walls rather than bottoming out, creating a
/// tight, beautiful fit.
///
/// Algorithm overview:
/// - For each closed path, compute the medial axis to determine variable-depth V-carve lines
/// - The female pocket is a standard V-carve at `inlayDepth`
/// - The male plug mirrors the paths about X (for flipping) and cuts to `inlayDepth + glueLine`
/// - An optional backer piece provides a flat clearing pass for flush surface mounting
public struct InlayToolpath {

    // MARK: - Public API

    /// Generate inlay toolpaths (pocket + plug + optional backer).
    ///
    /// - Parameters:
    ///   - paths: The closed vector paths defining the inlay design.
    ///   - config: Inlay configuration (depths, glue line, allowance, etc.).
    ///   - vBitTool: The V-bit tool used for both pocket and plug carving.
    ///   - clearingTool: Optional flat-bottom tool for clearing wide areas and backer passes.
    /// - Returns: An `InlayResult` containing the pocket, plug, and optional backer toolpaths.
    public static func generate(
        paths: [VectorPath],
        config: InlayConfig,
        vBitTool: Tool,
        clearingTool: Tool?
    ) -> InlayResult {
        let configID = UUID()

        // Generate female pocket toolpath (standard V-carve into main workpiece)
        let pocketToolpath = generatePocketToolpath(
            paths: paths,
            config: config,
            tool: vBitTool,
            clearingTool: clearingTool,
            configID: configID
        )

        // Generate male plug toolpath (mirrored paths, deeper cut for glue line)
        let plugToolpath = generatePlugToolpath(
            paths: paths,
            config: config,
            tool: vBitTool,
            clearingTool: clearingTool,
            configID: configID
        )

        // Generate optional backer toolpath
        let backerToolpath: ComputedToolpath?
        if config.type == .backer {
            backerToolpath = generateBackerToolpath(
                paths: paths,
                config: config,
                clearingTool: clearingTool ?? vBitTool,
                configID: configID
            )
        } else {
            backerToolpath = nil
        }

        return InlayResult(
            pocketToolpath: pocketToolpath,
            plugToolpath: plugToolpath,
            backerToolpath: backerToolpath
        )
    }

    // MARK: - Female Pocket Generation

    /// Generate the female pocket toolpath — a V-carve of the design into the main workpiece.
    private static func generatePocketToolpath(
        paths: [VectorPath],
        config: InlayConfig,
        tool: Tool,
        clearingTool: Tool?,
        configID: UUID
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []

        for path in paths {
            guard path.isClosed else { continue }
            let contour = path.flattenedPoints(tolerance: 0.1)
            guard contour.count >= 3 else { continue }

            // Compute medial axis for variable-depth V-carving
            let medialAxis = MedialAxis.compute(for: contour, resolution: 0.3)

            if medialAxis.segments.isEmpty {
                // Fallback: simple centerline V-carve
                let simpleMoves = generateSimpleVCarve(
                    contour: contour,
                    tool: tool,
                    safeZ: config.safeZ,
                    startDepth: config.startDepth,
                    maxDepth: config.inlayDepth,
                    feedRate: config.feedRate,
                    plungeRate: config.plungeRate,
                    allowanceOffset: config.allowance
                )
                moves.append(contentsOf: simpleMoves)
            } else {
                // Medial axis V-carve at the inlay pocket depth
                let vcarveMoves = generateMedialAxisVCarve(
                    medialAxis: medialAxis,
                    tool: tool,
                    safeZ: config.safeZ,
                    startDepth: config.startDepth,
                    maxDepth: config.inlayDepth,
                    feedRate: config.feedRate,
                    plungeRate: config.plungeRate,
                    allowanceOffset: config.allowance
                )
                moves.append(contentsOf: vcarveMoves)
            }

            // Flat-bottom clearing for wide areas within the pocket
            if let clearTool = clearingTool {
                let clearingMoves = generateFlatClearing(
                    contour: contour,
                    vBitTool: tool,
                    clearingTool: clearTool,
                    depth: config.inlayDepth,
                    safeZ: config.safeZ,
                    feedRate: clearTool.feedRate,
                    plungeRate: clearTool.plungeRate
                )
                moves.append(contentsOf: clearingMoves)
            }
        }

        var toolpath = ComputedToolpath(
            configID: configID,
            toolID: tool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Male Plug Generation

    /// Generate the male plug toolpath.
    ///
    /// The plug is cut on a separate piece of (typically contrasting) material. The design
    /// paths are mirrored about the X axis because the plug piece will be physically flipped
    /// upside-down before being inserted into the pocket. The cut depth is increased by the
    /// glue line amount so that the V-walls extend further, ensuring the plug seats on the
    /// angled walls rather than bottoming out in the pocket.
    private static func generatePlugToolpath(
        paths: [VectorPath],
        config: InlayConfig,
        tool: Tool,
        clearingTool: Tool?,
        configID: UUID
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []

        // Compute the bounding box of all paths to determine the mirror axis
        let allPoints = paths.flatMap { $0.flattenedPoints(tolerance: 0.1) }
        let bounds = BoundingBox2D.enclosing(allPoints)
        let mirrorY = bounds.center.y

        // Plug depth = inlay depth + glue line (cut deeper so walls extend further)
        let plugDepth = config.inlayDepth + config.glueLine

        for path in paths {
            guard path.isClosed else { continue }

            // Mirror the path about the X axis (flip Y coordinates around center)
            // This is necessary because the plug workpiece will be physically flipped
            let mirroredPath = path.mapPoints { point in
                Vector2D(point.x, 2.0 * mirrorY - point.y)
            }

            let contour = mirroredPath.flattenedPoints(tolerance: 0.1)
            guard contour.count >= 3 else { continue }

            // Compute medial axis for the mirrored contour
            let medialAxis = MedialAxis.compute(for: contour, resolution: 0.3)

            if medialAxis.segments.isEmpty {
                // Fallback: simple centerline V-carve at plug depth
                let simpleMoves = generateSimpleVCarve(
                    contour: contour,
                    tool: tool,
                    safeZ: config.safeZ,
                    startDepth: config.startDepth,
                    maxDepth: plugDepth,
                    feedRate: config.feedRate,
                    plungeRate: config.plungeRate,
                    allowanceOffset: -config.allowance  // Inverted for plug (slightly larger)
                )
                moves.append(contentsOf: simpleMoves)
            } else {
                // Medial axis V-carve at the deeper plug depth
                let vcarveMoves = generateMedialAxisVCarve(
                    medialAxis: medialAxis,
                    tool: tool,
                    safeZ: config.safeZ,
                    startDepth: config.startDepth,
                    maxDepth: plugDepth,
                    feedRate: config.feedRate,
                    plungeRate: config.plungeRate,
                    allowanceOffset: -config.allowance  // Inverted for plug
                )
                moves.append(contentsOf: vcarveMoves)
            }

            // Flat-bottom clearing for the plug (at the deeper depth)
            if let clearTool = clearingTool {
                let clearingMoves = generateFlatClearing(
                    contour: contour,
                    vBitTool: tool,
                    clearingTool: clearTool,
                    depth: plugDepth,
                    safeZ: config.safeZ,
                    feedRate: clearTool.feedRate,
                    plungeRate: clearTool.plungeRate
                )
                moves.append(contentsOf: clearingMoves)
            }
        }

        var toolpath = ComputedToolpath(
            configID: configID,
            toolID: tool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Backer Piece Generation

    /// Generate the backer toolpath — a flat pocket clearing pass.
    ///
    /// The backer is a flat-bottomed pocket cut to `flatAreaClearance` depth over the
    /// entire design area. This provides a flat gluing surface so the inlay sits flush
    /// with the surrounding material after trimming.
    private static func generateBackerToolpath(
        paths: [VectorPath],
        config: InlayConfig,
        clearingTool: Tool,
        configID: UUID
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []

        let backerDepth = config.flatAreaClearance > 0
            ? config.flatAreaClearance
            : config.inlayDepth

        let stepover = clearingTool.stepoverDistance
        let depthPerPass = clearingTool.depthPerPass

        for path in paths {
            guard path.isClosed else { continue }
            let contour = path.flattenedPoints(tolerance: 0.1)
            guard contour.count >= 3 else { continue }

            // Compute depth passes for the backer clearing
            let passes = computeDepthPasses(totalDepth: backerDepth, depthPerPass: depthPerPass)

            for passDepth in passes {
                let z = -(config.startDepth + passDepth)

                // Offset inward by tool radius for the first ring
                let firstOffset = PolygonOffset.offset(
                    polygon: contour,
                    distance: -clearingTool.radius
                )
                guard let outerRing = firstOffset.first, outerRing.count >= 3 else { continue }

                // Generate concentric offset rings for pocket clearing
                let innerRings = PolygonOffset.concentricOffsets(
                    polygon: outerRing,
                    stepover: stepover,
                    inward: true
                )
                let allRings = [outerRing] + innerRings

                for ring in allRings {
                    guard ring.count >= 2 else { continue }

                    // Rapid to start position
                    moves.append(.rapid(to: Vector3D(ring[0], z: config.safeZ)))

                    // Plunge to depth
                    moves.append(.plunge(
                        to: z,
                        at: ring[0],
                        feed: config.plungeRate
                    ))

                    // Cut around the ring
                    for i in 1..<ring.count {
                        moves.append(.linear(
                            to: Vector3D(ring[i], z: z),
                            feed: clearingTool.feedRate
                        ))
                    }

                    // Close the ring
                    moves.append(.linear(
                        to: Vector3D(ring[0], z: z),
                        feed: clearingTool.feedRate
                    ))

                    // Retract
                    moves.append(.rapid(to: Vector3D(ring[0], z: config.safeZ)))
                }
            }
        }

        var toolpath = ComputedToolpath(
            configID: configID,
            toolID: clearingTool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Medial Axis V-Carve

    /// Generate V-carve moves along the medial axis with variable depth.
    ///
    /// At each point on the medial axis, the inscribed circle radius determines the natural
    /// V-carve depth: `depth = radius / tan(halfAngle)`. The depth is clamped to `maxDepth`
    /// to respect the configured inlay depth limit.
    ///
    /// - Parameters:
    ///   - medialAxis: The computed medial axis of the contour.
    ///   - tool: The V-bit tool.
    ///   - safeZ: Safe retract height.
    ///   - startDepth: Starting depth offset from material surface.
    ///   - maxDepth: Maximum allowed cut depth (inlay depth or plug depth).
    ///   - feedRate: XY cutting feed rate.
    ///   - plungeRate: Z plunge feed rate.
    ///   - allowanceOffset: Fit allowance adjustment. Positive shrinks the pocket,
    ///     negative expands it. Applied as an offset to the inscribed radius.
    private static func generateMedialAxisVCarve(
        medialAxis: MedialAxis,
        tool: Tool,
        safeZ: Double,
        startDepth: Double,
        maxDepth: Double,
        feedRate: Double,
        plungeRate: Double,
        allowanceOffset: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let halfAngle = tool.halfAngleRadians
        guard halfAngle > 0 else { return moves }

        for segment in medialAxis.segments {
            // Determine sampling resolution based on segment length
            let segmentLength = segment.start.position.distance(to: segment.end.position)
            let numSamples = max(10, Int(segmentLength / 0.2))

            // Compute depth at segment start
            let startPoint = segment.start
            let startRadius = max(0, startPoint.radius + allowanceOffset)
            let startNaturalDepth = startRadius / tan(halfAngle)
            let startZ = -(startDepth + min(startNaturalDepth, maxDepth))

            // Rapid to segment start at safe height
            moves.append(.rapid(to: Vector3D(startPoint.position, z: safeZ)))

            // Plunge to cutting depth
            moves.append(ToolpathMove(
                type: .linear,
                position: Vector3D(startPoint.position, z: startZ),
                feedRate: plungeRate
            ))

            // Follow the medial axis with smoothly varying depth
            for i in 1...numSamples {
                let t = Double(i) / Double(numSamples)
                let point = segment.interpolate(t: t)

                // Adjust radius by the allowance offset
                let adjustedRadius = max(0, point.radius + allowanceOffset)

                // Compute V-carve depth: depth = radius / tan(halfAngle)
                let naturalDepth = adjustedRadius / tan(halfAngle)
                let clampedDepth = min(naturalDepth, maxDepth)
                let z = -(startDepth + clampedDepth)

                moves.append(.linear(
                    to: Vector3D(point.position, z: z),
                    feed: feedRate
                ))
            }

            // Retract to safe height at segment end
            moves.append(.rapid(to: Vector3D(segment.end.position, z: safeZ)))
        }

        return moves
    }

    // MARK: - Simple V-Carve Fallback

    /// Simple V-carve using a bounding-box centerline approach.
    ///
    /// Used as a fallback when the medial axis computation produces no segments
    /// (e.g., for very thin or degenerate shapes). Cuts a single pass along the
    /// horizontal center of the contour's bounding box at a depth determined by
    /// the contour width and V-bit angle.
    private static func generateSimpleVCarve(
        contour: [Vector2D],
        tool: Tool,
        safeZ: Double,
        startDepth: Double,
        maxDepth: Double,
        feedRate: Double,
        plungeRate: Double,
        allowanceOffset: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let bb = BoundingBox2D.enclosing(contour)

        let centerY = bb.center.y
        let startX = bb.min.x + tool.radius
        let endX = bb.max.x - tool.radius

        guard endX > startX else { return moves }

        // Compute depth from contour width, adjusted by allowance
        let halfWidth = max(0, (bb.height / 2.0) + allowanceOffset)
        let naturalDepth = tool.vBitDepthForWidth(halfWidth * 2.0)
        let depth = startDepth + min(naturalDepth, maxDepth)

        // Rapid to start
        moves.append(.rapid(to: Vector3D(startX, centerY, safeZ)))

        // Plunge
        moves.append(ToolpathMove(
            type: .linear,
            position: Vector3D(startX, centerY, -depth),
            feedRate: plungeRate
        ))

        // Cut along centerline
        moves.append(.linear(
            to: Vector3D(endX, centerY, -depth),
            feed: feedRate
        ))

        // Retract
        moves.append(.rapid(to: Vector3D(endX, centerY, safeZ)))

        return moves
    }

    // MARK: - Flat Bottom Clearing

    /// Generate flat-bottom clearing moves for areas where the V-carve exceeds a given depth.
    ///
    /// In wide areas of the design, the V-bit would need to cut very deep. Instead, we
    /// use a flat-bottom clearing tool to remove material in those regions, and the V-bit
    /// only cuts the tapered edges. The boundary between V-carve and flat clearing is
    /// determined by offsetting the contour inward by the V-bit width at the clearing depth.
    private static func generateFlatClearing(
        contour: [Vector2D],
        vBitTool: Tool,
        clearingTool: Tool,
        depth: Double,
        safeZ: Double,
        feedRate: Double,
        plungeRate: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        // Calculate the V-bit cut width at the target depth
        let vBitWidthAtDepth = vBitTool.vBitWidthAtDepth(depth)

        // Offset inward by the V-bit half-width at depth to find the flat clearing region
        let innerOffset = PolygonOffset.offset(
            polygon: contour,
            distance: -(vBitWidthAtDepth / 2.0)
        )

        for region in innerOffset {
            guard region.count >= 3 else { continue }

            // Further offset inward by the clearing tool radius
            let toolOffset = PolygonOffset.offset(
                polygon: region,
                distance: -clearingTool.radius
            )

            for toolRegion in toolOffset {
                guard toolRegion.count >= 3 else { continue }

                // Generate concentric clearing passes within this region
                let innerRings = PolygonOffset.concentricOffsets(
                    polygon: toolRegion,
                    stepover: clearingTool.stepoverDistance,
                    inward: true
                )
                let allRings = [toolRegion] + innerRings

                // Compute depth passes for incremental clearing
                let passes = computeDepthPasses(
                    totalDepth: depth,
                    depthPerPass: clearingTool.depthPerPass
                )

                for passDepth in passes {
                    let z = -passDepth

                    for ring in allRings {
                        guard ring.count >= 2 else { continue }

                        moves.append(.rapid(to: Vector3D(ring[0], z: safeZ)))
                        moves.append(.plunge(to: z, at: ring[0], feed: plungeRate))

                        for i in 1..<ring.count {
                            moves.append(.linear(
                                to: Vector3D(ring[i], z: z),
                                feed: feedRate
                            ))
                        }

                        // Close the ring
                        moves.append(.linear(
                            to: Vector3D(ring[0], z: z),
                            feed: feedRate
                        ))

                        moves.append(.rapid(to: Vector3D(ring[0], z: safeZ)))
                    }
                }
            }
        }

        return moves
    }

    // MARK: - Depth Pass Calculation

    /// Compute incremental depth passes from zero to the total depth.
    ///
    /// Splits a deep cut into multiple shallower passes, each no deeper than `depthPerPass`.
    /// The final pass always reaches exactly `totalDepth`.
    ///
    /// - Parameters:
    ///   - totalDepth: The total target depth.
    ///   - depthPerPass: Maximum depth increment per pass.
    /// - Returns: An array of cumulative depths for each pass, ending with `totalDepth`.
    private static func computeDepthPasses(totalDepth: Double, depthPerPass: Double) -> [Double] {
        guard totalDepth > 0 && depthPerPass > 0 else { return [totalDepth] }

        var passes: [Double] = []
        var currentDepth = depthPerPass

        while currentDepth < totalDepth {
            passes.append(currentDepth)
            currentDepth += depthPerPass
        }

        passes.append(totalDepth)
        return passes
    }
}
