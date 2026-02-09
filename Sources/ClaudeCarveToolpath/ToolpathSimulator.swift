import Foundation
import ClaudeCarveCore

// MARK: - Heightfield

/// A 2D grid of height values representing the material surface after CNC machining.
/// Each cell stores the current top-of-material Z height at that grid point.
/// Heights start at `materialThickness` (uncut surface) and decrease as material is removed.
public struct Heightfield: Sendable {
    /// Grid data stored in row-major order: index = y * width + x
    private var data: [Double]

    /// Number of columns in the grid.
    public let width: Int

    /// Number of rows in the grid.
    public let height: Int

    /// Real-world distance between adjacent grid points.
    public let cellSize: Double

    /// Real-world position of the grid corner (min X, min Y).
    public let origin: Vector2D

    /// The initial material surface height (Z = 0 for surface-referenced, Z = thickness for bed-referenced).
    public let surfaceHeight: Double

    /// Initialize a heightfield covering the given material dimensions.
    ///
    /// - Parameters:
    ///   - materialWidth: The X extent of the material in real-world units.
    ///   - materialHeight: The Y extent of the material in real-world units.
    ///   - materialThickness: The Z thickness of the material (initial height).
    ///   - resolution: The real-world distance between grid sample points.
    ///                 Smaller values produce finer simulation at higher memory cost.
    public init(
        materialWidth: Double,
        materialHeight: Double,
        materialThickness: Double,
        resolution: Double
    ) {
        precondition(resolution > 0, "Resolution must be positive")
        precondition(materialWidth > 0, "Material width must be positive")
        precondition(materialHeight > 0, "Material height must be positive")

        self.cellSize = resolution
        self.width = Swift.max(1, Int(ceil(materialWidth / resolution)) + 1)
        self.height = Swift.max(1, Int(ceil(materialHeight / resolution)) + 1)
        self.origin = .zero
        self.surfaceHeight = materialThickness
        self.data = [Double](repeating: materialThickness, count: width * height)
    }

    /// Initialize a heightfield with explicit origin for non-bottom-left datum positions.
    public init(
        materialWidth: Double,
        materialHeight: Double,
        materialThickness: Double,
        resolution: Double,
        origin: Vector2D
    ) {
        precondition(resolution > 0, "Resolution must be positive")
        precondition(materialWidth > 0, "Material width must be positive")
        precondition(materialHeight > 0, "Material height must be positive")

        self.cellSize = resolution
        self.width = Swift.max(1, Int(ceil(materialWidth / resolution)) + 1)
        self.height = Swift.max(1, Int(ceil(materialHeight / resolution)) + 1)
        self.origin = origin
        self.surfaceHeight = materialThickness
        self.data = [Double](repeating: materialThickness, count: width * height)
    }

    // MARK: - Grid Access

    /// Returns the height value at the given grid coordinates.
    /// Returns `surfaceHeight` for out-of-bounds coordinates.
    public func heightAt(x: Int, y: Int) -> Double {
        guard x >= 0, x < width, y >= 0, y < height else {
            return surfaceHeight
        }
        return data[y * width + x]
    }

    /// Sets the height at the given grid coordinates. Out-of-bounds writes are ignored.
    public mutating func setHeight(x: Int, y: Int, _ h: Double) {
        guard x >= 0, x < width, y >= 0, y < height else { return }
        data[y * width + x] = h
    }

    /// Lowers the height at the given grid coordinates to the minimum of the
    /// current value and the provided value. This models material removal:
    /// the tool can only remove material, never add it.
    @inline(__always)
    mutating func carve(x: Int, y: Int, to z: Double) {
        guard x >= 0, x < width, y >= 0, y < height else { return }
        let index = y * width + x
        if z < data[index] {
            data[index] = z
        }
    }

    // MARK: - Coordinate Conversion

    /// Converts a real-world 2D point to grid coordinates.
    /// The returned indices are clamped to valid grid bounds.
    public func worldToGrid(_ point: Vector2D) -> (Int, Int) {
        let gx = Int(round((point.x - origin.x) / cellSize))
        let gy = Int(round((point.y - origin.y) / cellSize))
        return (
            Swift.max(0, Swift.min(width - 1, gx)),
            Swift.max(0, Swift.min(height - 1, gy))
        )
    }

    /// Converts grid coordinates to the real-world 2D position of that grid point.
    public func gridToWorld(x: Int, y: Int) -> Vector2D {
        Vector2D(
            origin.x + Double(x) * cellSize,
            origin.y + Double(y) * cellSize
        )
    }

    /// Unclamped grid coordinates (may be out of bounds). Used internally
    /// so tool-carving loops can do their own bounds iteration.
    func worldToGridUnclamped(_ point: Vector2D) -> (Double, Double) {
        let gx = (point.x - origin.x) / cellSize
        let gy = (point.y - origin.y) / cellSize
        return (gx, gy)
    }
}

// MARK: - Simulation Statistics

/// Summary statistics computed from a simulation run.
public struct SimulationStats: Sendable {
    /// Total volume of material removed, in cubic units.
    public var materialRemoved: Double

    /// Maximum depth below the material surface reached by the tool.
    public var maxDepth: Double

    /// Estimated machining time in seconds, computed from move distances and feed rates.
    public var estimatedTime: TimeInterval

    public init(materialRemoved: Double = 0, maxDepth: Double = 0, estimatedTime: TimeInterval = 0) {
        self.materialRemoved = materialRemoved
        self.maxDepth = maxDepth
        self.estimatedTime = estimatedTime
    }
}

// MARK: - ToolpathSimulator

/// Simulates CNC material removal by sweeping a tool profile along a computed toolpath
/// and recording the resulting surface in a heightfield grid.
public struct ToolpathSimulator: Sendable {

    // MARK: - Public API

    /// Simulates the given toolpath and returns the resulting heightfield.
    ///
    /// - Parameters:
    ///   - toolpath: The computed toolpath containing all moves to simulate.
    ///   - tool: The cutting tool whose profile determines the shape of material removal.
    ///   - material: The material setup defining dimensions, thickness, and Z-zero reference.
    ///   - resolution: Grid cell size for the simulation. Smaller values increase accuracy
    ///                 but require more memory and computation time.
    /// - Returns: A heightfield representing the material surface after all cuts.
    public static func simulate(
        toolpath: ComputedToolpath,
        tool: Tool,
        material: MaterialSetup,
        resolution: Double
    ) -> Heightfield {
        let bounds = material.bounds
        let surfaceZ = surfaceZForMaterial(material)

        var heightfield = Heightfield(
            materialWidth: bounds.width,
            materialHeight: bounds.height,
            materialThickness: surfaceZ,
            resolution: resolution,
            origin: bounds.min
        )

        guard !toolpath.moves.isEmpty else { return heightfield }

        let profile = ToolProfile(tool: tool, material: material)

        // Track the current tool position. Start at the origin on the surface.
        var currentPos = Vector3D(bounds.min, z: material.safeZHeight)

        for move in toolpath.moves {
            let targetPos = move.position

            switch move.type {
            case .rapid:
                // Rapids move without cutting. However, if for some reason the
                // rapid dips below the surface we still carve (safety check).
                simulateLinearMove(
                    from: currentPos,
                    to: targetPos,
                    profile: profile,
                    heightfield: &heightfield
                )

            case .linear:
                simulateLinearMove(
                    from: currentPos,
                    to: targetPos,
                    profile: profile,
                    heightfield: &heightfield
                )

            case .cwArc, .ccwArc:
                // Approximate arcs as a series of linear segments.
                let segments = linearizeArc(
                    from: currentPos,
                    to: targetPos,
                    center: move.arcCenter,
                    clockwise: move.type == .cwArc
                )
                var segStart = currentPos
                for segEnd in segments {
                    simulateLinearMove(
                        from: segStart,
                        to: segEnd,
                        profile: profile,
                        heightfield: &heightfield
                    )
                    segStart = segEnd
                }
            }

            currentPos = targetPos
        }

        return heightfield
    }

    /// Computes summary statistics by comparing a carved heightfield against the original
    /// uncut material state.
    ///
    /// - Parameters:
    ///   - heightfield: The heightfield after simulation.
    ///   - toolpath: The toolpath that was simulated (used for time estimation).
    ///   - tool: The tool used (provides default feed rates for rapids).
    ///   - material: The material setup.
    /// - Returns: Statistics including volume removed, max depth, and estimated time.
    public static func computeStats(
        heightfield: Heightfield,
        toolpath: ComputedToolpath,
        tool: Tool,
        material: MaterialSetup
    ) -> SimulationStats {
        let surfaceZ = surfaceZForMaterial(material)
        let cellArea = heightfield.cellSize * heightfield.cellSize

        var volumeRemoved = 0.0
        var maxDepthBelow = 0.0

        for gy in 0..<heightfield.height {
            for gx in 0..<heightfield.width {
                let h = heightfield.heightAt(x: gx, y: gy)
                if h < surfaceZ {
                    let depthRemoved = surfaceZ - h
                    volumeRemoved += depthRemoved * cellArea
                    if depthRemoved > maxDepthBelow {
                        maxDepthBelow = depthRemoved
                    }
                }
            }
        }

        // Estimate machining time from move distances and feed rates.
        let rapidRate = 5000.0 // mm/min default rapid rate
        var totalTime = 0.0 // accumulated in minutes
        var prevPos = Vector3D.zero

        for move in toolpath.moves {
            let dist = prevPos.distance(to: move.position)
            let rate: Double
            switch move.type {
            case .rapid:
                rate = rapidRate
            case .linear, .cwArc, .ccwArc:
                rate = move.feedRate ?? tool.feedRate
            }
            if rate > 0 {
                totalTime += dist / rate
            }
            prevPos = move.position
        }

        return SimulationStats(
            materialRemoved: volumeRemoved,
            maxDepth: maxDepthBelow,
            estimatedTime: totalTime * 60.0 // convert minutes to seconds
        )
    }

    // MARK: - Internal: Linear Move Simulation

    /// Simulates a single linear move from `start` to `end`, sampling the tool profile
    /// at evenly-spaced points along the segment and carving the heightfield.
    private static func simulateLinearMove(
        from start: Vector3D,
        to end: Vector3D,
        profile: ToolProfile,
        heightfield: inout Heightfield
    ) {
        let delta = end - start
        let moveLength = delta.length

        // Determine step count based on the grid resolution for adequate sampling.
        // At minimum, carve at both endpoints. For longer moves, space samples
        // at roughly half the cell size for smooth results.
        let stepSize = heightfield.cellSize * 0.5
        let steps: Int
        if moveLength < 1e-9 {
            steps = 1
        } else {
            steps = Swift.max(1, Int(ceil(moveLength / stepSize)))
        }

        for i in 0...steps {
            let t = Double(i) / Double(steps)
            let pos = start.lerp(to: end, t: t)

            // Only carve when the tool tip is at or below the material surface.
            // This avoids unnecessary work for moves entirely above the material.
            if pos.z < heightfield.surfaceHeight {
                profile.carve(at: pos, into: &heightfield)
            }
        }
    }

    // MARK: - Internal: Arc Linearization

    /// Approximates a circular arc as a sequence of linear segments.
    ///
    /// - Parameters:
    ///   - from: Start position of the arc.
    ///   - to: End position of the arc.
    ///   - center: Arc center given as an offset (I, J) from the start point.
    ///   - clockwise: True for G2 (CW), false for G3 (CCW).
    /// - Returns: Array of intermediate and final positions forming the linearized arc.
    private static func linearizeArc(
        from start: Vector3D,
        to end: Vector3D,
        center: Vector2D?,
        clockwise: Bool
    ) -> [Vector3D] {
        guard let arcCenter = center else {
            // If no center is provided, fall back to a straight line.
            return [end]
        }

        // Arc center in absolute coordinates (I/J are relative to start).
        let cx = start.x + arcCenter.x
        let cy = start.y + arcCenter.y

        let r = sqrt((start.x - cx) * (start.x - cx) + (start.y - cy) * (start.y - cy))
        guard r > 1e-9 else { return [end] }

        let startAngle = atan2(start.y - cy, start.x - cx)
        var endAngle = atan2(end.y - cy, end.x - cx)

        // Determine sweep direction.
        if clockwise {
            // CW: angle decreases. If endAngle >= startAngle, subtract 2*pi.
            if endAngle >= startAngle {
                endAngle -= 2.0 * .pi
            }
        } else {
            // CCW: angle increases. If endAngle <= startAngle, add 2*pi.
            if endAngle <= startAngle {
                endAngle += 2.0 * .pi
            }
        }

        let sweep = abs(endAngle - startAngle)
        // Approximate: use enough segments that each subtends at most ~5 degrees.
        let segmentCount = Swift.max(4, Int(ceil(sweep / (5.0 * .pi / 180.0))))
        let zDelta = end.z - start.z

        var result: [Vector3D] = []
        result.reserveCapacity(segmentCount)

        for i in 1...segmentCount {
            let t = Double(i) / Double(segmentCount)
            let angle = startAngle + (endAngle - startAngle) * t
            let px = cx + r * cos(angle)
            let py = cy + r * sin(angle)
            let pz = start.z + zDelta * t
            result.append(Vector3D(px, py, pz))
        }

        return result
    }

    // MARK: - Internal: Z Reference Helpers

    /// Returns the Z height of the material surface in the job coordinate system.
    /// When Z-zero is at the material surface, the surface is at Z = 0 and cuts go negative.
    /// When Z-zero is at the machine bed, the surface is at Z = thickness.
    private static func surfaceZForMaterial(_ material: MaterialSetup) -> Double {
        switch material.zZeroPosition {
        case .materialSurface:
            return 0.0
        case .machineBed:
            return material.thickness
        }
    }
}

// MARK: - Tool Profile

/// Encapsulates the 2D radial profile of a tool and the logic for stamping (carving)
/// that profile into a heightfield at a given 3D position.
///
/// For each tool type, the profile determines how deep the tool cuts at a given
/// radial distance from the tool axis center.
struct ToolProfile: Sendable {
    /// The tool being modeled.
    let tool: Tool

    /// Precomputed tool radius for efficiency.
    let toolRadius: Double

    /// The grid-space radius: the number of cells the tool spans from center.
    /// Used to bound the carving loop.
    let gridRadius: Int

    /// Half angle in radians for V-bit tools. Zero for non-V-bits.
    let halfAngle: Double

    /// The Z height of the uncut material surface.
    let surfaceZ: Double

    init(tool: Tool, material: MaterialSetup) {
        self.tool = tool
        self.toolRadius = tool.radius
        self.halfAngle = tool.halfAngleRadians

        switch material.zZeroPosition {
        case .materialSurface:
            self.surfaceZ = 0.0
        case .machineBed:
            self.surfaceZ = material.thickness
        }

        // gridRadius is set later when carving (depends on heightfield cellSize),
        // but we store a reasonable default here.
        self.gridRadius = 0
    }

    /// Carves the tool profile into the heightfield centered at the given world position.
    ///
    /// For each grid cell within the tool's radius, computes the Z height the tool
    /// would cut to at that radial distance and applies it if lower than the current
    /// heightfield value.
    func carve(at position: Vector3D, into heightfield: inout Heightfield) {
        let toolTipZ = position.z
        let centerXY = position.xy

        // Compute the effective cutting radius at this Z depth.
        // For V-bits, the cutting width grows with depth from surface.
        let effectiveRadius = effectiveCuttingRadius(at: toolTipZ)
        guard effectiveRadius > 0 else { return }

        // Convert center to grid coordinates (unclamped for iteration bounds).
        let (gcx, gcy) = heightfield.worldToGridUnclamped(centerXY)

        let gridRad = Int(ceil(effectiveRadius / heightfield.cellSize)) + 1

        let minGX = Swift.max(0, Int(floor(gcx)) - gridRad)
        let maxGX = Swift.min(heightfield.width - 1, Int(ceil(gcx)) + gridRad)
        let minGY = Swift.max(0, Int(floor(gcy)) - gridRad)
        let maxGY = Swift.min(heightfield.height - 1, Int(ceil(gcy)) + gridRad)

        let radiusSq = effectiveRadius * effectiveRadius

        for gy in minGY...maxGY {
            for gx in minGX...maxGX {
                let worldPt = heightfield.gridToWorld(x: gx, y: gy)
                let dx = worldPt.x - centerXY.x
                let dy = worldPt.y - centerXY.y
                let distSq = dx * dx + dy * dy

                guard distSq <= radiusSq else { continue }

                let dist = sqrt(distSq)
                let cutZ = toolSurfaceZ(radialDistance: dist, toolTipZ: toolTipZ)

                // Only carve below the material surface; never raise material.
                if cutZ < heightfield.surfaceHeight {
                    heightfield.carve(x: gx, y: gy, to: cutZ)
                }
            }
        }
    }

    // MARK: - Tool Profile Functions

    /// Returns the effective lateral cutting radius of the tool at the current
    /// Z position. For most tools this is simply the tool radius. For V-bits,
    /// the cutting width depends on how deep the tool has plunged below the surface.
    private func effectiveCuttingRadius(at tipZ: Double) -> Double {
        switch tool.type {
        case .vBit, .engraving, .chamfer:
            // V-bit cutting width grows linearly with depth below surface.
            let depth = surfaceZ - tipZ
            guard depth > 0 else { return 0 }
            let vRadius = depth * tan(halfAngle)
            // Clamp to the physical tool diameter.
            return Swift.min(vRadius, toolRadius)

        default:
            return toolRadius
        }
    }

    /// Returns the Z height of the tool surface at a given radial distance from the
    /// tool axis, when the tool tip is at `toolTipZ`.
    ///
    /// This is the core profile function. For each tool type:
    /// - **End mill**: Flat bottom. The tool cuts to `toolTipZ` everywhere within the radius.
    /// - **Ball nose**: Hemispherical bottom. Z rises as a sphere from the center.
    /// - **V-bit**: Conical bottom. Z rises linearly from the center proportional to
    ///   `radialDistance / tan(halfAngle)`.
    /// - **Bull nose**: Flat center with rounded corners (end mill + corner radius).
    /// - **Drill**: Conical tip, typically 118-degree included angle.
    /// - **Other**: Treated as end mill (flat bottom).
    private func toolSurfaceZ(radialDistance dist: Double, toolTipZ tipZ: Double) -> Double {
        switch tool.type {
        case .endMill, .custom, .threadMill:
            // Flat bottom: uniform depth across the entire tool diameter.
            return tipZ

        case .ballNose, .taperedBallNose:
            // Hemispherical bottom: the tool surface follows a sphere of
            // radius R centered at (toolAxis, tipZ + R).
            // Z = centerZ - sqrt(R^2 - dist^2)
            //   = tipZ + R - sqrt(R^2 - dist^2)
            let r = toolRadius
            let rSq = r * r
            let dSq = dist * dist
            if dSq >= rSq { return tipZ + r } // outside tool; return top of sphere
            return tipZ + r - sqrt(rSq - dSq)

        case .vBit, .engraving, .chamfer:
            // Conical profile: Z rises linearly from the tip.
            // At the center (dist=0), Z = tipZ.
            // At distance d, Z = tipZ + d / tan(halfAngle).
            guard halfAngle > 1e-9 else { return tipZ }
            return tipZ + dist / tan(halfAngle)

        case .bullNose:
            // Flat center with rounded corners.
            // The flat region extends from center to (R - cornerRadius).
            // Beyond that, a quarter-circle of cornerRadius transitions to the shank.
            let cr = tool.cornerRadius
            let flatRadius = toolRadius - cr
            if dist <= flatRadius {
                return tipZ
            }
            // In the rounded corner region:
            let localDist = dist - flatRadius
            let crSq = cr * cr
            let ldSq = localDist * localDist
            if ldSq >= crSq { return tipZ + cr }
            return tipZ + cr - sqrt(crSq - ldSq)

        case .drill:
            // Standard drill point: 118-degree included angle (59-degree half-angle).
            let drillHalfAngle = 59.0 * .pi / 180.0
            return tipZ + dist / tan(drillHalfAngle)
        }
    }
}
