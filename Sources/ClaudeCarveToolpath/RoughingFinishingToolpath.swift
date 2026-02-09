import Foundation
import ClaudeCarveCore
import ClaudeCarveGeometry

// MARK: - 3D Roughing Strategy

/// Strategy for 3D roughing passes.
public enum RoughingStrategy: String, Codable, Sendable {
    case rasterX        // Parallel passes along X axis
    case rasterY        // Parallel passes along Y axis
    case offsetFromBoundary  // Concentric inward from boundary
    case adaptive       // Adaptive clearing (maintains constant engagement)
}

/// Configuration for 3D roughing toolpath.
public struct RoughingConfig: Codable, Sendable {
    public var strategy: RoughingStrategy
    public var stockToLeave: Double      // Material left for finishing pass
    public var depthPerPass: Double      // Maximum Z step per pass
    public var stepover: Double          // XY distance between passes (fraction of tool diameter)
    public var rampAngle: Double         // Maximum ramp-in angle in degrees
    public var useRamping: Bool          // Ramp instead of plunge
    public var cutDirection: CutDirection
    public var restMachining: Bool       // Only cut remaining material from previous larger tool

    public init(
        strategy: RoughingStrategy = .rasterX,
        stockToLeave: Double = 0.5,
        depthPerPass: Double = 3.0,
        stepover: Double = 0.4,
        rampAngle: Double = 5.0,
        useRamping: Bool = true,
        cutDirection: CutDirection = .climb,
        restMachining: Bool = false
    ) {
        self.strategy = strategy
        self.stockToLeave = stockToLeave
        self.depthPerPass = depthPerPass
        self.stepover = stepover
        self.rampAngle = rampAngle
        self.useRamping = useRamping
        self.cutDirection = cutDirection
        self.restMachining = restMachining
    }
}

/// Cut direction for toolpath passes.
public enum CutDirection: String, Codable, Sendable {
    case climb       // Tool moves in same direction as rotation at cutting edge
    case conventional // Tool moves against rotation direction
    case mixed       // Zigzag (alternating)
}

// MARK: - 3D Finishing Strategy

/// Strategy for 3D finishing passes.
public enum FinishingStrategy: String, Codable, Sendable {
    case rasterX          // Parallel passes along X
    case rasterY          // Parallel passes along Y
    case raster45         // Parallel passes at 45°
    case raster135        // Parallel passes at 135°
    case spiral           // Spiral from outside in
    case waterline        // Constant-Z contour lines
    case pencil           // Follow concave edges (internal corners)
}

/// Configuration for 3D finishing toolpath.
public struct FinishingConfig: Codable, Sendable {
    public var strategy: FinishingStrategy
    public var stepover: Double          // XY distance between passes (fraction of tool diameter)
    public var tolerance: Double         // Surface quality tolerance
    public var cutDirection: CutDirection
    public var scallop: Double           // Maximum scallop height

    public init(
        strategy: FinishingStrategy = .rasterX,
        stepover: Double = 0.1,
        tolerance: Double = 0.01,
        cutDirection: CutDirection = .climb,
        scallop: Double = 0.02
    ) {
        self.strategy = strategy
        self.stepover = stepover
        self.tolerance = tolerance
        self.cutDirection = cutDirection
        self.scallop = scallop
    }
}

// MARK: - 3D Surface Model

/// A 3D surface represented as a height map (Z values on a grid).
/// This is used as the target shape for 3D machining.
public struct SurfaceModel: Sendable {
    public let width: Int
    public let height: Int
    public let cellSize: Double
    public let origin: Vector2D
    private var heights: [Double]  // Row-major

    public init(width: Int, height: Int, cellSize: Double, origin: Vector2D = .zero, defaultHeight: Double = 0) {
        self.width = width
        self.height = height
        self.cellSize = cellSize
        self.origin = origin
        self.heights = Array(repeating: defaultHeight, count: width * height)
    }

    public func heightAt(x: Int, y: Int) -> Double {
        guard x >= 0, x < width, y >= 0, y < height else { return 0 }
        return heights[y * width + x]
    }

    public mutating func setHeight(x: Int, y: Int, _ h: Double) {
        guard x >= 0, x < width, y >= 0, y < height else { return }
        heights[y * width + x] = h
    }

    /// Bilinear interpolation of height at a world-space position.
    public func heightAt(worldPos: Vector2D) -> Double {
        let localX = (worldPos.x - origin.x) / cellSize
        let localY = (worldPos.y - origin.y) / cellSize

        let ix = Int(floor(localX))
        let iy = Int(floor(localY))
        let fx = localX - Double(ix)
        let fy = localY - Double(iy)

        let h00 = heightAt(x: ix, y: iy)
        let h10 = heightAt(x: ix + 1, y: iy)
        let h01 = heightAt(x: ix, y: iy + 1)
        let h11 = heightAt(x: ix + 1, y: iy + 1)

        let h0 = h00 + (h10 - h00) * fx
        let h1 = h01 + (h11 - h01) * fx
        return h0 + (h1 - h0) * fy
    }

    /// Create a flat surface at a given Z height.
    public static func flat(materialWidth: Double, materialHeight: Double, z: Double, resolution: Double) -> SurfaceModel {
        let w = max(1, Int(materialWidth / resolution))
        let h = max(1, Int(materialHeight / resolution))
        return SurfaceModel(width: w, height: h, cellSize: resolution, defaultHeight: z)
    }

    /// Create a hemisphere surface (useful for testing).
    public static func hemisphere(materialWidth: Double, materialHeight: Double, radius: Double, resolution: Double) -> SurfaceModel {
        let w = max(1, Int(materialWidth / resolution))
        let h = max(1, Int(materialHeight / resolution))
        var surface = SurfaceModel(width: w, height: h, cellSize: resolution)
        let center = Vector2D(materialWidth / 2, materialHeight / 2)

        for iy in 0..<h {
            for ix in 0..<w {
                let pos = Vector2D(Double(ix) * resolution, Double(iy) * resolution)
                let dist = pos.distance(to: center)
                if dist < radius {
                    let z = -sqrt(radius * radius - dist * dist) // Concave hemisphere (carved into material)
                    surface.setHeight(x: ix, y: iy, z)
                }
            }
        }
        return surface
    }
}

// MARK: - 3D Roughing Toolpath Generator

/// Generates 3D roughing toolpaths that clear bulk material in layers.
public struct RoughingToolpathGenerator {

    /// Generate a 3D roughing toolpath for a target surface.
    public static func generate(
        surface: SurfaceModel,
        boundary: [Vector2D],
        config: RoughingConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let stepoverDist = tool.diameter * config.stepover
        let stockOffset = config.stockToLeave

        // Compute the depth range
        var minZ = 0.0
        for iy in 0..<surface.height {
            for ix in 0..<surface.width {
                minZ = min(minZ, surface.heightAt(x: ix, y: iy))
            }
        }
        let targetMinZ = minZ - stockOffset

        // Generate depth passes from top to bottom
        var currentZ = -config.depthPerPass
        let safeZ = material.safeZHeight

        // Start at safe height
        moves.append(.rapid(to: Vector3D(0, 0, safeZ)))

        while currentZ >= targetMinZ {
            let passZ = max(currentZ, targetMinZ)

            switch config.strategy {
            case .rasterX:
                let passMoves = generateRasterPass(
                    surface: surface,
                    boundary: boundary,
                    passZ: passZ,
                    stockOffset: stockOffset,
                    stepover: stepoverDist,
                    tool: tool,
                    material: material,
                    alongX: true,
                    direction: config.cutDirection
                )
                moves.append(contentsOf: passMoves)

            case .rasterY:
                let passMoves = generateRasterPass(
                    surface: surface,
                    boundary: boundary,
                    passZ: passZ,
                    stockOffset: stockOffset,
                    stepover: stepoverDist,
                    tool: tool,
                    material: material,
                    alongX: false,
                    direction: config.cutDirection
                )
                moves.append(contentsOf: passMoves)

            case .offsetFromBoundary:
                let passMoves = generateOffsetPass(
                    boundary: boundary,
                    passZ: passZ,
                    stepover: stepoverDist,
                    tool: tool,
                    material: material
                )
                moves.append(contentsOf: passMoves)

            case .adaptive:
                // Adaptive uses raster with engagement limiting
                let passMoves = generateAdaptivePass(
                    surface: surface,
                    boundary: boundary,
                    passZ: passZ,
                    stockOffset: stockOffset,
                    stepover: stepoverDist,
                    tool: tool,
                    material: material
                )
                moves.append(contentsOf: passMoves)
            }

            // Retract between depth passes
            moves.append(.rapid(to: Vector3D(moves.last?.position.x ?? 0, moves.last?.position.y ?? 0, safeZ)))

            currentZ -= config.depthPerPass
        }

        // Return home
        moves.append(.rapid(to: Vector3D(0, 0, safeZ)))

        var toolpath = ComputedToolpath(configID: UUID(), toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Raster Pass

    private static func generateRasterPass(
        surface: SurfaceModel,
        boundary: [Vector2D],
        passZ: Double,
        stockOffset: Double,
        stepover: Double,
        tool: Tool,
        material: MaterialSetup,
        alongX: Bool,
        direction: CutDirection
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let bounds = material.bounds

        let primaryStart: Double
        let primaryEnd: Double
        let secondaryStart: Double
        let secondaryEnd: Double

        if alongX {
            primaryStart = bounds.min.y + tool.radius
            primaryEnd = bounds.max.y - tool.radius
            secondaryStart = bounds.min.x + tool.radius
            secondaryEnd = bounds.max.x - tool.radius
        } else {
            primaryStart = bounds.min.x + tool.radius
            primaryEnd = bounds.max.x - tool.radius
            secondaryStart = bounds.min.y + tool.radius
            secondaryEnd = bounds.max.y - tool.radius
        }

        var primaryPos = primaryStart
        var forward = true
        var passIndex = 0

        while primaryPos <= primaryEnd {
            let lineStart: Vector2D
            let lineEnd: Vector2D

            let s = forward ? secondaryStart : secondaryEnd
            let e = forward ? secondaryEnd : secondaryStart

            if alongX {
                lineStart = Vector2D(s, primaryPos)
                lineEnd = Vector2D(e, primaryPos)
            } else {
                lineStart = Vector2D(primaryPos, s)
                lineEnd = Vector2D(primaryPos, e)
            }

            // Rapid to start of line
            moves.append(.rapid(to: Vector3D(lineStart.x, lineStart.y, safeZ)))

            // Plunge to cutting depth (considering surface)
            let startSurfaceZ = surface.heightAt(worldPos: lineStart) + stockOffset
            let cutZ = max(passZ, startSurfaceZ)
            moves.append(.linear(to: Vector3D(lineStart.x, lineStart.y, cutZ), feed: tool.plungeRate))

            // Cut along the line, adjusting Z to follow surface + stock allowance
            let numSteps = max(1, Int(lineStart.distance(to: lineEnd) / (surface.cellSize * 2)))
            for step in 1...numSteps {
                let t = Double(step) / Double(numSteps)
                let pos = lineStart.lerp(to: lineEnd, t: t)
                let surfaceZ = surface.heightAt(worldPos: pos) + stockOffset
                let z = max(passZ, surfaceZ)
                moves.append(.linear(to: Vector3D(pos.x, pos.y, z), feed: tool.feedRate))
            }

            // Retract
            moves.append(.rapid(to: Vector3D(lineEnd.x, lineEnd.y, safeZ)))

            // Advance to next line
            primaryPos += stepover
            passIndex += 1

            if direction == .mixed {
                forward = !forward
            }
        }

        return moves
    }

    // MARK: - Offset Pass

    private static func generateOffsetPass(
        boundary: [Vector2D],
        passZ: Double,
        stepover: Double,
        tool: Tool,
        material: MaterialSetup
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        // Generate concentric offsets
        let offsets = PolygonOffset.concentricOffsets(polygon: boundary, stepover: stepover, inward: true)

        for contour in offsets {
            guard !contour.isEmpty else { continue }

            // Rapid to first point
            moves.append(.rapid(to: Vector3D(contour[0].x, contour[0].y, safeZ)))
            moves.append(.linear(to: Vector3D(contour[0].x, contour[0].y, passZ), feed: tool.plungeRate))

            // Follow contour
            for i in 1..<contour.count {
                moves.append(.linear(to: Vector3D(contour[i].x, contour[i].y, passZ), feed: tool.feedRate))
            }

            // Close contour
            moves.append(.linear(to: Vector3D(contour[0].x, contour[0].y, passZ), feed: tool.feedRate))

            // Retract
            moves.append(.rapid(to: Vector3D(contour[0].x, contour[0].y, safeZ)))
        }

        return moves
    }

    // MARK: - Adaptive Pass

    private static func generateAdaptivePass(
        surface: SurfaceModel,
        boundary: [Vector2D],
        passZ: Double,
        stockOffset: Double,
        stepover: Double,
        tool: Tool,
        material: MaterialSetup
    ) -> [ToolpathMove] {
        // Adaptive clearing: limit engagement angle to reduce tool load
        // Uses trochoidal-like motion when encountering full-width cuts
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let bounds = material.bounds
        let maxEngagement = 0.5 * tool.diameter  // Maximum radial engagement

        var y = bounds.min.y + tool.radius
        var forward = true

        while y <= bounds.max.y - tool.radius {
            let xStart = forward ? bounds.min.x + tool.radius : bounds.max.x - tool.radius
            let xEnd = forward ? bounds.max.x - tool.radius : bounds.min.x + tool.radius
            let xStep = forward ? surface.cellSize * 2 : -surface.cellSize * 2

            moves.append(.rapid(to: Vector3D(xStart, y, safeZ)))

            let surfaceZ = surface.heightAt(worldPos: Vector2D(xStart, y)) + stockOffset
            let cutZ = max(passZ, surfaceZ)
            moves.append(.linear(to: Vector3D(xStart, y, cutZ), feed: tool.plungeRate))

            var x = xStart
            while forward ? (x <= xEnd) : (x >= xEnd) {
                let pos = Vector2D(x, y)
                let surfZ = surface.heightAt(worldPos: pos) + stockOffset
                let z = max(passZ, surfZ)

                // Check if we need trochoidal motion (full engagement)
                let engagementWidth = stepover
                if engagementWidth > maxEngagement && z < -0.1 {
                    // Add a small trochoidal loop to reduce engagement
                    let loopRadius = stepover * 0.4
                    let cx = x
                    let cy = y
                    let steps = 8
                    for s in 0...steps {
                        let angle = Double(s) / Double(steps) * 2.0 * .pi
                        let lx = cx + loopRadius * cos(angle)
                        let ly = cy + loopRadius * sin(angle)
                        moves.append(.linear(to: Vector3D(lx, ly, z), feed: tool.feedRate * 0.7))
                    }
                }

                moves.append(.linear(to: Vector3D(x, y, z), feed: tool.feedRate))
                x += xStep
            }

            moves.append(.rapid(to: Vector3D(x, y, safeZ)))

            y += stepover
            forward = !forward
        }

        return moves
    }
}

// MARK: - 3D Finishing Toolpath Generator

/// Generates 3D finishing toolpaths for final surface quality.
public struct FinishingToolpathGenerator {

    /// Generate a 3D finishing toolpath for a target surface.
    public static func generate(
        surface: SurfaceModel,
        boundary: [Vector2D],
        config: FinishingConfig,
        tool: Tool,
        material: MaterialSetup
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        moves.append(.rapid(to: Vector3D(0, 0, safeZ)))

        switch config.strategy {
        case .rasterX:
            moves.append(contentsOf: generateRasterFinish(
                surface: surface, tool: tool, material: material,
                stepover: tool.diameter * config.stepover, angle: 0))

        case .rasterY:
            moves.append(contentsOf: generateRasterFinish(
                surface: surface, tool: tool, material: material,
                stepover: tool.diameter * config.stepover, angle: .pi / 2))

        case .raster45:
            moves.append(contentsOf: generateRasterFinish(
                surface: surface, tool: tool, material: material,
                stepover: tool.diameter * config.stepover, angle: .pi / 4))

        case .raster135:
            moves.append(contentsOf: generateRasterFinish(
                surface: surface, tool: tool, material: material,
                stepover: tool.diameter * config.stepover, angle: 3 * .pi / 4))

        case .spiral:
            moves.append(contentsOf: generateSpiralFinish(
                surface: surface, boundary: boundary, tool: tool, material: material,
                stepover: tool.diameter * config.stepover))

        case .waterline:
            moves.append(contentsOf: generateWaterlineFinish(
                surface: surface, tool: tool, material: material,
                stepInterval: tool.diameter * config.stepover))

        case .pencil:
            moves.append(contentsOf: generatePencilFinish(
                surface: surface, tool: tool, material: material,
                tolerance: config.tolerance))
        }

        moves.append(.rapid(to: Vector3D(0, 0, safeZ)))

        var toolpath = ComputedToolpath(configID: UUID(), toolID: tool.id, moves: moves)
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Raster Finish (at arbitrary angle)

    private static func generateRasterFinish(
        surface: SurfaceModel,
        tool: Tool,
        material: MaterialSetup,
        stepover: Double,
        angle: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight
        let bounds = material.bounds

        // For angled raster, we rotate the scan direction
        let cosA = cos(angle)
        let sinA = sin(angle)

        // Compute rotated bounds extents
        let corners = [
            bounds.min,
            Vector2D(bounds.max.x, bounds.min.y),
            bounds.max,
            Vector2D(bounds.min.x, bounds.max.y),
        ]

        // Project corners onto the perpendicular axis to find scan range
        var minPerp = Double.infinity
        var maxPerp = -Double.infinity
        var minAlong = Double.infinity
        var maxAlong = -Double.infinity

        for corner in corners {
            let along = corner.x * cosA + corner.y * sinA
            let perp = -corner.x * sinA + corner.y * cosA
            minPerp = min(minPerp, perp)
            maxPerp = max(maxPerp, perp)
            minAlong = min(minAlong, along)
            maxAlong = max(maxAlong, along)
        }

        var perpPos = minPerp + tool.radius
        var forward = true

        while perpPos <= maxPerp - tool.radius {
            let aStart = forward ? minAlong + tool.radius : maxAlong - tool.radius
            let aEnd = forward ? maxAlong - tool.radius : minAlong + tool.radius
            let numSteps = max(1, Int(abs(aEnd - aStart) / (surface.cellSize)))

            // First point
            let startX = aStart * cosA - perpPos * sinA
            let startY = aStart * sinA + perpPos * cosA
            let startPos = Vector2D(startX, startY)

            // Check if start is within bounds
            if bounds.contains(startPos) {
                moves.append(.rapid(to: Vector3D(startX, startY, safeZ)))

                let startZ = computeToolZ(surface: surface, tool: tool, at: startPos)
                moves.append(.linear(to: Vector3D(startX, startY, startZ), feed: tool.plungeRate))

                for step in 1...numSteps {
                    let t = Double(step) / Double(numSteps)
                    let along = aStart + (aEnd - aStart) * t
                    let x = along * cosA - perpPos * sinA
                    let y = along * sinA + perpPos * cosA
                    let pos = Vector2D(x, y)

                    if bounds.contains(pos) {
                        let z = computeToolZ(surface: surface, tool: tool, at: pos)
                        moves.append(.linear(to: Vector3D(x, y, z), feed: tool.feedRate))
                    }
                }

                let lastPos = moves.last?.position ?? Vector3D.zero
                moves.append(.rapid(to: Vector3D(lastPos.x, lastPos.y, safeZ)))
            }

            perpPos += stepover
            forward = !forward
        }

        return moves
    }

    // MARK: - Spiral Finish

    private static func generateSpiralFinish(
        surface: SurfaceModel,
        boundary: [Vector2D],
        tool: Tool,
        material: MaterialSetup,
        stepover: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        // Generate spiral from outside in using offset contours
        let offsets = PolygonOffset.concentricOffsets(polygon: boundary, stepover: stepover, inward: true)

        for contour in offsets {
            guard contour.count >= 2 else { continue }

            let start = contour[0]
            moves.append(.rapid(to: Vector3D(start.x, start.y, safeZ)))

            let startZ = surface.heightAt(worldPos: start)
            moves.append(.linear(to: Vector3D(start.x, start.y, startZ), feed: tool.plungeRate))

            for i in 1..<contour.count {
                let pos = contour[i]
                let z = computeToolZ(surface: surface, tool: tool, at: pos)
                moves.append(.linear(to: Vector3D(pos.x, pos.y, z), feed: tool.feedRate))
            }

            // Close
            let z = computeToolZ(surface: surface, tool: tool, at: start)
            moves.append(.linear(to: Vector3D(start.x, start.y, z), feed: tool.feedRate))
            moves.append(.rapid(to: Vector3D(start.x, start.y, safeZ)))
        }

        return moves
    }

    // MARK: - Waterline Finish

    private static func generateWaterlineFinish(
        surface: SurfaceModel,
        tool: Tool,
        material: MaterialSetup,
        stepInterval: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        // Find Z range
        var minZ = 0.0
        for iy in 0..<surface.height {
            for ix in 0..<surface.width {
                minZ = min(minZ, surface.heightAt(x: ix, y: iy))
            }
        }

        // Generate contours at each Z level
        var z = -stepInterval
        while z >= minZ {
            let contours = extractContours(surface: surface, z: z, tool: tool)

            for contour in contours {
                guard contour.count >= 2 else { continue }

                moves.append(.rapid(to: Vector3D(contour[0].x, contour[0].y, safeZ)))
                moves.append(.linear(to: Vector3D(contour[0].x, contour[0].y, z), feed: tool.plungeRate))

                for i in 1..<contour.count {
                    moves.append(.linear(to: Vector3D(contour[i].x, contour[i].y, z), feed: tool.feedRate))
                }

                // Close contour
                moves.append(.linear(to: Vector3D(contour[0].x, contour[0].y, z), feed: tool.feedRate))
                moves.append(.rapid(to: Vector3D(contour[0].x, contour[0].y, safeZ)))
            }

            z -= stepInterval
        }

        return moves
    }

    /// Extract iso-height contours from the surface using marching squares.
    private static func extractContours(surface: SurfaceModel, z: Double, tool: Tool) -> [[Vector2D]] {
        var contours: [[Vector2D]] = []
        var visited = Array(repeating: Array(repeating: false, count: surface.width), count: surface.height)

        for iy in 0..<(surface.height - 1) {
            for ix in 0..<(surface.width - 1) {
                if visited[iy][ix] { continue }

                let h00 = surface.heightAt(x: ix, y: iy)
                let h10 = surface.heightAt(x: ix + 1, y: iy)
                let h01 = surface.heightAt(x: ix, y: iy + 1)
                let h11 = surface.heightAt(x: ix + 1, y: iy + 1)

                // Check if this cell crosses the iso-level
                let below00 = h00 < z
                let below10 = h10 < z
                let below01 = h01 < z
                let below11 = h11 < z

                if below00 != below10 || below00 != below01 || below00 != below11 {
                    // This cell has a contour crossing
                    visited[iy][ix] = true
                    var contour: [Vector2D] = []

                    // Simple contour point: interpolate along edges
                    let worldX = surface.origin.x + Double(ix) * surface.cellSize
                    let worldY = surface.origin.y + Double(iy) * surface.cellSize

                    if below00 != below10 {
                        let t = (z - h00) / (h10 - h00)
                        contour.append(Vector2D(worldX + t * surface.cellSize, worldY))
                    }
                    if below10 != below11 {
                        let t = (z - h10) / (h11 - h10)
                        contour.append(Vector2D(worldX + surface.cellSize, worldY + t * surface.cellSize))
                    }
                    if below01 != below11 {
                        let t = (z - h01) / (h11 - h01)
                        contour.append(Vector2D(worldX + t * surface.cellSize, worldY + surface.cellSize))
                    }
                    if below00 != below01 {
                        let t = (z - h00) / (h01 - h00)
                        contour.append(Vector2D(worldX, worldY + t * surface.cellSize))
                    }

                    if contour.count >= 2 {
                        contours.append(contour)
                    }
                }
            }
        }

        // Merge connected contour segments
        return mergeContourSegments(contours)
    }

    /// Merge short contour segments into longer chains.
    private static func mergeContourSegments(_ segments: [[Vector2D]]) -> [[Vector2D]] {
        guard !segments.isEmpty else { return [] }

        var chains: [[Vector2D]] = []
        var used = Set<Int>()
        let tolerance = 1e-6

        for i in 0..<segments.count {
            if used.contains(i) { continue }
            used.insert(i)

            var chain = segments[i]
            var changed = true

            while changed {
                changed = false
                for j in 0..<segments.count {
                    if used.contains(j) { continue }

                    let seg = segments[j]
                    guard let chainLast = chain.last, let segFirst = seg.first, let segLast = seg.last else { continue }

                    if chainLast.distance(to: segFirst) < tolerance {
                        chain.append(contentsOf: seg.dropFirst())
                        used.insert(j)
                        changed = true
                    } else if chainLast.distance(to: segLast) < tolerance {
                        chain.append(contentsOf: seg.reversed().dropFirst())
                        used.insert(j)
                        changed = true
                    }
                }
            }

            if chain.count >= 3 {
                chains.append(chain)
            }
        }

        return chains
    }

    // MARK: - Pencil Finish

    private static func generatePencilFinish(
        surface: SurfaceModel,
        tool: Tool,
        material: MaterialSetup,
        tolerance: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let safeZ = material.safeZHeight

        // Pencil finishing follows concave edges (valleys in the surface)
        // Detect concavities by looking at the Laplacian of the height field
        var concavePoints: [(Vector2D, Double)] = [] // (position, z)

        for iy in 1..<(surface.height - 1) {
            for ix in 1..<(surface.width - 1) {
                let h = surface.heightAt(x: ix, y: iy)
                let hN = surface.heightAt(x: ix, y: iy - 1)
                let hS = surface.heightAt(x: ix, y: iy + 1)
                let hE = surface.heightAt(x: ix + 1, y: iy)
                let hW = surface.heightAt(x: ix - 1, y: iy)

                // Laplacian (discrete)
                let laplacian = hN + hS + hE + hW - 4 * h

                // Negative Laplacian = concave region (valley)
                if laplacian < -tolerance {
                    let worldPos = Vector2D(
                        surface.origin.x + Double(ix) * surface.cellSize,
                        surface.origin.y + Double(iy) * surface.cellSize
                    )
                    concavePoints.append((worldPos, h))
                }
            }
        }

        // Sort concave points and trace paths through them
        // Simple approach: scan rows, connect adjacent concave points
        guard !concavePoints.isEmpty else { return moves }

        // Group by approximate Y coordinate
        let sortedPoints = concavePoints.sorted { $0.0.y < $1.0.y || ($0.0.y == $1.0.y && $0.0.x < $1.0.x) }

        var currentPath: [(Vector2D, Double)] = []
        var lastPos: Vector2D?

        for (pos, z) in sortedPoints {
            if let last = lastPos, pos.distance(to: last) < surface.cellSize * 2 {
                currentPath.append((pos, z))
            } else {
                // Emit current path
                if currentPath.count >= 3 {
                    let start = currentPath[0].0
                    moves.append(.rapid(to: Vector3D(start.x, start.y, safeZ)))
                    moves.append(.linear(to: Vector3D(start.x, start.y, currentPath[0].1), feed: tool.plungeRate))

                    for i in 1..<currentPath.count {
                        let (p, pz) = currentPath[i]
                        let toolZ = computeToolZ(surface: surface, tool: tool, at: p)
                        moves.append(.linear(to: Vector3D(p.x, p.y, min(pz, toolZ)), feed: tool.feedRate))
                    }

                    moves.append(.rapid(to: Vector3D(currentPath.last!.0.x, currentPath.last!.0.y, safeZ)))
                }
                currentPath = [(pos, z)]
            }
            lastPos = pos
        }

        // Emit final path
        if currentPath.count >= 3 {
            let start = currentPath[0].0
            moves.append(.rapid(to: Vector3D(start.x, start.y, safeZ)))
            moves.append(.linear(to: Vector3D(start.x, start.y, currentPath[0].1), feed: tool.plungeRate))

            for i in 1..<currentPath.count {
                let (p, pz) = currentPath[i]
                moves.append(.linear(to: Vector3D(p.x, p.y, pz), feed: tool.feedRate))
            }

            moves.append(.rapid(to: Vector3D(currentPath.last!.0.x, currentPath.last!.0.y, safeZ)))
        }

        return moves
    }

    // MARK: - Tool Z Computation

    /// Compute the Z height the tool center should be at for a given XY position,
    /// accounting for tool shape (ball nose drops into concavities differently than end mill).
    private static func computeToolZ(surface: SurfaceModel, tool: Tool, at pos: Vector2D) -> Double {
        let baseZ = surface.heightAt(worldPos: pos)

        switch tool.type {
        case .ballNose:
            // Ball nose: tool center is at surface + radius, but the contact point
            // is the lowest point of the hemisphere. Sample surrounding area.
            let r = tool.radius
            var maxZ = baseZ
            let steps = 8
            for i in 0..<steps {
                let angle = Double(i) / Double(steps) * 2 * .pi
                for ri in 1...3 {
                    let rr = r * Double(ri) / 3.0
                    let samplePos = pos + Vector2D(rr * cos(angle), rr * sin(angle))
                    let sampleZ = surface.heightAt(worldPos: samplePos)
                    // The tool center height needed so the ball nose touches this point
                    let dxy = rr
                    if dxy < r {
                        let zOffset = r - sqrt(r * r - dxy * dxy)
                        maxZ = max(maxZ, sampleZ + zOffset)
                    }
                }
            }
            return maxZ

        case .endMill, .bullNose:
            // End mill: check all points within tool radius for highest surface
            var maxZ = baseZ
            let r = tool.radius
            let steps = 8
            for i in 0..<steps {
                let angle = Double(i) / Double(steps) * 2 * .pi
                for ri in 1...3 {
                    let rr = r * Double(ri) / 3.0
                    let samplePos = pos + Vector2D(rr * cos(angle), rr * sin(angle))
                    maxZ = max(maxZ, surface.heightAt(worldPos: samplePos))
                }
            }
            return maxZ

        default:
            return baseZ
        }
    }
}
