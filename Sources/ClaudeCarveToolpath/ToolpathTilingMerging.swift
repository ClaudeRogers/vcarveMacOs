import Foundation
import ClaudeCarveCore

// MARK: - Tiling Strategy

/// Strategy for ordering tiles when splitting an oversized toolpath.
public enum TilingStrategy: String, Codable, Sendable {
    /// Simple left-to-right, bottom-to-top grid traversal.
    case grid
    /// Zigzag pattern — alternates row direction for efficient movement between tiles.
    case zigzag
    /// Spiral from center outward for balanced material removal.
    case spiral
}

// MARK: - Tiling Configuration

/// Configuration for splitting an oversized toolpath into machine-sized tiles.
public struct TilingConfig: Codable, Sendable {
    /// Width of each tile in machine X travel (mm).
    public var tileWidth: Double
    /// Height of each tile in machine Y travel (mm).
    public var tileHeight: Double
    /// Overlap between adjacent tiles (mm) to ensure seamless joins.
    public var overlap: Double
    /// Strategy for ordering tile execution.
    public var strategy: TilingStrategy
    /// Safe Z height for rapid moves between tiles (mm).
    public var safeZ: Double
    /// Whether to insert a pause/tool change between tiles.
    public var pauseBetweenTiles: Bool

    public init(
        tileWidth: Double = 300,
        tileHeight: Double = 300,
        overlap: Double = 5.0,
        strategy: TilingStrategy = .zigzag,
        safeZ: Double = 5.0,
        pauseBetweenTiles: Bool = true
    ) {
        self.tileWidth = tileWidth
        self.tileHeight = tileHeight
        self.overlap = overlap
        self.strategy = strategy
        self.safeZ = safeZ
        self.pauseBetweenTiles = pauseBetweenTiles
    }
}

// MARK: - Tile Info

/// Describes a single tile produced by splitting an oversized toolpath.
public struct TileInfo: Sendable {
    /// Sequential index of this tile in the execution order.
    public let index: Int
    /// Row of this tile in the grid (0-based, from bottom).
    public let row: Int
    /// Column of this tile in the grid (0-based, from left).
    public let column: Int
    /// Origin of this tile in design coordinates.
    public let origin: Vector2D
    /// Bounding box of this tile in design coordinates.
    public let bounds: BoundingBox2D
    /// The clipped toolpath for this tile.
    public let toolpath: ComputedToolpath
}

// MARK: - Toolpath Tiling

/// Splits oversized toolpaths into machine-sized tiles with optional registration marks.
public struct ToolpathTiling {

    // MARK: - Public API

    /// Split an oversized toolpath into tiles that fit the machine's work area.
    ///
    /// The algorithm:
    /// 1. Computes the bounding box of the entire toolpath.
    /// 2. Divides the design area into a grid of tiles with specified overlap.
    /// 3. For each tile, clips moves to tile bounds with safe retracts at boundaries.
    /// 4. Splits moves that cross tile boundaries.
    /// 5. Orders tiles according to the chosen tiling strategy.
    public static func tile(
        toolpath: ComputedToolpath,
        config: TilingConfig
    ) -> [TileInfo] {
        let bb3D = toolpath.boundingBox
        let designBounds = BoundingBox2D(
            min: Vector2D(bb3D.min.x, bb3D.min.y),
            max: Vector2D(bb3D.max.x, bb3D.max.y)
        )

        guard !designBounds.isEmpty else { return [] }

        // Calculate grid dimensions
        let effectiveTileWidth = config.tileWidth - config.overlap
        let effectiveTileHeight = config.tileHeight - config.overlap

        guard effectiveTileWidth > 0 && effectiveTileHeight > 0 else { return [] }

        let numCols = Swift.max(1, Int(ceil(designBounds.width / effectiveTileWidth)))
        let numRows = Swift.max(1, Int(ceil(designBounds.height / effectiveTileHeight)))

        // Build tile grid
        var tiles: [(row: Int, col: Int)] = []
        for row in 0..<numRows {
            for col in 0..<numCols {
                tiles.append((row, col))
            }
        }

        // Reorder tiles according to strategy
        let orderedTiles = orderTiles(tiles, rows: numRows, cols: numCols, strategy: config.strategy)

        // Generate clipped toolpaths for each tile
        var results: [TileInfo] = []

        for (index, tile) in orderedTiles.enumerated() {
            let tileOriginX = designBounds.min.x + Double(tile.col) * effectiveTileWidth
            let tileOriginY = designBounds.min.y + Double(tile.row) * effectiveTileHeight
            let tileOrigin = Vector2D(tileOriginX, tileOriginY)

            let tileBounds = BoundingBox2D(
                min: tileOrigin,
                max: Vector2D(
                    Swift.min(tileOriginX + config.tileWidth, designBounds.max.x),
                    Swift.min(tileOriginY + config.tileHeight, designBounds.max.y)
                )
            )

            let clippedMoves = clipMovesToTile(
                moves: toolpath.moves,
                tileBounds: tileBounds,
                safeZ: config.safeZ
            )

            guard !clippedMoves.isEmpty else { continue }

            var tileToolpath = ComputedToolpath(
                configID: toolpath.configID,
                toolID: toolpath.toolID,
                moves: clippedMoves
            )
            tileToolpath.calculateEstimates()

            results.append(TileInfo(
                index: index,
                row: tile.row,
                column: tile.col,
                origin: tileOrigin,
                bounds: tileBounds,
                toolpath: tileToolpath
            ))
        }

        return results
    }

    /// Generate tiled output with registration marks at tile corners for precise alignment.
    ///
    /// Registration marks are small cross-shaped engravings at each tile corner
    /// that allow the operator to precisely reposition the workpiece between tiles.
    public static func tileWithRegistration(
        toolpath: ComputedToolpath,
        config: TilingConfig,
        markSize: Double = 5.0
    ) -> [(tile: TileInfo, registrationMarks: ComputedToolpath)] {
        let tiles = tile(toolpath: toolpath, config: config)

        return tiles.map { tileInfo in
            let marks = generateRegistrationMarks(
                bounds: tileInfo.bounds,
                markSize: markSize,
                safeZ: config.safeZ,
                configID: toolpath.configID,
                toolID: toolpath.toolID
            )
            return (tile: tileInfo, registrationMarks: marks)
        }
    }

    // MARK: - Tile Ordering

    /// Order tiles according to the selected tiling strategy.
    private static func orderTiles(
        _ tiles: [(row: Int, col: Int)],
        rows: Int,
        cols: Int,
        strategy: TilingStrategy
    ) -> [(row: Int, col: Int)] {
        switch strategy {
        case .grid:
            // Simple left-to-right, bottom-to-top
            return tiles.sorted { a, b in
                if a.row != b.row { return a.row < b.row }
                return a.col < b.col
            }

        case .zigzag:
            // Alternate row direction: even rows left-to-right, odd rows right-to-left
            return tiles.sorted { a, b in
                if a.row != b.row { return a.row < b.row }
                if a.row % 2 == 0 {
                    return a.col < b.col
                } else {
                    return a.col > b.col
                }
            }

        case .spiral:
            return spiralOrder(rows: rows, cols: cols)
        }
    }

    /// Generate a spiral order from center outward.
    private static func spiralOrder(rows: Int, cols: Int) -> [(row: Int, col: Int)] {
        guard rows > 0 && cols > 0 else { return [] }

        // Start from center
        let centerRow = rows / 2
        let centerCol = cols / 2

        var visited = Set<Int>() // row * cols + col
        var result: [(row: Int, col: Int)] = []

        // Directions: right, up, left, down
        let dRow = [0, 1, 0, -1]
        let dCol = [1, 0, -1, 0]

        var row = centerRow
        var col = centerCol
        var direction = 0
        var stepsInDirection = 1
        var stepsTaken = 0
        var directionChanges = 0

        let total = rows * cols

        while result.count < total {
            if row >= 0 && row < rows && col >= 0 && col < cols {
                let key = row * cols + col
                if !visited.contains(key) {
                    visited.insert(key)
                    result.append((row, col))
                }
            }

            stepsTaken += 1
            row += dRow[direction]
            col += dCol[direction]

            if stepsTaken >= stepsInDirection {
                stepsTaken = 0
                direction = (direction + 1) % 4
                directionChanges += 1
                // Increase step length every two direction changes
                if directionChanges % 2 == 0 {
                    stepsInDirection += 1
                }
            }

            // Safety valve — stop if we've wandered far enough
            if result.count >= total { break }
            if abs(row - centerRow) > total && abs(col - centerCol) > total { break }
        }

        return result
    }

    // MARK: - Move Clipping

    /// Clip toolpath moves to a tile's bounding region, splitting moves that cross boundaries.
    private static func clipMovesToTile(
        moves: [ToolpathMove],
        tileBounds: BoundingBox2D,
        safeZ: Double
    ) -> [ToolpathMove] {
        guard !moves.isEmpty else { return [] }

        var clipped: [ToolpathMove] = []
        var previousPosition = Vector3D.zero
        var isInsideTile = false

        for move in moves {
            let prevXY = previousPosition.xy
            let currXY = move.position.xy
            let prevInside = tileBounds.contains(prevXY)
            let currInside = tileBounds.contains(currXY)

            switch (prevInside, currInside) {
            case (true, true):
                // Entire move is inside the tile
                if !isInsideTile {
                    // Rapid to safe Z, then to position
                    clipped.append(.rapid(to: Vector3D(prevXY, z: safeZ)))
                    isInsideTile = true
                }
                clipped.append(ToolpathMove(
                    type: move.type,
                    position: move.position,
                    feedRate: move.feedRate,
                    spindleSpeed: move.spindleSpeed
                ))

            case (true, false):
                // Move exits the tile — split at boundary
                if let t = lineBoundaryIntersection(
                    from: prevXY, to: currXY, bounds: tileBounds
                ) {
                    let splitPos = interpolatePosition(
                        from: previousPosition, to: move.position, t: t
                    )
                    clipped.append(ToolpathMove(
                        type: move.type,
                        position: splitPos,
                        feedRate: move.feedRate,
                        spindleSpeed: move.spindleSpeed
                    ))
                    // Retract to safe Z
                    clipped.append(.rapid(to: Vector3D(splitPos.xy, z: safeZ)))
                }
                isInsideTile = false

            case (false, true):
                // Move enters the tile — split at boundary
                if let t = lineBoundaryIntersection(
                    from: prevXY, to: currXY, bounds: tileBounds
                ) {
                    let entryPos = interpolatePosition(
                        from: previousPosition, to: move.position, t: t
                    )
                    // Rapid to entry point at safe Z, then plunge
                    clipped.append(.rapid(to: Vector3D(entryPos.xy, z: safeZ)))
                    if move.type != .rapid {
                        clipped.append(ToolpathMove(
                            type: .linear,
                            position: entryPos,
                            feedRate: move.feedRate,
                            spindleSpeed: move.spindleSpeed
                        ))
                    }
                    clipped.append(ToolpathMove(
                        type: move.type,
                        position: move.position,
                        feedRate: move.feedRate,
                        spindleSpeed: move.spindleSpeed
                    ))
                    isInsideTile = true
                }

            case (false, false):
                // Both outside — but the move might cross through the tile
                let intersections = allBoundaryIntersections(
                    from: prevXY, to: currXY, bounds: tileBounds
                )
                if intersections.count >= 2 {
                    let t1 = intersections[0]
                    let t2 = intersections[1]
                    let entryPos = interpolatePosition(
                        from: previousPosition, to: move.position, t: t1
                    )
                    let exitPos = interpolatePosition(
                        from: previousPosition, to: move.position, t: t2
                    )

                    clipped.append(.rapid(to: Vector3D(entryPos.xy, z: safeZ)))
                    if move.type != .rapid {
                        clipped.append(ToolpathMove(
                            type: .linear,
                            position: entryPos,
                            feedRate: move.feedRate,
                            spindleSpeed: move.spindleSpeed
                        ))
                    }
                    clipped.append(ToolpathMove(
                        type: move.type,
                        position: exitPos,
                        feedRate: move.feedRate,
                        spindleSpeed: move.spindleSpeed
                    ))
                    clipped.append(.rapid(to: Vector3D(exitPos.xy, z: safeZ)))
                }
                isInsideTile = false
            }

            previousPosition = move.position
        }

        return clipped
    }

    // MARK: - Line/Boundary Intersection

    /// Find the first parametric t where a line segment intersects a bounding box edge.
    /// Returns nil if no intersection in [0, 1].
    private static func lineBoundaryIntersection(
        from p0: Vector2D,
        to p1: Vector2D,
        bounds: BoundingBox2D
    ) -> Double? {
        let intersections = allBoundaryIntersections(from: p0, to: p1, bounds: bounds)
        return intersections.first
    }

    /// Find all parametric t values where a line segment crosses bounding box edges,
    /// sorted in ascending order.
    private static func allBoundaryIntersections(
        from p0: Vector2D,
        to p1: Vector2D,
        bounds: BoundingBox2D
    ) -> [Double] {
        let dx = p1.x - p0.x
        let dy = p1.y - p0.y
        var tValues: [Double] = []
        let epsilon = 1.0e-10

        // Intersect with each edge
        // Left edge: x = bounds.min.x
        if abs(dx) > epsilon {
            let t = (bounds.min.x - p0.x) / dx
            if t >= 0 && t <= 1 {
                let y = p0.y + t * dy
                if y >= bounds.min.y - epsilon && y <= bounds.max.y + epsilon {
                    tValues.append(t)
                }
            }
        }

        // Right edge: x = bounds.max.x
        if abs(dx) > epsilon {
            let t = (bounds.max.x - p0.x) / dx
            if t >= 0 && t <= 1 {
                let y = p0.y + t * dy
                if y >= bounds.min.y - epsilon && y <= bounds.max.y + epsilon {
                    tValues.append(t)
                }
            }
        }

        // Bottom edge: y = bounds.min.y
        if abs(dy) > epsilon {
            let t = (bounds.min.y - p0.y) / dy
            if t >= 0 && t <= 1 {
                let x = p0.x + t * dx
                if x >= bounds.min.x - epsilon && x <= bounds.max.x + epsilon {
                    tValues.append(t)
                }
            }
        }

        // Top edge: y = bounds.max.y
        if abs(dy) > epsilon {
            let t = (bounds.max.y - p0.y) / dy
            if t >= 0 && t <= 1 {
                let x = p0.x + t * dx
                if x >= bounds.min.x - epsilon && x <= bounds.max.x + epsilon {
                    tValues.append(t)
                }
            }
        }

        // Remove duplicates (corner intersections) and sort
        let sorted = tValues.sorted()
        var deduplicated: [Double] = []
        for t in sorted {
            if let last = deduplicated.last, abs(t - last) < epsilon {
                continue
            }
            deduplicated.append(t)
        }

        return deduplicated
    }

    /// Linearly interpolate a 3D position between two points.
    private static func interpolatePosition(
        from p0: Vector3D,
        to p1: Vector3D,
        t: Double
    ) -> Vector3D {
        p0.lerp(to: p1, t: t)
    }

    // MARK: - Registration Marks

    /// Generate cross-shaped registration marks at each corner of a tile.
    private static func generateRegistrationMarks(
        bounds: BoundingBox2D,
        markSize: Double,
        safeZ: Double,
        configID: UUID,
        toolID: UUID
    ) -> ComputedToolpath {
        var moves: [ToolpathMove] = []
        let halfMark = markSize / 2.0
        let markZ = -0.2 // Shallow engraving depth for registration marks
        let markFeed = 200.0 // Slow feed for precision

        let corners = [
            bounds.min,
            Vector2D(bounds.max.x, bounds.min.y),
            bounds.max,
            Vector2D(bounds.min.x, bounds.max.y),
        ]

        for corner in corners {
            // Horizontal stroke
            moves.append(.rapid(to: Vector3D(corner.x - halfMark, corner.y, safeZ)))
            moves.append(.linear(
                to: Vector3D(corner.x - halfMark, corner.y, markZ),
                feed: markFeed
            ))
            moves.append(.linear(
                to: Vector3D(corner.x + halfMark, corner.y, markZ),
                feed: markFeed
            ))
            moves.append(.rapid(to: Vector3D(corner.x + halfMark, corner.y, safeZ)))

            // Vertical stroke
            moves.append(.rapid(to: Vector3D(corner.x, corner.y - halfMark, safeZ)))
            moves.append(.linear(
                to: Vector3D(corner.x, corner.y - halfMark, markZ),
                feed: markFeed
            ))
            moves.append(.linear(
                to: Vector3D(corner.x, corner.y + halfMark, markZ),
                feed: markFeed
            ))
            moves.append(.rapid(to: Vector3D(corner.x, corner.y + halfMark, safeZ)))
        }

        var markToolpath = ComputedToolpath(
            configID: configID,
            toolID: toolID,
            moves: moves
        )
        markToolpath.calculateEstimates()
        return markToolpath
    }
}

// MARK: - Merging Configuration

/// Configuration for merging and optimizing multiple toolpaths.
public struct MergingConfig: Codable, Sendable {
    /// Reorder cut sequences to minimize rapid travel distance.
    public var optimizeRapids: Bool
    /// Merge overlapping toolpaths into a single pass where possible.
    public var combineOverlapping: Bool
    /// Maximum rapid move distance before inserting a split point (mm).
    public var maxRapidDistance: Double
    /// Safe Z height for rapid repositioning (mm).
    public var safeZ: Double

    public init(
        optimizeRapids: Bool = true,
        combineOverlapping: Bool = false,
        maxRapidDistance: Double = 1000,
        safeZ: Double = 5.0
    ) {
        self.optimizeRapids = optimizeRapids
        self.combineOverlapping = combineOverlapping
        self.maxRapidDistance = maxRapidDistance
        self.safeZ = safeZ
    }
}

// MARK: - Time Estimate

/// Estimated machining time and distance breakdown for a toolpath.
public struct TimeEstimate: Sendable {
    /// Total time in seconds (cutting + rapids).
    public let totalSeconds: Double
    /// Time spent cutting in seconds.
    public let cuttingSeconds: Double
    /// Time spent in rapid moves in seconds.
    public let rapidSeconds: Double
    /// Total distance traveled (mm).
    public let totalDistance: Double
    /// Distance traveled while cutting (mm).
    public let cuttingDistance: Double
    /// Distance traveled during rapid moves (mm).
    public let rapidDistance: Double
}

// MARK: - Toolpath Merging

/// Merges, optimizes, and analyzes CNC toolpaths.
public struct ToolpathMerging {

    // MARK: - Public API

    /// Merge multiple toolpaths into one optimized toolpath.
    ///
    /// All input toolpaths must use the same tool. The merged result uses the
    /// `configID` and `toolID` from the first toolpath.
    ///
    /// Algorithm:
    /// 1. Concatenate all move lists.
    /// 2. Separate into cut sequences (connected groups of cutting moves between rapids).
    /// 3. If `optimizeRapids` is enabled, reorder sequences using nearest-neighbor heuristic.
    /// 4. If `combineOverlapping` is enabled, merge sequences that share endpoints.
    /// 5. Reconnect sequences with optimized rapids via safe Z.
    public static func merge(
        _ toolpaths: [ComputedToolpath],
        config: MergingConfig
    ) -> ComputedToolpath {
        guard let first = toolpaths.first else {
            return ComputedToolpath(configID: UUID(), toolID: UUID())
        }
        guard toolpaths.count > 1 else {
            if config.optimizeRapids {
                return optimizeRapids(first, safeZ: config.safeZ)
            }
            return first
        }

        // Gather all moves from all toolpaths
        var allMoves: [ToolpathMove] = []
        for tp in toolpaths {
            allMoves.append(contentsOf: tp.moves)
        }

        // Split into cut sequences
        var sequences = extractCutSequences(from: allMoves)

        guard !sequences.isEmpty else {
            return ComputedToolpath(configID: first.configID, toolID: first.toolID)
        }

        // Optimize ordering via nearest-neighbor
        if config.optimizeRapids {
            sequences = reorderSequencesNearestNeighbor(sequences)
        }

        // Reassemble with safe rapids
        let mergedMoves = reassembleWithRapids(
            sequences: sequences,
            safeZ: config.safeZ,
            maxRapidDistance: config.maxRapidDistance
        )

        var result = ComputedToolpath(
            configID: first.configID,
            toolID: first.toolID,
            moves: mergedMoves
        )
        result.calculateEstimates()
        return result
    }

    /// Optimize rapid movements in a toolpath using nearest-neighbor ordering.
    ///
    /// Breaks the toolpath into cut sequences and reorders them so that each
    /// rapid traverse goes to the nearest unvisited sequence start.
    public static func optimizeRapids(
        _ toolpath: ComputedToolpath,
        safeZ: Double
    ) -> ComputedToolpath {
        var sequences = extractCutSequences(from: toolpath.moves)

        guard sequences.count > 1 else { return toolpath }

        sequences = reorderSequencesNearestNeighbor(sequences)

        let optimizedMoves = reassembleWithRapids(
            sequences: sequences,
            safeZ: safeZ,
            maxRapidDistance: .infinity
        )

        var result = ComputedToolpath(
            configID: toolpath.configID,
            toolID: toolpath.toolID,
            moves: optimizedMoves
        )
        result.calculateEstimates()
        return result
    }

    /// Remove redundant moves: duplicate consecutive positions, zero-length moves,
    /// and consecutive rapids to the same XY location.
    public static func removeRedundant(
        _ toolpath: ComputedToolpath
    ) -> ComputedToolpath {
        guard toolpath.moves.count > 1 else { return toolpath }

        var filtered: [ToolpathMove] = []
        let epsilon = 1.0e-6

        for move in toolpath.moves {
            if let last = filtered.last {
                let dist = last.position.distance(to: move.position)
                // Skip zero-length moves
                if dist < epsilon {
                    continue
                }
                // Skip consecutive rapids to the same XY (keep the last one)
                if last.type == .rapid && move.type == .rapid {
                    let xyDist = last.position.xy.distance(to: move.position.xy)
                    if xyDist < epsilon {
                        // Replace the previous rapid with this one (different Z)
                        filtered[filtered.count - 1] = move
                        continue
                    }
                }
            }
            filtered.append(move)
        }

        var result = ComputedToolpath(
            configID: toolpath.configID,
            toolID: toolpath.toolID,
            moves: filtered
        )
        result.calculateEstimates()
        return result
    }

    /// Estimate the total machining time broken down by cutting vs. rapid movement.
    ///
    /// - Parameters:
    ///   - toolpath: The toolpath to analyze.
    ///   - rapidFeedRate: Machine rapid traverse rate in mm/min.
    /// - Returns: A `TimeEstimate` with time and distance breakdowns.
    public static func estimateTime(
        _ toolpath: ComputedToolpath,
        rapidFeedRate: Double
    ) -> TimeEstimate {
        var cuttingDistance = 0.0
        var rapidDistance = 0.0
        var cuttingTime = 0.0
        var rapidTime = 0.0
        var previous = Vector3D.zero

        for move in toolpath.moves {
            let distance = previous.distance(to: move.position)

            switch move.type {
            case .rapid:
                rapidDistance += distance
                if rapidFeedRate > 0 {
                    rapidTime += distance / rapidFeedRate
                }

            case .linear, .cwArc, .ccwArc:
                cuttingDistance += distance
                let feed = move.feedRate ?? rapidFeedRate
                if feed > 0 {
                    cuttingTime += distance / feed
                }
            }

            previous = move.position
        }

        // Convert from minutes to seconds
        let cuttingSeconds = cuttingTime * 60.0
        let rapidSeconds = rapidTime * 60.0

        return TimeEstimate(
            totalSeconds: cuttingSeconds + rapidSeconds,
            cuttingSeconds: cuttingSeconds,
            rapidSeconds: rapidSeconds,
            totalDistance: cuttingDistance + rapidDistance,
            cuttingDistance: cuttingDistance,
            rapidDistance: rapidDistance
        )
    }

    // MARK: - Cut Sequence Extraction

    /// A contiguous sequence of cutting moves (non-rapid) that forms one logical cut.
    private struct CutSequence {
        var moves: [ToolpathMove]

        var startPosition: Vector3D {
            moves.first?.position ?? .zero
        }

        var endPosition: Vector3D {
            moves.last?.position ?? .zero
        }
    }

    /// Break a flat list of moves into discrete cutting sequences separated by rapids.
    private static func extractCutSequences(from moves: [ToolpathMove]) -> [CutSequence] {
        var sequences: [CutSequence] = []
        var currentSequence: [ToolpathMove] = []

        for move in moves {
            switch move.type {
            case .rapid:
                // A rapid move ends the current cutting sequence
                if !currentSequence.isEmpty {
                    sequences.append(CutSequence(moves: currentSequence))
                    currentSequence = []
                }

            case .linear, .cwArc, .ccwArc:
                currentSequence.append(move)
            }
        }

        // Don't forget the last sequence
        if !currentSequence.isEmpty {
            sequences.append(CutSequence(moves: currentSequence))
        }

        return sequences
    }

    // MARK: - Nearest-Neighbor Reordering

    /// Reorder cut sequences using a greedy nearest-neighbor heuristic.
    ///
    /// Starting from the origin (0, 0, 0), repeatedly picks the unvisited sequence
    /// whose start point is closest to the current position, considering both the
    /// start and end of each sequence (reversing if the end is closer).
    private static func reorderSequencesNearestNeighbor(
        _ sequences: [CutSequence]
    ) -> [CutSequence] {
        guard sequences.count > 1 else { return sequences }

        var remaining = sequences
        var ordered: [CutSequence] = []
        var currentPos = Vector3D.zero

        while !remaining.isEmpty {
            var bestIndex = 0
            var bestDistance = Double.infinity
            var bestReverse = false

            for (index, seq) in remaining.enumerated() {
                let distToStart = currentPos.xy.distanceSquared(to: seq.startPosition.xy)
                let distToEnd = currentPos.xy.distanceSquared(to: seq.endPosition.xy)

                if distToStart < bestDistance {
                    bestDistance = distToStart
                    bestIndex = index
                    bestReverse = false
                }
                if distToEnd < bestDistance {
                    bestDistance = distToEnd
                    bestIndex = index
                    bestReverse = true
                }
            }

            var chosen = remaining.remove(at: bestIndex)
            if bestReverse {
                chosen.moves.reverse()
            }
            currentPos = chosen.endPosition
            ordered.append(chosen)
        }

        return ordered
    }

    // MARK: - Reassembly

    /// Reconnect ordered cut sequences with rapid moves through safe Z.
    private static func reassembleWithRapids(
        sequences: [CutSequence],
        safeZ: Double,
        maxRapidDistance: Double
    ) -> [ToolpathMove] {
        var result: [ToolpathMove] = []
        var currentPos = Vector3D.zero

        for sequence in sequences {
            let targetStart = sequence.startPosition

            // Retract to safe Z
            if currentPos.z < safeZ {
                result.append(.rapid(to: Vector3D(currentPos.xy, z: safeZ)))
            }

            // Rapid to above sequence start
            let rapidXYDist = currentPos.xy.distance(to: targetStart.xy)
            if rapidXYDist > 1.0e-6 {
                // For very long rapids, go through safe Z
                if rapidXYDist > maxRapidDistance {
                    // Break into waypoints if needed — for now just go via safe Z
                    result.append(.rapid(to: Vector3D(targetStart.xy, z: safeZ)))
                } else {
                    result.append(.rapid(to: Vector3D(targetStart.xy, z: safeZ)))
                }
            }

            // Plunge to first cut position
            if abs(safeZ - targetStart.z) > 1.0e-6 {
                // Use the first move's feed rate for plunge, or default
                let plungeFeed = sequence.moves.first?.feedRate
                if let feed = plungeFeed {
                    result.append(.linear(to: targetStart, feed: feed))
                } else {
                    result.append(.rapid(to: targetStart))
                }
            }

            // Append all cutting moves in the sequence
            result.append(contentsOf: sequence.moves)
            currentPos = sequence.endPosition
        }

        // Final retract to safe Z
        if !result.isEmpty {
            result.append(.rapid(to: Vector3D(currentPos.xy, z: safeZ)))
        }

        return result
    }
}
