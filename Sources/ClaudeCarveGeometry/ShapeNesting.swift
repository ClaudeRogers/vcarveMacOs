import Foundation
import ClaudeCarveCore

// MARK: - Data Types

/// Result of a nesting operation, describing where each shape was placed
/// on the material sheet and how efficiently the material is used.
public struct NestingResult: Sendable {
    /// The placement of each shape that was successfully nested.
    public let placements: [ShapePlacement]

    /// Material utilization ratio (0.0-1.0) representing the fraction
    /// of the material bounding region occupied by placed shapes.
    public let utilization: Double

    /// Axis-aligned bounding box enclosing all placed shapes.
    public let boundingBox: BoundingBox2D

    public init(placements: [ShapePlacement], utilization: Double, boundingBox: BoundingBox2D) {
        self.placements = placements
        self.utilization = utilization
        self.boundingBox = boundingBox
    }
}

/// Describes the placement of a single shape on the material sheet.
public struct ShapePlacement: Sendable {
    /// Index of the shape in the original input array.
    public let shapeIndex: Int

    /// Translation offset from the shape's original position.
    public let position: Vector2D

    /// Rotation angle applied to the shape, in radians.
    public let rotation: Double

    /// Whether the shape was mirrored (flipped horizontally).
    public let flipped: Bool

    public init(shapeIndex: Int, position: Vector2D, rotation: Double, flipped: Bool) {
        self.shapeIndex = shapeIndex
        self.position = position
        self.rotation = rotation
        self.flipped = flipped
    }
}

/// Configuration for the shape nesting algorithm.
public struct NestingConfig: Sendable {
    /// Width of the material sheet.
    public let materialWidth: Double

    /// Height of the material sheet.
    public let materialHeight: Double

    /// Minimum gap between shapes (and between shapes and material edges).
    public let spacing: Double

    /// Whether the algorithm is allowed to rotate shapes to find better fits.
    public let allowRotation: Bool

    /// Number of discrete rotation angles to try. 4 corresponds to 0/90/180/270 degrees.
    public let rotationSteps: Int

    /// Whether the algorithm is allowed to mirror (flip) shapes.
    public let allowFlip: Bool

    public init(
        materialWidth: Double,
        materialHeight: Double,
        spacing: Double,
        allowRotation: Bool = true,
        rotationSteps: Int = 4,
        allowFlip: Bool = false
    ) {
        self.materialWidth = materialWidth
        self.materialHeight = materialHeight
        self.spacing = spacing
        self.allowRotation = allowRotation
        self.rotationSteps = rotationSteps
        self.allowFlip = allowFlip
    }
}

// MARK: - Shape Nester

/// Places multiple cut shapes onto a material sheet to minimize waste.
///
/// Uses a Bottom-Left Fill (BLF) algorithm with rotation search:
/// 1. Shapes are sorted by bounding box area (largest first).
/// 2. For each shape, all allowed rotations (and optionally flips) are tried.
/// 3. For each candidate orientation, the lowest then leftmost valid position is found
///    using a sweep over candidate Y positions derived from already-placed shapes.
/// 4. The orientation producing the best (bottom-left-most) placement wins.
public struct ShapeNester: Sendable {

    /// Nest the given shapes onto a material sheet described by `config`.
    ///
    /// - Parameters:
    ///   - shapes: Array of polygons, each defined as an array of `Vector2D` vertices.
    ///   - config: Nesting configuration (material size, spacing, rotation options).
    /// - Returns: A `NestingResult` with placements, utilization, and overall bounding box.
    public static func nest(shapes: [[Vector2D]], config: NestingConfig) -> NestingResult {
        guard !shapes.isEmpty else {
            return NestingResult(
                placements: [],
                utilization: 0,
                boundingBox: BoundingBox2D(min: .zero, max: .zero)
            )
        }

        // Compute bounding box areas and sort indices largest-first.
        let indexed = shapes.enumerated().map { (index: $0.offset, shape: $0.element) }
        let sorted = indexed.sorted { a, b in
            let bbA = BoundingBox2D.enclosing(a.shape)
            let bbB = BoundingBox2D.enclosing(b.shape)
            return bbA.width * bbA.height > bbB.width * bbB.height
        }

        // Build the set of candidate rotation angles.
        let rotationAngles = buildRotationAngles(config: config)

        // Build the set of flip options.
        let flipOptions: [Bool] = config.allowFlip ? [false, true] : [false]

        // Effective material bounds (inset by spacing so shapes stay away from edges).
        let matMinX = config.spacing
        let matMinY = config.spacing
        let matMaxX = config.materialWidth - config.spacing
        let matMaxY = config.materialHeight - config.spacing

        guard matMaxX > matMinX && matMaxY > matMinY else {
            return NestingResult(
                placements: [],
                utilization: 0,
                boundingBox: BoundingBox2D(min: .zero, max: Vector2D(config.materialWidth, config.materialHeight))
            )
        }

        var placements: [ShapePlacement] = []
        // Track placed polygons (already translated/rotated to final positions) for overlap tests.
        var placedPolygons: [(polygon: [Vector2D], boundingBox: BoundingBox2D)] = []

        for entry in sorted {
            let shapeIndex = entry.index
            let baseShape = entry.shape
            guard baseShape.count >= 3 else { continue }

            var bestPlacement: ShapePlacement?
            var bestY = Double.infinity
            var bestX = Double.infinity

            for flip in flipOptions {
                let flippedShape = flip ? flipPolygon(baseShape) : baseShape

                for angle in rotationAngles {
                    let rotated = rotatePolygon(flippedShape, by: angle)

                    // Normalize so that the shape's bounding box minimum is at the origin.
                    let rBB = BoundingBox2D.enclosing(rotated)
                    let normalized = translatePolygon(rotated, by: Vector2D(-rBB.min.x, -rBB.min.y))
                    let normBB = BoundingBox2D.enclosing(normalized)

                    // The shape occupies [0, normBB.width] x [0, normBB.height].
                    // When placed at offset (ox, oy), its bounding box is
                    // [ox, ox + normBB.width] x [oy, oy + normBB.height].
                    let shapeW = normBB.width
                    let shapeH = normBB.height

                    // Shape must fit within the effective material bounds.
                    guard shapeW <= (matMaxX - matMinX) && shapeH <= (matMaxY - matMinY) else {
                        continue
                    }

                    // Find the bottom-left-most valid position.
                    if let (px, py) = findBottomLeftPosition(
                        normalized: normalized,
                        shapeWidth: shapeW,
                        shapeHeight: shapeH,
                        matMinX: matMinX,
                        matMinY: matMinY,
                        matMaxX: matMaxX,
                        matMaxY: matMaxY,
                        spacing: config.spacing,
                        placedPolygons: placedPolygons
                    ) {
                        if py < bestY || (py == bestY && px < bestX) {
                            bestY = py
                            bestX = px
                            // The final translation is (px, py) relative to the normalized shape,
                            // but we need position relative to the original shape.
                            // normalized = rotated - rBB.min, so placed = normalized + (px, py)
                            // = rotated - rBB.min + (px, py)
                            // The position offset from the original (pre-rotation/flip) shape:
                            bestPlacement = ShapePlacement(
                                shapeIndex: shapeIndex,
                                position: Vector2D(px - rBB.min.x, py - rBB.min.y),
                                rotation: angle,
                                flipped: flip
                            )
                        }
                    }
                }
            }

            if let placement = bestPlacement {
                placements.append(placement)
                // Build the final polygon for future overlap checks.
                let transformed = transformShape(
                    baseShape,
                    placement: placement
                )
                let tBB = BoundingBox2D.enclosing(transformed)
                placedPolygons.append((polygon: transformed, boundingBox: tBB))
            }
        }

        // Compute overall bounding box and utilization.
        let resultBB = computeOverallBoundingBox(placedPolygons: placedPolygons, config: config)
        let totalShapeArea = computeTotalShapeArea(shapes: shapes, placements: placements)
        let materialArea = config.materialWidth * config.materialHeight
        let utilization = materialArea > 0 ? min(totalShapeArea / materialArea, 1.0) : 0

        return NestingResult(
            placements: placements,
            utilization: utilization,
            boundingBox: resultBB
        )
    }

    // MARK: - Bottom-Left Position Search

    /// Find the bottom-left-most position where `normalized` (origin-aligned) fits
    /// without overlapping existing placed polygons or exceeding material bounds.
    private static func findBottomLeftPosition(
        normalized: [Vector2D],
        shapeWidth: Double,
        shapeHeight: Double,
        matMinX: Double,
        matMinY: Double,
        matMaxX: Double,
        matMaxY: Double,
        spacing: Double,
        placedPolygons: [(polygon: [Vector2D], boundingBox: BoundingBox2D)]
    ) -> (Double, Double)? {
        // Candidate Y values: start from matMinY, plus the top edge + spacing of each placed shape.
        var candidateYs: [Double] = [matMinY]
        for placed in placedPolygons {
            let topEdge = placed.boundingBox.max.y + spacing
            if topEdge + shapeHeight <= matMaxY {
                candidateYs.append(topEdge)
            }
        }
        candidateYs.sort()

        // Remove near-duplicates.
        candidateYs = deduplicateSorted(candidateYs, tolerance: 1e-9)

        var bestPos: (Double, Double)?
        var bestY = Double.infinity
        var bestX = Double.infinity

        for cy in candidateYs {
            // Early exit: once we have a valid placement, no need to check higher Y values.
            if cy > bestY { break }

            // Candidate X values: start from matMinX, plus the right edge + spacing of each placed shape.
            var candidateXs: [Double] = [matMinX]
            for placed in placedPolygons {
                let rightEdge = placed.boundingBox.max.x + spacing
                if rightEdge + shapeWidth <= matMaxX {
                    candidateXs.append(rightEdge)
                }
            }
            candidateXs.sort()
            candidateXs = deduplicateSorted(candidateXs, tolerance: 1e-9)

            for cx in candidateXs {
                // Check if (cx, cy) is a valid placement.
                let candidateBB = BoundingBox2D(
                    min: Vector2D(cx, cy),
                    max: Vector2D(cx + shapeWidth, cy + shapeHeight)
                )

                // Must fit within material.
                guard candidateBB.max.x <= matMaxX && candidateBB.max.y <= matMaxY else {
                    continue
                }

                let candidatePoly = translatePolygon(normalized, by: Vector2D(cx, cy))

                var overlaps = false
                for placed in placedPolygons {
                    // Fast bounding box rejection (expanded by spacing).
                    let expandedPlaced = placed.boundingBox.expanded(by: spacing)
                    guard expandedPlaced.intersects(candidateBB) else { continue }

                    // Detailed polygon overlap test with spacing.
                    if polygonsOverlap(candidatePoly, placed.polygon, spacing: spacing) {
                        overlaps = true
                        break
                    }
                }

                if !overlaps {
                    if cy < bestY || (cy == bestY && cx < bestX) {
                        bestY = cy
                        bestX = cx
                        bestPos = (cx, cy)
                    }
                    // Found best X for this Y, move to next Y.
                    break
                }
            }
        }

        return bestPos
    }

    // MARK: - Rotation Angles

    private static func buildRotationAngles(config: NestingConfig) -> [Double] {
        guard config.allowRotation && config.rotationSteps > 1 else {
            return [0.0]
        }
        return (0..<config.rotationSteps).map { i in
            Double(i) * (2.0 * .pi / Double(config.rotationSteps))
        }
    }

    // MARK: - Polygon Transformations

    /// Rotate all vertices of a polygon around the origin.
    private static func rotatePolygon(_ polygon: [Vector2D], by angle: Double) -> [Vector2D] {
        guard abs(angle) > 1e-12 else { return polygon }
        return polygon.map { $0.rotated(by: angle) }
    }

    /// Translate all vertices of a polygon by an offset.
    private static func translatePolygon(_ polygon: [Vector2D], by offset: Vector2D) -> [Vector2D] {
        polygon.map { $0 + offset }
    }

    /// Flip a polygon horizontally (mirror across Y axis, i.e., negate x coordinates).
    private static func flipPolygon(_ polygon: [Vector2D]) -> [Vector2D] {
        polygon.map { Vector2D(-$0.x, $0.y) }
    }

    /// Apply a full placement transform to a shape: flip, rotate, translate.
    private static func transformShape(
        _ shape: [Vector2D],
        placement: ShapePlacement
    ) -> [Vector2D] {
        var result = shape
        if placement.flipped {
            result = flipPolygon(result)
        }
        if abs(placement.rotation) > 1e-12 {
            result = rotatePolygon(result, by: placement.rotation)
        }
        result = translatePolygon(result, by: placement.position)
        return result
    }

    // MARK: - Overlap Detection

    /// Check whether two convex or concave polygons overlap, considering a minimum spacing.
    /// Uses bounding box pre-check, then Separating Axis Theorem (SAT) for a conservative test,
    /// followed by vertex-in-polygon tests for concave shapes.
    private static func polygonsOverlap(
        _ a: [Vector2D],
        _ b: [Vector2D],
        spacing: Double
    ) -> Bool {
        // Expand polygon A outward by spacing/2 and B by spacing/2 is equivalent to
        // checking if the Minkowski sum boundaries are closer than spacing.
        // For simplicity, we use a vertex-distance and edge-distance approach.

        // Check if any vertex of A is inside B (or within spacing distance).
        for vertex in a {
            if isPointInsideOrNearPolygon(vertex, polygon: b, margin: spacing) {
                return true
            }
        }

        // Check if any vertex of B is inside A (or within spacing distance).
        for vertex in b {
            if isPointInsideOrNearPolygon(vertex, polygon: a, margin: spacing) {
                return true
            }
        }

        // Check if any edges are closer than spacing.
        if edgesWithinDistance(a, b, distance: spacing) {
            return true
        }

        return false
    }

    /// Test if a point is inside a polygon or within `margin` distance of its boundary.
    private static func isPointInsideOrNearPolygon(
        _ point: Vector2D,
        polygon: [Vector2D],
        margin: Double
    ) -> Bool {
        // Ray-casting point-in-polygon test.
        if isPointInsidePolygon(point, polygon) {
            return true
        }

        // Check distance to each edge.
        let n = polygon.count
        let marginSq = margin * margin
        for i in 0..<n {
            let a = polygon[i]
            let b = polygon[(i + 1) % n]
            if pointToSegmentDistanceSquared(point, a, b) < marginSq {
                return true
            }
        }

        return false
    }

    /// Ray-casting point-in-polygon test.
    private static func isPointInsidePolygon(_ point: Vector2D, _ polygon: [Vector2D]) -> Bool {
        var inside = false
        let n = polygon.count
        var j = n - 1

        for i in 0..<n {
            let pi = polygon[i]
            let pj = polygon[j]

            if (pi.y > point.y) != (pj.y > point.y) {
                let xIntersect = pi.x + (point.y - pi.y) / (pj.y - pi.y) * (pj.x - pi.x)
                if point.x < xIntersect {
                    inside = !inside
                }
            }
            j = i
        }

        return inside
    }

    /// Squared distance from a point to a line segment.
    private static func pointToSegmentDistanceSquared(
        _ p: Vector2D,
        _ a: Vector2D,
        _ b: Vector2D
    ) -> Double {
        let ab = b - a
        let ap = p - a
        let lenSq = ab.lengthSquared
        guard lenSq > 1e-20 else {
            return ap.lengthSquared
        }

        let t = max(0, min(1, ap.dot(ab) / lenSq))
        let projection = a + ab * t
        return p.distanceSquared(to: projection)
    }

    /// Check if any edge of polygon A is within `distance` of any edge of polygon B.
    private static func edgesWithinDistance(
        _ a: [Vector2D],
        _ b: [Vector2D],
        distance: Double
    ) -> Bool {
        let distSq = distance * distance
        let na = a.count
        let nb = b.count

        for i in 0..<na {
            let a1 = a[i]
            let a2 = a[(i + 1) % na]

            for j in 0..<nb {
                let b1 = b[j]
                let b2 = b[(j + 1) % nb]

                if segmentToSegmentDistanceSquared(a1, a2, b1, b2) < distSq {
                    return true
                }
            }
        }

        return false
    }

    /// Squared distance between two line segments.
    private static func segmentToSegmentDistanceSquared(
        _ a1: Vector2D, _ a2: Vector2D,
        _ b1: Vector2D, _ b2: Vector2D
    ) -> Double {
        // Check if segments intersect (distance = 0).
        if segmentsIntersect(a1, a2, b1, b2) {
            return 0
        }

        // Otherwise, minimum distance is the smallest of point-to-segment distances.
        let d1 = pointToSegmentDistanceSquared(a1, b1, b2)
        let d2 = pointToSegmentDistanceSquared(a2, b1, b2)
        let d3 = pointToSegmentDistanceSquared(b1, a1, a2)
        let d4 = pointToSegmentDistanceSquared(b2, a1, a2)

        return min(min(d1, d2), min(d3, d4))
    }

    /// Test whether two line segments intersect (proper or endpoint).
    private static func segmentsIntersect(
        _ a1: Vector2D, _ a2: Vector2D,
        _ b1: Vector2D, _ b2: Vector2D
    ) -> Bool {
        let d1 = a2 - a1
        let d2 = b2 - b1
        let cross = d1.cross(d2)

        guard abs(cross) > 1e-10 else { return false }

        let d3 = b1 - a1
        let t = d3.cross(d2) / cross
        let u = d3.cross(d1) / cross

        return t >= 0 && t <= 1 && u >= 0 && u <= 1
    }

    // MARK: - Area and Bounding Box Computation

    /// Compute the signed area of a polygon (positive for counter-clockwise winding).
    private static func polygonArea(_ polygon: [Vector2D]) -> Double {
        guard polygon.count >= 3 else { return 0 }
        var area = 0.0
        let n = polygon.count
        for i in 0..<n {
            let j = (i + 1) % n
            area += polygon[i].x * polygon[j].y
            area -= polygon[j].x * polygon[i].y
        }
        return abs(area) / 2.0
    }

    /// Compute total area occupied by placed shapes.
    private static func computeTotalShapeArea(
        shapes: [[Vector2D]],
        placements: [ShapePlacement]
    ) -> Double {
        var totalArea = 0.0
        for placement in placements {
            let shape = shapes[placement.shapeIndex]
            totalArea += polygonArea(shape)
        }
        return totalArea
    }

    /// Compute the overall bounding box of all placed polygons, clamped to material bounds.
    private static func computeOverallBoundingBox(
        placedPolygons: [(polygon: [Vector2D], boundingBox: BoundingBox2D)],
        config: NestingConfig
    ) -> BoundingBox2D {
        guard !placedPolygons.isEmpty else {
            return BoundingBox2D(
                min: .zero,
                max: Vector2D(config.materialWidth, config.materialHeight)
            )
        }

        var overall = BoundingBox2D()
        for placed in placedPolygons {
            overall.expand(toInclude: placed.boundingBox)
        }
        return overall
    }

    // MARK: - Utilities

    /// Remove near-duplicate values from a sorted array.
    private static func deduplicateSorted(_ values: [Double], tolerance: Double) -> [Double] {
        guard !values.isEmpty else { return values }
        var result: [Double] = [values[0]]
        for i in 1..<values.count {
            if values[i] - result.last! > tolerance {
                result.append(values[i])
            }
        }
        return result
    }
}
