import Foundation
import ClaudeCarveCore

/// Polygon offset operations for toolpath generation.
/// Uses the vertex bisector method with proper self-intersection handling.
public struct PolygonOffset {

    /// Offset a polygon by the given distance.
    /// Positive distance = outward (expand), negative = inward (shrink).
    /// Returns potentially multiple polygons (a single offset can split into islands).
    public static func offset(polygon: [Vector2D], distance: Double) -> [[Vector2D]] {
        guard polygon.count >= 3 else { return [] }

        // Step 1: Compute raw offset vertices using bisector method
        let rawOffset = computeRawOffset(polygon: polygon, distance: distance)
        guard rawOffset.count >= 3 else { return [] }

        // Step 2: Find and handle self-intersections
        let cleanedPolygons = resolveSelfintersections(rawOffset)

        // Step 3: Filter out invalid (zero-area or wrong-winding) results
        return filterValid(polygons: cleanedPolygons, originalWinding: signedArea(polygon), distance: distance)
    }

    /// Generate concentric offset contours for pocket clearing.
    /// Returns offsets from outermost to innermost.
    public static func concentricOffsets(polygon: [Vector2D], stepover: Double, inward: Bool = true) -> [[Vector2D]] {
        var results: [[Vector2D]] = []
        var currentPolygons = [polygon]
        let offsetDist = inward ? -stepover : stepover
        let maxIterations = 1000

        for _ in 0..<maxIterations {
            var nextPolygons: [[Vector2D]] = []

            for poly in currentPolygons {
                let offsets = offset(polygon: poly, distance: offsetDist)
                for off in offsets {
                    if off.count >= 3 {
                        nextPolygons.append(off)
                    }
                }
            }

            if nextPolygons.isEmpty { break }
            results.append(contentsOf: nextPolygons)
            currentPolygons = nextPolygons
        }

        return results
    }

    // MARK: - Raw Offset Computation

    /// Compute raw offset vertices using the vertex normal bisector method.
    private static func computeRawOffset(polygon: [Vector2D], distance: Double) -> [Vector2D] {
        let n = polygon.count
        var result: [Vector2D] = []

        for i in 0..<n {
            let prev = polygon[(i - 1 + n) % n]
            let curr = polygon[i]
            let next = polygon[(i + 1) % n]

            let edge1Dir = (curr - prev).normalized
            let edge2Dir = (next - curr).normalized

            // Edge normals (pointing left of edge direction)
            let normal1 = edge1Dir.perpendicular
            let normal2 = edge2Dir.perpendicular

            // Bisector of the two normals
            let bisector = (normal1 + normal2)
            let bisectorLen = bisector.length

            if bisectorLen < 1e-10 {
                // Normals are opposite (180° turn) - use either normal
                result.append(curr + normal1 * distance)
                continue
            }

            let bisectorNorm = bisector / bisectorLen

            // Compute the offset distance along the bisector
            // The bisector needs to be scaled so the perpendicular distance to offset edges = distance
            let dotProduct = normal1.dot(bisectorNorm)

            let offsetDist: Double
            if abs(dotProduct) > 1e-10 {
                offsetDist = distance / dotProduct
            } else {
                offsetDist = distance
            }

            // Miter limit: prevent extremely sharp corners from creating huge spikes
            let miterLimit = 4.0 * abs(distance)
            let clampedDist = max(-miterLimit, min(miterLimit, offsetDist))

            let offsetPoint = curr + bisectorNorm * clampedDist

            // For very sharp corners (miter exceeded), insert a bevel
            if abs(offsetDist) > miterLimit {
                // Insert two points instead (bevel join)
                let p1 = curr + normal1 * distance
                let p2 = curr + normal2 * distance
                result.append(p1)
                result.append(p2)
            } else {
                result.append(offsetPoint)
            }
        }

        return result
    }

    // MARK: - Self-Intersection Resolution

    /// Find and resolve self-intersections in an offset polygon.
    /// When a polygon self-intersects, it creates loops. We split it into
    /// multiple valid sub-polygons by cutting at intersection points.
    private static func resolveSelfintersections(_ polygon: [Vector2D]) -> [[Vector2D]] {
        let n = polygon.count
        guard n >= 3 else { return [] }

        // Find all self-intersection points
        var intersections: [(Int, Int, Double, Double, Vector2D)] = [] // (edge_i, edge_j, t_i, t_j, point)

        for i in 0..<n {
            let a1 = polygon[i]
            let a2 = polygon[(i + 1) % n]

            guard i + 2 < n else { continue }
            for j in (i + 2)..<n {
                // Don't check adjacent edges
                if j == (i + n - 1) % n { continue }
                if (i == 0 && j == n - 1) { continue }

                let b1 = polygon[j]
                let b2 = polygon[(j + 1) % n]

                if let (t, u) = segmentIntersection(a1, a2, b1, b2) {
                    let point = a1.lerp(to: a2, t: t)
                    intersections.append((i, j, t, u, point))
                }
            }
        }

        // No self-intersections: return the polygon as-is
        if intersections.isEmpty {
            return [polygon]
        }

        // Split polygon at intersection points to extract valid sub-polygons
        return extractValidLoops(polygon: polygon, intersections: intersections)
    }

    /// Extract valid loops from a self-intersecting polygon.
    private static func extractValidLoops(
        polygon: [Vector2D],
        intersections: [(Int, Int, Double, Double, Vector2D)]
    ) -> [[Vector2D]] {
        let n = polygon.count

        // Build an augmented vertex list with intersection points inserted
        struct AugmentedVertex {
            let point: Vector2D
            let originalIndex: Int   // -1 for intersection points
            let isIntersection: Bool
            let pairIndex: Int       // For intersection points, the index of the paired vertex
        }

        // Sort intersections by edge index and parameter
        var edgeIntersections: [Int: [(Double, Vector2D, Int)]] = [:]
        for (idx, intersection) in intersections.enumerated() {
            let (ei, ej, ti, tj, point) = intersection
            edgeIntersections[ei, default: []].append((ti, point, idx))
            edgeIntersections[ej, default: []].append((tj, point, idx + intersections.count))
        }

        // Sort intersections on each edge by parameter
        for key in edgeIntersections.keys {
            edgeIntersections[key]?.sort { $0.0 < $1.0 }
        }

        // Build augmented list
        var augmented: [AugmentedVertex] = []
        var intersectionPairs: [Int: Int] = [:]

        for i in 0..<n {
            augmented.append(AugmentedVertex(point: polygon[i], originalIndex: i, isIntersection: false, pairIndex: -1))

            if let edgeInts = edgeIntersections[i] {
                for (_, point, id) in edgeInts {
                    let idx = augmented.count
                    augmented.append(AugmentedVertex(point: point, originalIndex: -1, isIntersection: true, pairIndex: id))

                    // Track pairs: map this intersection's ID to its augmented index
                    intersectionPairs[id] = idx
                }
            }
        }

        // Now extract loops by traversing the augmented list
        // For simplicity, try to extract the largest valid loops
        var loops: [[Vector2D]] = []
        var visited = Set<Int>()

        for startIdx in 0..<augmented.count {
            guard augmented[startIdx].isIntersection && !visited.contains(startIdx) else { continue }

            var loop: [Vector2D] = []
            var current = startIdx
            var maxSteps = augmented.count + 1

            while maxSteps > 0 {
                maxSteps -= 1
                visited.insert(current)
                loop.append(augmented[current].point)

                // Move to next vertex
                current = (current + 1) % augmented.count

                if current == startIdx { break }

                // If we hit another intersection, we could jump to its pair
                if augmented[current].isIntersection && !visited.contains(current) {
                    // Check if jumping creates a shorter/valid loop
                    let pairId = augmented[current].pairIndex
                    if let pairIdx = intersectionPairs[pairId], !visited.contains(pairIdx) {
                        loop.append(augmented[current].point)
                        visited.insert(current)
                        current = (pairIdx + 1) % augmented.count
                    }
                }
            }

            if loop.count >= 3 {
                loops.append(loop)
            }
        }

        // If no loops were extracted (complex case), fall back to the original polygon
        if loops.isEmpty {
            return [polygon]
        }

        return loops
    }

    // MARK: - Validation & Filtering

    /// Filter offset results to keep only valid polygons.
    private static func filterValid(polygons: [[Vector2D]], originalWinding: Double, distance: Double) -> [[Vector2D]] {
        return polygons.compactMap { poly -> [Vector2D]? in
            guard poly.count >= 3 else { return nil }

            let area = signedArea(poly)

            // Must have non-zero area
            guard abs(area) > 1e-10 else { return nil }

            // For inward offsets (distance < 0), keep polygons with same winding as original
            // For outward offsets (distance > 0), also keep same winding
            if (originalWinding > 0) == (area > 0) {
                return poly
            }

            // If winding flipped and area is very small, this is likely a degenerate artifact
            if abs(area) < abs(originalWinding) * 0.01 {
                return nil
            }

            return nil
        }
    }

    // MARK: - Utilities

    /// Signed area of a polygon (positive = CCW, negative = CW).
    private static func signedArea(_ polygon: [Vector2D]) -> Double {
        var area = 0.0
        let n = polygon.count
        for i in 0..<n {
            let j = (i + 1) % n
            area += polygon[i].x * polygon[j].y
            area -= polygon[j].x * polygon[i].y
        }
        return area / 2.0
    }

    /// Segment-segment intersection. Returns (t, u) parameters in [0,1] or nil.
    private static func segmentIntersection(
        _ p1: Vector2D, _ p2: Vector2D,
        _ p3: Vector2D, _ p4: Vector2D
    ) -> (Double, Double)? {
        let d1 = p2 - p1
        let d2 = p4 - p3
        let cross = d1.cross(d2)

        guard abs(cross) > 1e-10 else { return nil }

        let d3 = p3 - p1
        let t = d3.cross(d2) / cross
        let u = d3.cross(d1) / cross

        // Use slightly inset range to avoid numerical issues at endpoints
        let eps = 1e-8
        guard t > eps, t < 1 - eps, u > eps, u < 1 - eps else { return nil }

        return (t, u)
    }
}
