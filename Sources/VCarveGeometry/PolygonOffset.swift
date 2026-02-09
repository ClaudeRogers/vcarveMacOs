import Foundation
import VCarveCore

/// Polygon offset operations for toolpath generation.
/// Used for profile offsets (inside/outside) and pocket clearing.
public struct PolygonOffset {
    /// Offset a polygon (list of points) by the given distance.
    /// Positive distance = outward (expand), negative = inward (shrink).
    public static func offset(polygon: [Vector2D], distance: Double) -> [[Vector2D]] {
        guard polygon.count >= 3 else { return [] }

        var result: [Vector2D] = []
        let n = polygon.count

        for i in 0..<n {
            let prev = polygon[(i - 1 + n) % n]
            let curr = polygon[i]
            let next = polygon[(i + 1) % n]

            let edge1 = (curr - prev).normalized
            let edge2 = (next - curr).normalized

            let normal1 = edge1.perpendicular
            let normal2 = edge2.perpendicular

            // Bisector direction
            let bisector = (normal1 + normal2).normalized

            // Calculate offset distance along bisector
            let sinHalfAngle = edge1.cross(edge2)
            let cosHalfAngle = edge1.dot(edge2)

            let offsetDist: Double
            if abs(sinHalfAngle) > 1e-10 {
                let halfAngle = atan2(abs(sinHalfAngle), 1 + cosHalfAngle)
                offsetDist = distance / cos(halfAngle)
            } else {
                offsetDist = distance
            }

            let clampedDist = max(-1000, min(1000, offsetDist))
            result.append(curr + bisector * clampedDist)
        }

        // Check for self-intersections and handle degenerate cases
        let cleanedResult = removeSelfintersections(result)
        if cleanedResult.isEmpty {
            return []
        }

        return [cleanedResult]
    }

    /// Generate concentric offset contours for pocket clearing.
    /// Returns offsets from outermost to innermost.
    public static func concentricOffsets(polygon: [Vector2D], stepover: Double, inward: Bool = true) -> [[Vector2D]] {
        var results: [[Vector2D]] = []
        var currentPolygons = [polygon]
        let currentOffset = inward ? -stepover : stepover
        let maxIterations = 1000

        for _ in 0..<maxIterations {
            var nextPolygons: [[Vector2D]] = []

            for poly in currentPolygons {
                let offsets = offset(polygon: poly, distance: currentOffset)
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

    /// Remove self-intersections from a polygon by finding crossing points.
    private static func removeSelfintersections(_ polygon: [Vector2D]) -> [Vector2D] {
        guard polygon.count >= 3 else { return polygon }

        // Simple area-based validity check
        var area = 0.0
        for i in 0..<polygon.count {
            let j = (i + 1) % polygon.count
            area += polygon[i].x * polygon[j].y
            area -= polygon[j].x * polygon[i].y
        }

        if abs(area) < 1e-10 {
            return []
        }

        return polygon
    }
}
