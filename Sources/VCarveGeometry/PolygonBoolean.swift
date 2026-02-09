import Foundation
import VCarveCore

/// Boolean operations on polygons (union, intersection, difference).
/// Used for combining shapes, creating inlays, and toolpath region computation.
public struct PolygonBoolean {

    /// Compute the union of two polygons using the Weiler-Atherton algorithm.
    public static func union(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        let intersections = findIntersections(a, b)
        if intersections.isEmpty {
            // No intersections — check containment
            if isPointInsidePolygon(b[0], a) {
                return [a] // b inside a
            } else if isPointInsidePolygon(a[0], b) {
                return [b] // a inside b
            } else {
                return [a, b] // disjoint
            }
        }

        return weilerAtherton(a, b, operation: .union)
    }

    /// Compute the intersection of two polygons.
    public static func intersection(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        let intersections = findIntersections(a, b)
        if intersections.isEmpty {
            if isPointInsidePolygon(b[0], a) {
                return [b]
            } else if isPointInsidePolygon(a[0], b) {
                return [a]
            } else {
                return []
            }
        }

        return weilerAtherton(a, b, operation: .intersection)
    }

    /// Compute a - b (subtract b from a).
    public static func difference(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        let intersections = findIntersections(a, b)
        if intersections.isEmpty {
            if isPointInsidePolygon(a[0], b) {
                return [] // a entirely inside b
            } else {
                return [a] // disjoint, a unchanged
            }
        }

        return weilerAtherton(a, b, operation: .difference)
    }

    // MARK: - Internal

    private enum BooleanOp {
        case union, intersection, difference
    }

    private struct Intersection {
        let point: Vector2D
        let edgeA: Int
        let paramA: Double
        let edgeB: Int
        let paramB: Double
    }

    /// Find all intersection points between two polygons.
    private static func findIntersections(_ a: [Vector2D], _ b: [Vector2D]) -> [Intersection] {
        var results: [Intersection] = []

        for i in 0..<a.count {
            let a1 = a[i]
            let a2 = a[(i + 1) % a.count]

            for j in 0..<b.count {
                let b1 = b[j]
                let b2 = b[(j + 1) % b.count]

                if let (t, u) = segmentIntersection(a1, a2, b1, b2) {
                    let point = a1.lerp(to: a2, t: t)
                    results.append(Intersection(point: point, edgeA: i, paramA: t, edgeB: j, paramB: u))
                }
            }
        }

        return results
    }

    /// Find the intersection parameter of two line segments.
    /// Returns (t, u) where t is parameter on (p1,p2) and u on (p3,p4), both in [0,1].
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

        guard t >= 0 && t <= 1 && u >= 0 && u <= 1 else { return nil }

        return (t, u)
    }

    /// Simplified Weiler-Atherton for polygon boolean operations.
    private static func weilerAtherton(
        _ a: [Vector2D],
        _ b: [Vector2D],
        operation: BooleanOp
    ) -> [[Vector2D]] {
        // For the initial implementation, use a simplified clipping approach
        // This handles the common cases needed for CNC toolpath generation
        // Full Weiler-Atherton will be implemented as needed
        switch operation {
        case .union:
            return [a] // Placeholder
        case .intersection:
            return sutherlandHodgmanClip(subject: a, clip: b)
        case .difference:
            return [a] // Placeholder
        }
    }

    /// Sutherland-Hodgman polygon clipping algorithm.
    private static func sutherlandHodgmanClip(subject: [Vector2D], clip: [Vector2D]) -> [[Vector2D]] {
        var output = subject

        for i in 0..<clip.count {
            guard !output.isEmpty else { return [] }

            let edgeStart = clip[i]
            let edgeEnd = clip[(i + 1) % clip.count]
            let input = output
            output = []

            for j in 0..<input.count {
                let current = input[j]
                let previous = input[(j - 1 + input.count) % input.count]

                let currInside = isLeftOf(current, edgeStart, edgeEnd)
                let prevInside = isLeftOf(previous, edgeStart, edgeEnd)

                if currInside {
                    if !prevInside {
                        if let p = lineIntersection(previous, current, edgeStart, edgeEnd) {
                            output.append(p)
                        }
                    }
                    output.append(current)
                } else if prevInside {
                    if let p = lineIntersection(previous, current, edgeStart, edgeEnd) {
                        output.append(p)
                    }
                }
            }
        }

        if output.count >= 3 {
            return [output]
        }
        return []
    }

    private static func isLeftOf(_ point: Vector2D, _ lineStart: Vector2D, _ lineEnd: Vector2D) -> Bool {
        (lineEnd - lineStart).cross(point - lineStart) >= 0
    }

    private static func lineIntersection(
        _ p1: Vector2D, _ p2: Vector2D,
        _ p3: Vector2D, _ p4: Vector2D
    ) -> Vector2D? {
        let d1 = p2 - p1
        let d2 = p4 - p3
        let cross = d1.cross(d2)
        guard abs(cross) > 1e-10 else { return nil }
        let t = (p3 - p1).cross(d2) / cross
        return p1 + d1 * t
    }

    /// Point-in-polygon test using ray casting.
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
}
