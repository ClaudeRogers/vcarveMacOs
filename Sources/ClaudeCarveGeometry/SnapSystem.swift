import Foundation
import ClaudeCarveCore

/// Types of snap targets available in the design canvas.
public enum SnapType: String, Codable, Sendable, CaseIterable {
    case grid           // Snap to grid intersections
    case endpoint       // Snap to path endpoints
    case midpoint       // Snap to segment midpoints
    case center         // Snap to circle/ellipse/arc centers
    case intersection   // Snap to path-path intersections
    case nearestPoint   // Snap to nearest point on a path
    case guideline      // Snap to guidelines
    case tangent        // Snap tangent to curves
    case perpendicular  // Snap perpendicular to edges
}

/// A guideline for precise alignment.
public struct Guideline: Identifiable, Codable, Sendable {
    public let id: UUID
    public var orientation: GuidelineOrientation
    public var position: Double
    public var isLocked: Bool

    public init(id: UUID = UUID(), orientation: GuidelineOrientation, position: Double, isLocked: Bool = false) {
        self.id = id
        self.orientation = orientation
        self.position = position
        self.isLocked = isLocked
    }
}

public enum GuidelineOrientation: String, Codable, Sendable {
    case horizontal
    case vertical
    case angular
}

/// Result of a snap operation.
public struct SnapResult: Sendable {
    public let snappedPoint: Vector2D
    public let snapType: SnapType
    public let distance: Double

    public init(snappedPoint: Vector2D, snapType: SnapType, distance: Double) {
        self.snappedPoint = snappedPoint
        self.snapType = snapType
        self.distance = distance
    }
}

/// Configuration for the snap system.
public struct SnapConfig: Codable, Sendable {
    public var isEnabled: Bool
    public var snapRadius: Double       // World-space snap radius
    public var gridSpacing: Double      // Grid snap spacing
    public var enabledSnapTypes: Set<SnapType>
    public var guidelines: [Guideline]
    public var angularSnap: Bool        // Snap to 15° increments
    public var angularIncrement: Double // Degrees

    public init(
        isEnabled: Bool = true,
        snapRadius: Double = 5.0,
        gridSpacing: Double = 10.0,
        enabledSnapTypes: Set<SnapType> = Set(SnapType.allCases),
        guidelines: [Guideline] = [],
        angularSnap: Bool = false,
        angularIncrement: Double = 15.0
    ) {
        self.isEnabled = isEnabled
        self.snapRadius = snapRadius
        self.gridSpacing = gridSpacing
        self.enabledSnapTypes = enabledSnapTypes
        self.guidelines = guidelines
        self.angularSnap = angularSnap
        self.angularIncrement = angularIncrement
    }
}

extension SnapType: Hashable {}

/// The snap system finds the best snap target for a cursor position.
public struct SnapSystem {

    /// Find the best snap target for a point, given paths and configuration.
    public static func snap(
        point: Vector2D,
        paths: [VectorPath],
        config: SnapConfig,
        referencePoint: Vector2D? = nil
    ) -> SnapResult? {
        guard config.isEnabled else { return nil }

        var candidates: [SnapResult] = []

        if config.enabledSnapTypes.contains(.grid) {
            if let result = snapToGrid(point: point, spacing: config.gridSpacing) {
                candidates.append(result)
            }
        }

        if config.enabledSnapTypes.contains(.guideline) {
            candidates.append(contentsOf: snapToGuidelines(point: point, guidelines: config.guidelines))
        }

        if config.enabledSnapTypes.contains(.endpoint) {
            candidates.append(contentsOf: snapToEndpoints(point: point, paths: paths))
        }

        if config.enabledSnapTypes.contains(.midpoint) {
            candidates.append(contentsOf: snapToMidpoints(point: point, paths: paths))
        }

        if config.enabledSnapTypes.contains(.center) {
            candidates.append(contentsOf: snapToCenters(point: point, paths: paths))
        }

        if config.enabledSnapTypes.contains(.intersection) {
            candidates.append(contentsOf: snapToIntersections(point: point, paths: paths))
        }

        if config.enabledSnapTypes.contains(.nearestPoint) {
            candidates.append(contentsOf: snapToNearestPoints(point: point, paths: paths))
        }

        // Angular snap relative to reference point
        if config.angularSnap, let ref = referencePoint {
            if let result = snapToAngle(point: point, reference: ref, increment: config.angularIncrement) {
                candidates.append(result)
            }
        }

        // Filter by snap radius and return closest
        let valid = candidates.filter { $0.distance <= config.snapRadius }
        return valid.min(by: { $0.distance < $1.distance })
    }

    // MARK: - Grid Snap

    private static func snapToGrid(point: Vector2D, spacing: Double) -> SnapResult? {
        guard spacing > 0 else { return nil }
        let snappedX = (point.x / spacing).rounded() * spacing
        let snappedY = (point.y / spacing).rounded() * spacing
        let snapped = Vector2D(snappedX, snappedY)
        return SnapResult(snappedPoint: snapped, snapType: .grid, distance: point.distance(to: snapped))
    }

    // MARK: - Guideline Snap

    private static func snapToGuidelines(point: Vector2D, guidelines: [Guideline]) -> [SnapResult] {
        var results: [SnapResult] = []

        for guide in guidelines {
            switch guide.orientation {
            case .horizontal:
                let snapped = Vector2D(point.x, guide.position)
                let dist = abs(point.y - guide.position)
                results.append(SnapResult(snappedPoint: snapped, snapType: .guideline, distance: dist))

            case .vertical:
                let snapped = Vector2D(guide.position, point.y)
                let dist = abs(point.x - guide.position)
                results.append(SnapResult(snappedPoint: snapped, snapType: .guideline, distance: dist))

            case .angular:
                // Angular guidelines not yet implemented
                break
            }
        }

        return results
    }

    // MARK: - Endpoint Snap

    private static func snapToEndpoints(point: Vector2D, paths: [VectorPath]) -> [SnapResult] {
        var results: [SnapResult] = []

        for path in paths {
            // Start point
            let startDist = point.distance(to: path.startPoint)
            results.append(SnapResult(snappedPoint: path.startPoint, snapType: .endpoint, distance: startDist))

            // Segment endpoints
            for segment in path.segments {
                let endPoint = segment.endPoint
                let dist = point.distance(to: endPoint)
                results.append(SnapResult(snappedPoint: endPoint, snapType: .endpoint, distance: dist))
            }
        }

        return results
    }

    // MARK: - Midpoint Snap

    private static func snapToMidpoints(point: Vector2D, paths: [VectorPath]) -> [SnapResult] {
        var results: [SnapResult] = []

        for path in paths {
            var current = path.startPoint
            for segment in path.segments {
                let end = segment.endPoint
                let mid = current.lerp(to: end, t: 0.5)
                let dist = point.distance(to: mid)
                results.append(SnapResult(snappedPoint: mid, snapType: .midpoint, distance: dist))
                current = end
            }
        }

        return results
    }

    // MARK: - Center Snap

    private static func snapToCenters(point: Vector2D, paths: [VectorPath]) -> [SnapResult] {
        var results: [SnapResult] = []

        for path in paths {
            // Arc centers
            for segment in path.segments {
                if case .arcTo(let center, _, _, _, _) = segment {
                    let dist = point.distance(to: center)
                    results.append(SnapResult(snappedPoint: center, snapType: .center, distance: dist))
                }
            }

            // Bounding box center for closed paths (approximate circle/ellipse center)
            if path.isClosed {
                let bb = path.boundingBox
                let center = bb.center
                let dist = point.distance(to: center)
                results.append(SnapResult(snappedPoint: center, snapType: .center, distance: dist))
            }
        }

        return results
    }

    // MARK: - Intersection Snap

    private static func snapToIntersections(point: Vector2D, paths: [VectorPath]) -> [SnapResult] {
        var results: [SnapResult] = []
        guard paths.count >= 2 else { return results }

        // Check pairs of paths for intersections (limited for performance)
        let maxPairs = min(paths.count, 10)
        for i in 0..<maxPairs {
            for j in (i + 1)..<min(paths.count, maxPairs) {
                let intersections = findPathIntersections(paths[i], paths[j])
                for intersection in intersections {
                    let dist = point.distance(to: intersection)
                    results.append(SnapResult(snappedPoint: intersection, snapType: .intersection, distance: dist))
                }
            }
        }

        return results
    }

    // MARK: - Nearest Point Snap

    private static func snapToNearestPoints(point: Vector2D, paths: [VectorPath]) -> [SnapResult] {
        var results: [SnapResult] = []

        for path in paths {
            let points = path.flattenedPoints(tolerance: 0.5)
            guard points.count >= 2 else { continue }

            var bestDist = Double.infinity
            var bestPoint = point

            for i in 0..<(points.count - 1) {
                let nearest = nearestPointOnSegment(point, points[i], points[i + 1])
                let dist = point.distance(to: nearest)
                if dist < bestDist {
                    bestDist = dist
                    bestPoint = nearest
                }
            }

            if bestDist < Double.infinity {
                results.append(SnapResult(snappedPoint: bestPoint, snapType: .nearestPoint, distance: bestDist))
            }
        }

        return results
    }

    // MARK: - Angular Snap

    private static func snapToAngle(point: Vector2D, reference: Vector2D, increment: Double) -> SnapResult? {
        let dx = point.x - reference.x
        let dy = point.y - reference.y
        let dist = sqrt(dx * dx + dy * dy)
        guard dist > 0.001 else { return nil }

        let angle = atan2(dy, dx)
        let incrementRad = increment * .pi / 180.0
        let snappedAngle = (angle / incrementRad).rounded() * incrementRad

        let snapped = Vector2D(
            reference.x + dist * cos(snappedAngle),
            reference.y + dist * sin(snappedAngle)
        )

        return SnapResult(snappedPoint: snapped, snapType: .tangent, distance: point.distance(to: snapped))
    }

    // MARK: - Helpers

    private static func nearestPointOnSegment(_ p: Vector2D, _ a: Vector2D, _ b: Vector2D) -> Vector2D {
        let ab = b - a
        let ap = p - a
        let lenSq = ab.lengthSquared
        guard lenSq > 1e-12 else { return a }

        let t = max(0, min(1, ap.dot(ab) / lenSq))
        return a + ab * t
    }

    private static func findPathIntersections(_ path1: VectorPath, _ path2: VectorPath) -> [Vector2D] {
        let pts1 = path1.flattenedPoints(tolerance: 0.5)
        let pts2 = path2.flattenedPoints(tolerance: 0.5)
        var intersections: [Vector2D] = []

        guard pts1.count >= 2 && pts2.count >= 2 else { return intersections }

        for i in 0..<(pts1.count - 1) {
            for j in 0..<(pts2.count - 1) {
                if let point = segmentIntersection(pts1[i], pts1[i + 1], pts2[j], pts2[j + 1]) {
                    intersections.append(point)
                }
            }
        }

        return intersections
    }

    private static func segmentIntersection(
        _ p1: Vector2D, _ p2: Vector2D,
        _ p3: Vector2D, _ p4: Vector2D
    ) -> Vector2D? {
        let d1 = p2 - p1
        let d2 = p4 - p3
        let cross = d1.cross(d2)
        guard abs(cross) > 1e-10 else { return nil }

        let d3 = p3 - p1
        let t = d3.cross(d2) / cross
        let u = d3.cross(d1) / cross

        guard t >= 0, t <= 1, u >= 0, u <= 1 else { return nil }
        return p1 + d1 * t
    }
}
