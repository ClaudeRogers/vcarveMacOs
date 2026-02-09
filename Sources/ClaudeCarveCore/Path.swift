import Foundation

/// A segment of a vector path.
public enum PathSegment: Hashable, Codable, Sendable {
    /// Straight line to a point.
    case lineTo(Vector2D)
    /// Quadratic Bézier curve.
    case quadTo(control: Vector2D, end: Vector2D)
    /// Cubic Bézier curve.
    case cubicTo(control1: Vector2D, control2: Vector2D, end: Vector2D)
    /// Circular arc defined by center, radius, start and end angles (radians), direction.
    case arcTo(center: Vector2D, radius: Double, startAngle: Double, endAngle: Double, clockwise: Bool)

    /// The endpoint of this segment.
    public var endPoint: Vector2D {
        switch self {
        case .lineTo(let p): return p
        case .quadTo(_, let end): return end
        case .cubicTo(_, _, let end): return end
        case .arcTo(let center, let radius, _, let endAngle, _):
            return center + Vector2D.fromAngle(endAngle) * radius
        }
    }
}

/// A continuous vector path (contour) composed of segments.
public struct VectorPath: Identifiable, Hashable, Codable, Sendable {
    public let id: UUID
    public var startPoint: Vector2D
    public var segments: [PathSegment]
    public var isClosed: Bool

    public init(id: UUID = UUID(), startPoint: Vector2D, segments: [PathSegment] = [], isClosed: Bool = false) {
        self.id = id
        self.startPoint = startPoint
        self.segments = segments
        self.isClosed = isClosed
    }

    /// All points including start and each segment endpoint.
    public var points: [Vector2D] {
        var result = [startPoint]
        for seg in segments {
            result.append(seg.endPoint)
        }
        return result
    }

    /// Compute the bounding box of the path (approximate for curves).
    public var boundingBox: BoundingBox2D {
        BoundingBox2D.enclosing(flattenedPoints(tolerance: 0.1))
    }

    /// Flatten the path to line segments with the given tolerance.
    public func flattenedPoints(tolerance: Double = 0.01) -> [Vector2D] {
        var result = [startPoint]
        var current = startPoint

        for segment in segments {
            switch segment {
            case .lineTo(let p):
                result.append(p)
                current = p

            case .quadTo(let control, let end):
                let steps = Self.adaptiveSteps(from: current, control1: control, end: end, tolerance: tolerance)
                for i in 1...steps {
                    let t = Double(i) / Double(steps)
                    let p = Self.quadraticBezier(p0: current, p1: control, p2: end, t: t)
                    result.append(p)
                }
                current = end

            case .cubicTo(let c1, let c2, let end):
                let steps = Self.adaptiveStepsCubic(from: current, c1: c1, c2: c2, end: end, tolerance: tolerance)
                for i in 1...steps {
                    let t = Double(i) / Double(steps)
                    let p = Self.cubicBezier(p0: current, p1: c1, p2: c2, p3: end, t: t)
                    result.append(p)
                }
                current = end

            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                var sweep = endAngle - startAngle
                if clockwise && sweep > 0 { sweep -= 2 * .pi }
                if !clockwise && sweep < 0 { sweep += 2 * .pi }
                let arcLength = abs(sweep) * radius
                let steps = max(4, Int(arcLength / tolerance))
                for i in 1...steps {
                    let t = Double(i) / Double(steps)
                    let angle = startAngle + sweep * t
                    let p = center + Vector2D.fromAngle(angle) * radius
                    result.append(p)
                }
                current = center + Vector2D.fromAngle(endAngle) * radius
            }
        }

        return result
    }

    /// Total length of the path (approximate).
    public var length: Double {
        let pts = flattenedPoints(tolerance: 0.01)
        var total = 0.0
        for i in 1..<pts.count {
            total += pts[i - 1].distance(to: pts[i])
        }
        return total
    }

    /// Signed area (positive = CCW, negative = CW). Only meaningful for closed paths.
    public var signedArea: Double {
        let pts = flattenedPoints(tolerance: 0.01)
        guard pts.count >= 3 else { return 0 }
        var area = 0.0
        for i in 0..<pts.count {
            let j = (i + 1) % pts.count
            area += pts[i].x * pts[j].y
            area -= pts[j].x * pts[i].y
        }
        return area / 2.0
    }

    /// Whether the path winds counter-clockwise (positive area).
    public var isCounterClockwise: Bool {
        signedArea > 0
    }

    /// Reverse the direction of this path.
    public func reversed() -> VectorPath {
        let pts = flattenedPoints(tolerance: 0.001)
        guard pts.count >= 2 else { return self }
        let reversedPts = Array(pts.reversed())
        var path = VectorPath(id: id, startPoint: reversedPts[0], isClosed: isClosed)
        for i in 1..<reversedPts.count {
            path.segments.append(.lineTo(reversedPts[i]))
        }
        return path
    }

    /// Transform all points in the path by applying a function.
    public func mapPoints(_ transform: (Vector2D) -> Vector2D) -> VectorPath {
        var result = VectorPath(id: id, startPoint: transform(startPoint), isClosed: isClosed)
        for segment in segments {
            switch segment {
            case .lineTo(let p):
                result.segments.append(.lineTo(transform(p)))
            case .quadTo(let c, let end):
                result.segments.append(.quadTo(control: transform(c), end: transform(end)))
            case .cubicTo(let c1, let c2, let end):
                result.segments.append(.cubicTo(control1: transform(c1), control2: transform(c2), end: transform(end)))
            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                result.segments.append(.arcTo(center: transform(center), radius: radius, startAngle: startAngle, endAngle: endAngle, clockwise: clockwise))
            }
        }
        return result
    }

    /// Translate (move) the path by the given offset.
    public func translated(by offset: Vector2D) -> VectorPath {
        mapPoints { $0 + offset }
    }

    /// Scale the path uniformly around a center point.
    public func scaled(by factor: Double, around center: Vector2D) -> VectorPath {
        mapPoints { center + ($0 - center) * factor }
    }

    /// Closest point on the path to the given point (approximate via flattening).
    public func closestPoint(to target: Vector2D, tolerance: Double = 0.1) -> (point: Vector2D, distance: Double)? {
        let pts = flattenedPoints(tolerance: tolerance)
        guard pts.count >= 2 else { return nil }

        var bestDist = Double.infinity
        var bestPoint = target

        for i in 0..<(pts.count - 1) {
            let a = pts[i], b = pts[i + 1]
            let ab = b - a
            let lenSq = ab.lengthSquared
            if lenSq < 1e-12 { continue }
            let t = max(0, min(1, (target - a).dot(ab) / lenSq))
            let proj = a + ab * t
            let dist = target.distance(to: proj)
            if dist < bestDist {
                bestDist = dist
                bestPoint = proj
            }
        }

        return (bestPoint, bestDist)
    }

    // MARK: - Bézier helpers

    private static func quadraticBezier(p0: Vector2D, p1: Vector2D, p2: Vector2D, t: Double) -> Vector2D {
        let mt = 1 - t
        return p0 * (mt * mt) + p1 * (2 * mt * t) + p2 * (t * t)
    }

    private static func cubicBezier(p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, t: Double) -> Vector2D {
        let mt = 1 - t
        let mt2 = mt * mt
        let t2 = t * t
        return p0 * (mt2 * mt) + p1 * (3 * mt2 * t) + p2 * (3 * mt * t2) + p3 * (t2 * t)
    }

    private static func adaptiveSteps(from p0: Vector2D, control1: Vector2D, end: Vector2D, tolerance: Double) -> Int {
        let d = p0.distance(to: control1) + control1.distance(to: end)
        return max(4, Int(d / tolerance))
    }

    private static func adaptiveStepsCubic(from p0: Vector2D, c1: Vector2D, c2: Vector2D, end: Vector2D, tolerance: Double) -> Int {
        let d = p0.distance(to: c1) + c1.distance(to: c2) + c2.distance(to: end)
        return max(4, Int(d / tolerance))
    }
}
