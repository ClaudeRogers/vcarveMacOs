import Foundation
import ClaudeCarveCore

// MARK: - Affine Transform

/// 2D affine transformation matrix.
///
/// Stored as the upper 2x3 portion of a 3x3 homogeneous matrix:
/// ```
/// | a  b  tx |
/// | c  d  ty |
/// | 0  0  1  |
/// ```
/// Applied to a point (x, y) as:
///   x' = a*x + b*y + tx
///   y' = c*x + d*y + ty
public struct AffineTransform2D: Codable, Sendable, Hashable {
    public var a: Double
    public var b: Double
    public var c: Double
    public var d: Double
    public var tx: Double
    public var ty: Double

    public init(a: Double, b: Double, c: Double, d: Double, tx: Double, ty: Double) {
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.tx = tx
        self.ty = ty
    }

    // MARK: - Identity

    public static let identity = AffineTransform2D(a: 1, b: 0, c: 0, d: 1, tx: 0, ty: 0)

    // MARK: - Factory methods

    /// Translation by (dx, dy).
    public static func translation(_ dx: Double, _ dy: Double) -> AffineTransform2D {
        AffineTransform2D(a: 1, b: 0, c: 0, d: 1, tx: dx, ty: dy)
    }

    /// Scale by (sx, sy) about the origin.
    public static func scale(_ sx: Double, _ sy: Double) -> AffineTransform2D {
        AffineTransform2D(a: sx, b: 0, c: 0, d: sy, tx: 0, ty: 0)
    }

    /// Rotation by angle (radians) counter-clockwise about the origin.
    public static func rotation(_ angle: Double) -> AffineTransform2D {
        let cosA = cos(angle)
        let sinA = sin(angle)
        return AffineTransform2D(a: cosA, b: -sinA, c: sinA, d: cosA, tx: 0, ty: 0)
    }

    /// Rotation by angle (radians) counter-clockwise about a given center point.
    public static func rotation(_ angle: Double, around center: Vector2D) -> AffineTransform2D {
        let toOrigin = translation(-center.x, -center.y)
        let rot = rotation(angle)
        let back = translation(center.x, center.y)
        return toOrigin.concatenated(with: rot).concatenated(with: back)
    }

    /// Mirror across an axis passing through a center point.
    public static func mirror(axis: MirrorAxis, around center: Vector2D) -> AffineTransform2D {
        let toOrigin = translation(-center.x, -center.y)
        let back = translation(center.x, center.y)

        let mirrorMatrix: AffineTransform2D
        switch axis {
        case .horizontal:
            // Mirror across X axis (flip Y)
            mirrorMatrix = scale(1, -1)
        case .vertical:
            // Mirror across Y axis (flip X)
            mirrorMatrix = scale(-1, 1)
        case .custom:
            // Default custom mirrors across the X axis
            mirrorMatrix = scale(1, -1)
        }

        return toOrigin.concatenated(with: mirrorMatrix).concatenated(with: back)
    }

    /// Shear transform.
    public static func shear(_ shx: Double, _ shy: Double) -> AffineTransform2D {
        AffineTransform2D(a: 1, b: shx, c: shy, d: 1, tx: 0, ty: 0)
    }

    // MARK: - Application

    /// Apply this transform to a 2D point.
    public func apply(to point: Vector2D) -> Vector2D {
        Vector2D(
            a * point.x + b * point.y + tx,
            c * point.x + d * point.y + ty
        )
    }

    // MARK: - Composition

    /// Concatenate this transform with another: self * other.
    /// The result applies `self` first, then `other`.
    public func concatenated(with other: AffineTransform2D) -> AffineTransform2D {
        AffineTransform2D(
            a:  other.a * a  + other.b * c,
            b:  other.a * b  + other.b * d,
            c:  other.c * a  + other.d * c,
            d:  other.c * b  + other.d * d,
            tx: other.a * tx + other.b * ty + other.tx,
            ty: other.c * tx + other.d * ty + other.ty
        )
    }

    // MARK: - Inverse

    /// The inverse of this transform, or `nil` if the matrix is singular (determinant is zero).
    public var inverse: AffineTransform2D? {
        let det = a * d - b * c
        guard abs(det) > 1e-15 else { return nil }
        let invDet = 1.0 / det
        return AffineTransform2D(
            a:   d * invDet,
            b:  -b * invDet,
            c:  -c * invDet,
            d:   a * invDet,
            tx: (b * ty - d * tx) * invDet,
            ty: (c * tx - a * ty) * invDet
        )
    }

    // MARK: - Classification helpers

    /// Whether this transform is a uniform scale + rotation + translation (no shear, no non-uniform scale).
    /// In this case, circles remain circles under transformation, so arcs can be directly transformed.
    var isConformal: Bool {
        // A conformal (angle-preserving) 2D affine transform has the form:
        //   | s*cos(t)  -s*sin(t)  tx |
        //   | s*sin(t)   s*cos(t)  ty |
        // or the reflection variant:
        //   |  s*cos(t)  s*sin(t)  tx |
        //   |  s*sin(t) -s*cos(t)  ty |
        // Check: a*a + c*c == b*b + d*d (uniform scaling) and a*b + c*d == 0 (orthogonality)
        let scaleXSq = a * a + c * c
        let scaleYSq = b * b + d * d
        let ortho = a * b + c * d
        return abs(scaleXSq - scaleYSq) < 1e-10 && abs(ortho) < 1e-10
    }

    /// The uniform scale factor for a conformal transform.
    var conformalScale: Double {
        sqrt(a * a + c * c)
    }

    /// The rotation angle (radians) for a conformal transform.
    var conformalAngle: Double {
        atan2(c, a)
    }

    /// Determinant of the linear part.
    var determinant: Double {
        a * d - b * c
    }
}

// MARK: - Mirror Axis

/// Axis for mirror operations.
public enum MirrorAxis: String, Codable, Sendable {
    /// Mirror across X axis (flip Y coordinates).
    case horizontal
    /// Mirror across Y axis (flip X coordinates).
    case vertical
    /// Custom angle mirror.
    case custom
}

// MARK: - Array Pattern

/// Patterns for creating arrays of path copies.
public enum ArrayPattern: Sendable {
    /// Linear array: copies spaced by (dx, dy).
    case linear(dx: Double, dy: Double, copies: Int)
    /// Circular array: copies around a center point.
    case circular(center: Vector2D, totalAngle: Double, copies: Int, rotateObjects: Bool)
    /// Rectangular grid array.
    case grid(dx: Double, dy: Double, columns: Int, rows: Int)
}

// MARK: - Alignment & Distribution

/// Alignment options for positioning paths relative to a reference.
public enum AlignmentOption: Sendable {
    case left, right, top, bottom, centerH, centerV
}

/// Direction for distributing paths evenly.
public enum DistributeDirection: Sendable {
    case horizontal, vertical
}

// MARK: - Vector Transforms

/// Utilities for transforming vector paths with affine transforms, arrays, alignment, and distribution.
public struct VectorTransforms: Sendable {

    // MARK: - Single path transform

    /// Apply an affine transform to a VectorPath.
    ///
    /// Handles all segment types. For arcs under non-conformal transforms (shear or non-uniform scale),
    /// the arc is first approximated by cubic Bezier segments, which are then transformed.
    public static func transform(_ path: VectorPath, by transform: AffineTransform2D) -> VectorPath {
        let newStart = transform.apply(to: path.startPoint)

        var newSegments: [PathSegment] = []

        for segment in path.segments {
            switch segment {
            case .lineTo(let p):
                newSegments.append(.lineTo(transform.apply(to: p)))

            case .quadTo(let control, let end):
                newSegments.append(.quadTo(
                    control: transform.apply(to: control),
                    end: transform.apply(to: end)
                ))

            case .cubicTo(let control1, let control2, let end):
                newSegments.append(.cubicTo(
                    control1: transform.apply(to: control1),
                    control2: transform.apply(to: control2),
                    end: transform.apply(to: end)
                ))

            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                if transform.isConformal {
                    // Conformal transform: circles remain circles.
                    let newCenter = transform.apply(to: center)
                    let newRadius = radius * transform.conformalScale
                    let rotAngle = transform.conformalAngle
                    let det = transform.determinant
                    // If the determinant is negative, the transform includes a reflection,
                    // which reverses winding direction.
                    let flipped = det < 0
                    let newStartAngle: Double
                    let newEndAngle: Double
                    let newClockwise: Bool
                    if flipped {
                        // Reflection flips angles: reflect angle about the rotation axis.
                        // For a reflection, angle -> -(angle) + rotAngle equivalent.
                        // More precisely: under reflection, the angle transforms differently.
                        // We compute by transforming the start/end points on the arc.
                        let startPt = center + Vector2D.fromAngle(startAngle) * radius
                        let endPt = center + Vector2D.fromAngle(endAngle) * radius
                        let newStartPt = transform.apply(to: startPt)
                        let newEndPt = transform.apply(to: endPt)
                        newStartAngle = (newStartPt - newCenter).angle
                        newEndAngle = (newEndPt - newCenter).angle
                        newClockwise = !clockwise
                    } else {
                        newStartAngle = startAngle + rotAngle
                        newEndAngle = endAngle + rotAngle
                        newClockwise = clockwise
                    }
                    newSegments.append(.arcTo(
                        center: newCenter,
                        radius: newRadius,
                        startAngle: newStartAngle,
                        endAngle: newEndAngle,
                        clockwise: newClockwise
                    ))
                } else {
                    // Non-conformal transform: arc becomes an elliptical arc.
                    // Approximate the arc with cubic Bezier segments, then transform those.
                    let cubics = approximateArcWithCubics(
                        center: center,
                        radius: radius,
                        startAngle: startAngle,
                        endAngle: endAngle,
                        clockwise: clockwise
                    )
                    for cubic in cubics {
                        newSegments.append(.cubicTo(
                            control1: transform.apply(to: cubic.c1),
                            control2: transform.apply(to: cubic.c2),
                            end: transform.apply(to: cubic.end)
                        ))
                    }
                }
            }
        }

        return VectorPath(
            id: path.id,
            startPoint: newStart,
            segments: newSegments,
            isClosed: path.isClosed
        )
    }

    // MARK: - Multiple paths transform

    /// Apply a transform to an array of paths.
    public static func transform(_ paths: [VectorPath], by transform: AffineTransform2D) -> [VectorPath] {
        paths.map { self.transform($0, by: transform) }
    }

    // MARK: - Scale to fit

    /// Scale paths to fit within a target bounding box, optionally maintaining aspect ratio.
    public static func scaleToFit(
        _ paths: [VectorPath],
        target: BoundingBox2D,
        maintainAspect: Bool
    ) -> [VectorPath] {
        guard !paths.isEmpty else { return [] }

        let sourceBB = combinedBoundingBox(of: paths)
        guard sourceBB.width > 1e-12 && sourceBB.height > 1e-12 else { return paths }
        guard target.width > 1e-12 && target.height > 1e-12 else { return paths }

        let sx = target.width / sourceBB.width
        let sy = target.height / sourceBB.height

        let finalSx: Double
        let finalSy: Double
        if maintainAspect {
            let s = Swift.min(sx, sy)
            finalSx = s
            finalSy = s
        } else {
            finalSx = sx
            finalSy = sy
        }

        // Move source center to origin, scale, then move to target center
        let sourceCenter = sourceBB.center
        let targetCenter = target.center

        let t = AffineTransform2D.translation(-sourceCenter.x, -sourceCenter.y)
            .concatenated(with: .scale(finalSx, finalSy))
            .concatenated(with: .translation(targetCenter.x, targetCenter.y))

        return transform(paths, by: t)
    }

    // MARK: - Array copies

    /// Create array copies of paths according to a pattern.
    ///
    /// The original paths are included as the first set. Additional copies follow.
    public static func array(_ paths: [VectorPath], pattern: ArrayPattern) -> [VectorPath] {
        guard !paths.isEmpty else { return [] }

        switch pattern {
        case .linear(let dx, let dy, let copies):
            return arrayLinear(paths, dx: dx, dy: dy, copies: copies)
        case .circular(let center, let totalAngle, let copies, let rotateObjects):
            return arrayCircular(paths, center: center, totalAngle: totalAngle, copies: copies, rotateObjects: rotateObjects)
        case .grid(let dx, let dy, let columns, let rows):
            return arrayGrid(paths, dx: dx, dy: dy, columns: columns, rows: rows)
        }
    }

    // MARK: - Alignment

    /// Align paths relative to a reference bounding box.
    ///
    /// Computes the combined bounding box of the input paths and translates all paths
    /// so that the specified edge or center matches the corresponding edge or center of the reference.
    public static func align(
        _ paths: [VectorPath],
        alignment: AlignmentOption,
        reference: BoundingBox2D
    ) -> [VectorPath] {
        guard !paths.isEmpty else { return [] }

        let pathsBB = combinedBoundingBox(of: paths)

        let dx: Double
        let dy: Double

        switch alignment {
        case .left:
            dx = reference.min.x - pathsBB.min.x
            dy = 0
        case .right:
            dx = reference.max.x - pathsBB.max.x
            dy = 0
        case .top:
            dx = 0
            dy = reference.max.y - pathsBB.max.y
        case .bottom:
            dx = 0
            dy = reference.min.y - pathsBB.min.y
        case .centerH:
            dx = reference.center.x - pathsBB.center.x
            dy = 0
        case .centerV:
            dx = 0
            dy = reference.center.y - pathsBB.center.y
        }

        guard abs(dx) > 1e-15 || abs(dy) > 1e-15 else { return paths }

        let t = AffineTransform2D.translation(dx, dy)
        return transform(paths, by: t)
    }

    // MARK: - Distribution

    /// Distribute paths evenly within a bounding box along the specified direction.
    ///
    /// Sorts paths by their center position along the distribution axis and spaces them
    /// evenly so that the first and last path centers align with the edges of the bounds.
    public static func distribute(
        _ paths: [VectorPath],
        direction: DistributeDirection,
        bounds: BoundingBox2D
    ) -> [VectorPath] {
        guard paths.count >= 2 else { return paths }

        // Pair each path with its bounding box and center
        struct PathInfo {
            let index: Int
            let path: VectorPath
            let bb: BoundingBox2D
            let center: Double
        }

        let infos: [PathInfo] = paths.enumerated().map { i, path in
            let bb = path.boundingBox
            let center: Double
            switch direction {
            case .horizontal:
                center = bb.center.x
            case .vertical:
                center = bb.center.y
            }
            return PathInfo(index: i, path: path, bb: bb, center: center)
        }

        let sorted = infos.sorted { $0.center < $1.center }

        // Compute the span for even distribution
        let startPos: Double
        let endPos: Double
        switch direction {
        case .horizontal:
            // Distribute centers from the leftmost edge + half first path width
            // to the rightmost edge - half last path width
            startPos = bounds.min.x + sorted.first!.bb.width / 2.0
            endPos = bounds.max.x - sorted.last!.bb.width / 2.0
        case .vertical:
            startPos = bounds.min.y + sorted.first!.bb.height / 2.0
            endPos = bounds.max.y - sorted.last!.bb.height / 2.0
        }

        let count = sorted.count
        let step = count > 1 ? (endPos - startPos) / Double(count - 1) : 0.0

        var result = paths  // Start with originals; we will replace in-place by original index
        for (i, info) in sorted.enumerated() {
            let targetCenter = startPos + step * Double(i)
            let offset = targetCenter - info.center
            guard abs(offset) > 1e-15 else {
                result[info.index] = info.path
                continue
            }

            let t: AffineTransform2D
            switch direction {
            case .horizontal:
                t = .translation(offset, 0)
            case .vertical:
                t = .translation(0, offset)
            }
            result[info.index] = transform(info.path, by: t)
        }

        return result
    }

    // MARK: - Private helpers

    /// Compute the combined bounding box of an array of paths.
    private static func combinedBoundingBox(of paths: [VectorPath]) -> BoundingBox2D {
        var bb = BoundingBox2D()
        for path in paths {
            bb.expand(toInclude: path.boundingBox)
        }
        return bb
    }

    /// Linear array: create evenly spaced copies along a direction.
    private static func arrayLinear(
        _ paths: [VectorPath],
        dx: Double,
        dy: Double,
        copies: Int
    ) -> [VectorPath] {
        var result: [VectorPath] = []
        // copies includes the original (copy 0 = original, copy 1..copies-1 = duplicates)
        for i in 0..<copies {
            let t = AffineTransform2D.translation(dx * Double(i), dy * Double(i))
            for path in paths {
                var transformed = transform(path, by: t)
                if i > 0 {
                    // Give duplicates unique IDs
                    transformed = VectorPath(
                        startPoint: transformed.startPoint,
                        segments: transformed.segments,
                        isClosed: transformed.isClosed
                    )
                }
                result.append(transformed)
            }
        }
        return result
    }

    /// Circular array: create copies arranged around a center point.
    private static func arrayCircular(
        _ paths: [VectorPath],
        center: Vector2D,
        totalAngle: Double,
        copies: Int,
        rotateObjects: Bool
    ) -> [VectorPath] {
        guard copies >= 1 else { return [] }

        var result: [VectorPath] = []

        // Distribute copies evenly over the total angle.
        // If totalAngle is a full circle (2*pi), we space copies evenly without doubling at start/end.
        let angleStep: Double
        let isFullCircle = abs(abs(totalAngle) - 2.0 * .pi) < 1e-10
        if isFullCircle {
            angleStep = totalAngle / Double(copies)
        } else {
            angleStep = copies > 1 ? totalAngle / Double(copies - 1) : 0.0
        }

        let pathsBB = combinedBoundingBox(of: paths)
        let pathsCenter = pathsBB.center

        for i in 0..<copies {
            let angle = angleStep * Double(i)

            if rotateObjects {
                // Rotate the entire object around the center point.
                // This both moves the object along the circle and rotates its orientation.
                let t = AffineTransform2D.rotation(angle, around: center)
                for path in paths {
                    var transformed = transform(path, by: t)
                    if i > 0 {
                        transformed = VectorPath(
                            startPoint: transformed.startPoint,
                            segments: transformed.segments,
                            isClosed: transformed.isClosed
                        )
                    }
                    result.append(transformed)
                }
            } else {
                // Move the object to its position on the circle but keep its orientation.
                // 1. Compute where the paths center should be on the circle.
                let relativeToCenter = pathsCenter - center
                let rotatedRelative = relativeToCenter.rotated(by: angle)
                let newCenter = center + rotatedRelative
                let offset = newCenter - pathsCenter

                let t = AffineTransform2D.translation(offset.x, offset.y)
                for path in paths {
                    var transformed = transform(path, by: t)
                    if i > 0 {
                        transformed = VectorPath(
                            startPoint: transformed.startPoint,
                            segments: transformed.segments,
                            isClosed: transformed.isClosed
                        )
                    }
                    result.append(transformed)
                }
            }
        }

        return result
    }

    /// Grid array: create copies in a rectangular grid.
    private static func arrayGrid(
        _ paths: [VectorPath],
        dx: Double,
        dy: Double,
        columns: Int,
        rows: Int
    ) -> [VectorPath] {
        var result: [VectorPath] = []
        for row in 0..<rows {
            for col in 0..<columns {
                let t = AffineTransform2D.translation(dx * Double(col), dy * Double(row))
                let isOriginal = row == 0 && col == 0
                for path in paths {
                    var transformed = transform(path, by: t)
                    if !isOriginal {
                        transformed = VectorPath(
                            startPoint: transformed.startPoint,
                            segments: transformed.segments,
                            isClosed: transformed.isClosed
                        )
                    }
                    result.append(transformed)
                }
            }
        }
        return result
    }
}

// MARK: - Arc to Cubic Bezier Approximation

/// A cubic Bezier segment represented by its two control points and endpoint.
/// (The start point is implied by the previous segment or path start.)
private struct CubicSegment {
    let c1: Vector2D
    let c2: Vector2D
    let end: Vector2D
}

/// Approximate a circular arc with one or more cubic Bezier segments.
///
/// Splits the arc into segments spanning at most pi/2 radians each for accuracy,
/// then uses the standard tangent-length method for each sub-arc.
private func approximateArcWithCubics(
    center: Vector2D,
    radius: Double,
    startAngle: Double,
    endAngle: Double,
    clockwise: Bool
) -> [CubicSegment] {
    // Compute sweep angle
    var sweep = endAngle - startAngle
    if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
    if !clockwise && sweep < 0 { sweep += 2.0 * .pi }

    // Handle zero-length arc
    guard abs(sweep) > 1e-12 else { return [] }

    // Split into segments of at most pi/2
    let maxSweep = Double.pi / 2.0
    let numSegments = max(1, Int(ceil(abs(sweep) / maxSweep)))
    let segSweep = sweep / Double(numSegments)

    var cubics: [CubicSegment] = []

    for i in 0..<numSegments {
        let a0 = startAngle + segSweep * Double(i)
        let a1 = a0 + segSweep

        let cubic = arcSegmentToCubic(center: center, radius: radius, angle0: a0, angle1: a1)
        cubics.append(cubic)
    }

    return cubics
}

/// Convert a single arc segment (sweep <= pi/2) into a cubic Bezier.
///
/// Uses the standard formula:
///   alpha = 4/3 * tan(sweep/4)
/// to compute control point offsets from the arc endpoints along the tangent directions.
private func arcSegmentToCubic(
    center: Vector2D,
    radius: Double,
    angle0: Double,
    angle1: Double
) -> CubicSegment {
    let sweep = angle1 - angle0

    // Endpoints on the arc
    let p0 = center + Vector2D(cos(angle0), sin(angle0)) * radius
    let p3 = center + Vector2D(cos(angle1), sin(angle1)) * radius

    // Alpha factor for cubic approximation of a circular arc
    let alpha = 4.0 / 3.0 * tan(sweep / 4.0)

    // Tangent directions at start and end (perpendicular to the radial direction)
    let t0 = Vector2D(-sin(angle0), cos(angle0))
    let t1 = Vector2D(-sin(angle1), cos(angle1))

    let c1 = p0 + t0 * (alpha * radius)
    let c2 = p3 - t1 * (alpha * radius)

    return CubicSegment(c1: c1, c2: c2, end: p3)
}
