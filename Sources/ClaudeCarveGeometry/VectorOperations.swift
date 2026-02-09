import Foundation
import ClaudeCarveCore

// MARK: - Vector Editing Operations

/// Provides vector editing operations commonly used in CNC CAD/CAM workflows:
/// welding (boolean union), trimming, filleting, chamfering, and path manipulation.
///
/// All operations work on `VectorPath` instances and produce new paths without
/// mutating inputs. Geometry is flattened to line segments for boolean and
/// intersection operations, then reconstructed as `VectorPath` with `.lineTo`
/// segments. Arc-based fillets produce `.arcTo` segments for smooth toolpaths.
public struct VectorOperations: Sendable {

    // MARK: - Tolerances

    /// Spatial coincidence tolerance (user units, typically mm or inches).
    private static let epsilon: Double = 1e-9

    /// Squared epsilon for distance comparisons without sqrt.
    private static let epsilonSq: Double = 1e-18

    /// Tolerance for parametric values along edges [0, 1].
    private static let paramEpsilon: Double = 1e-10

    /// Tolerance for cross-product collinearity checks.
    private static let crossEpsilon: Double = 1e-10

    /// Default flattening tolerance for converting curves to line segments.
    private static let flattenTolerance: Double = 0.01

    // MARK: - Welding (Union)

    /// Weld (boolean union) multiple overlapping closed paths into a single outline.
    ///
    /// Flattens all paths to line-segment polygons, computes pairwise intersections,
    /// and builds the union boundary by walking the outermost contour. Segments that
    /// lie inside any other polygon are discarded.
    ///
    /// - Parameter paths: Closed `VectorPath` instances to union together.
    /// - Returns: One or more paths representing the merged outer boundary.
    public static func weld(_ paths: [VectorPath]) -> [VectorPath] {
        let closed = paths.filter { $0.isClosed && $0.segments.count >= 2 }
        guard closed.count >= 2 else { return paths }

        // Flatten each path to a polygon
        var polygons = closed.map { $0.flattenedPoints(tolerance: flattenTolerance) }

        // Ensure all polygons have at least 3 vertices
        polygons = polygons.filter { $0.count >= 3 }
        guard polygons.count >= 2 else { return paths }

        // Iteratively union polygons pairwise
        var result = [polygons[0]]

        for i in 1..<polygons.count {
            let polyB = polygons[i]
            var newResult: [[Vector2D]] = []

            var merged = false
            for existing in result {
                if !merged {
                    let unionResult = unionPolygons(existing, polyB)
                    if unionResult.count == 1 {
                        // Successful merge into one polygon
                        newResult.append(unionResult[0])
                        merged = true
                    } else if unionResult.count == 2 {
                        // Disjoint - keep existing, will add polyB after
                        newResult.append(existing)
                    } else {
                        newResult.append(existing)
                    }
                } else {
                    newResult.append(existing)
                }
            }

            if !merged {
                newResult.append(polyB)
            }

            result = newResult
        }

        return result.compactMap { polygonToPath($0, closed: true) }
    }

    // MARK: - Trimming

    /// Trim paths at their intersections, keeping specified regions.
    ///
    /// Finds all pairwise intersection points, splits each path at those points,
    /// then classifies each resulting segment as inside or outside the union of
    /// the other paths using point-in-polygon testing.
    ///
    /// - Parameters:
    ///   - paths: Paths to trim against each other.
    ///   - keepInside: If `true`, keep segments inside other paths; if `false`, keep outside segments.
    /// - Returns: The individual trimmed segments as separate paths.
    public static func trim(_ paths: [VectorPath], keepInside: Bool = true) -> [VectorPath] {
        guard paths.count >= 2 else { return paths }

        let polygons = paths.map { $0.flattenedPoints(tolerance: flattenTolerance) }
        var resultPaths: [VectorPath] = []

        for i in 0..<polygons.count {
            let polyA = polygons[i]
            guard polyA.count >= 2 else { continue }

            // Collect all intersections with other paths
            var splitPoints: [(Int, Double)] = [] // (segmentIndex, parameter)

            for j in 0..<polygons.count where j != i {
                let polyB = polygons[j]
                let intersections = findPolygonIntersections(polyA, polyB, closedA: paths[i].isClosed, closedB: paths[j].isClosed)
                for inter in intersections {
                    splitPoints.append((inter.edgeA, inter.paramA))
                }
            }

            // Split the path at intersection points
            let segments = splitPolygonAtParameters(polyA, splitPoints: splitPoints, isClosed: paths[i].isClosed)

            // Classify each segment as inside/outside the other paths
            for segment in segments {
                guard segment.count >= 2 else { continue }
                let midpoint = segmentMidpoint(segment)

                var isInsideAny = false
                for j in 0..<polygons.count where j != i {
                    if paths[j].isClosed && polygons[j].count >= 3 {
                        if pointInPolygon(midpoint, polygons[j]) {
                            isInsideAny = true
                            break
                        }
                    }
                }

                if isInsideAny == keepInside {
                    if let path = polygonToPath(segment, closed: false) {
                        resultPaths.append(path)
                    }
                }
            }
        }

        return resultPaths
    }

    /// Trim a path to keep only the portion inside a boundary.
    ///
    /// The boundary must be a closed path. Segments of the input path that fall
    /// outside the boundary polygon are discarded.
    ///
    /// - Parameters:
    ///   - path: The path to trim.
    ///   - boundary: A closed path defining the clipping region.
    /// - Returns: Path segments that lie inside the boundary.
    public static func trimToBoundary(_ path: VectorPath, boundary: VectorPath) -> [VectorPath] {
        guard boundary.isClosed else { return [path] }

        let pathPoly = path.flattenedPoints(tolerance: flattenTolerance)
        let boundaryPoly = boundary.flattenedPoints(tolerance: flattenTolerance)
        guard pathPoly.count >= 2, boundaryPoly.count >= 3 else { return [path] }

        // Find intersections between path and boundary
        let intersections = findPolygonIntersections(pathPoly, boundaryPoly, closedA: path.isClosed, closedB: true)

        // Split the path at intersection points
        var splitPoints: [(Int, Double)] = []
        for inter in intersections {
            splitPoints.append((inter.edgeA, inter.paramA))
        }

        let segments = splitPolygonAtParameters(pathPoly, splitPoints: splitPoints, isClosed: path.isClosed)

        // Keep only segments whose midpoint is inside the boundary
        var result: [VectorPath] = []
        for segment in segments {
            guard segment.count >= 2 else { continue }
            let mid = segmentMidpoint(segment)
            if pointInPolygon(mid, boundaryPoly) {
                if let p = polygonToPath(segment, closed: false) {
                    result.append(p)
                }
            }
        }

        return result
    }

    /// Extend open paths to meet each other or a boundary.
    ///
    /// Each open path's endpoints are extended along their tangent direction until
    /// they intersect the boundary (if provided) or another path. If no intersection
    /// is found within a reasonable extension distance, the path is returned unchanged.
    ///
    /// - Parameters:
    ///   - paths: Open paths to extend.
    ///   - boundary: Optional closed boundary path to extend to.
    /// - Returns: Extended paths.
    public static func extend(_ paths: [VectorPath], toBoundary boundary: VectorPath?) -> [VectorPath] {
        var result: [VectorPath] = []
        let maxExtension = 1000.0 // Maximum extension distance

        let boundaryPoly: [Vector2D]?
        if let b = boundary, b.isClosed {
            boundaryPoly = b.flattenedPoints(tolerance: flattenTolerance)
        } else {
            boundaryPoly = nil
        }

        // Flatten all paths for intersection testing
        let allPolys = paths.map { $0.flattenedPoints(tolerance: flattenTolerance) }

        for (idx, path) in paths.enumerated() {
            guard !path.isClosed else {
                result.append(path)
                continue
            }

            let poly = allPolys[idx]
            guard poly.count >= 2 else {
                result.append(path)
                continue
            }

            var extendedPoly = poly

            // Extend start point backward
            let startDir = (poly[0] - poly[1]).normalized

            if let hitPoint = findFirstIntersectionAlongRay(
                from: poly[0], direction: startDir, maxDistance: maxExtension,
                polygons: allPolys, excludeIndex: idx,
                boundary: boundaryPoly
            ) {
                extendedPoly.insert(hitPoint, at: 0)
            }

            // Extend end point forward
            let lastIdx = poly.count - 1
            let endDir = (poly[lastIdx] - poly[lastIdx - 1]).normalized

            if let hitPoint = findFirstIntersectionAlongRay(
                from: poly[lastIdx], direction: endDir, maxDistance: maxExtension,
                polygons: allPolys, excludeIndex: idx,
                boundary: boundaryPoly
            ) {
                extendedPoly.append(hitPoint)
            }

            if let p = polygonToPath(extendedPoly, closed: false) {
                result.append(p)
            }
        }

        return result
    }

    // MARK: - Filleting (Rounding Corners)

    /// Add fillets (rounded corners) to a path at each vertex.
    ///
    /// At each vertex where the angle between adjacent segments is not a straight
    /// line, a circular arc tangent to both edges is inserted at the specified
    /// radius distance from the vertex.
    ///
    /// The fillet center is computed at `radius / sin(halfAngle)` along the angle
    /// bisector. The resulting arc connects the two tangent points on the adjacent
    /// edges.
    ///
    /// - Parameters:
    ///   - path: The path to fillet.
    ///   - radius: The fillet radius (must be positive).
    /// - Returns: A new path with arcs inserted at each eligible corner.
    public static func fillet(_ path: VectorPath, radius: Double) -> VectorPath {
        guard radius > epsilon else { return path }

        let points = path.flattenedPoints(tolerance: flattenTolerance)
        guard points.count >= 3 else { return path }

        let isClosed = path.isClosed
        let count = points.count

        var newStart: Vector2D?
        var newSegments: [PathSegment] = []

        // Determine vertex range: for closed paths, process all vertices;
        // for open paths, skip start and end vertices.
        let startIndex = isClosed ? 0 : 1
        let endIndex = isClosed ? count : count - 1

        // If open path, the first point is the start and we begin with a line to the
        // first fillet tangent point.
        if !isClosed {
            newStart = points[0]
        }

        for i in startIndex..<endIndex {
            let prevIdx: Int
            let nextIdx: Int

            if isClosed {
                prevIdx = (i - 1 + count) % count
                nextIdx = (i + 1) % count
            } else {
                prevIdx = i - 1
                nextIdx = i + 1
            }

            let prev = points[prevIdx]
            let curr = points[i]
            let next = points[nextIdx]

            let dirIn = (curr - prev).normalized
            let dirOut = (next - curr).normalized

            // Cross product determines turn direction; dot product gives angle
            let cross = dirIn.cross(dirOut)
            let dot = dirIn.dot(dirOut)

            // Skip nearly straight segments (angle close to 0 or pi)
            let sinAngle = abs(cross)
            if sinAngle < 1e-6 {
                // Straight or near-straight: no fillet
                if newStart == nil {
                    newStart = curr
                } else {
                    newSegments.append(.lineTo(curr))
                }
                continue
            }

            // Half-angle between the two edges
            let angle = atan2(sinAngle, dot)
            let halfAngle = angle / 2.0

            // Distance from vertex to tangent point along each edge
            let tangentDist = radius / tan(halfAngle)

            // Check that the tangent distance does not exceed edge lengths
            let edgeInLen = prev.distance(to: curr)
            let edgeOutLen = curr.distance(to: next)
            let maxTangentDist = min(edgeInLen, edgeOutLen) * 0.5

            let effectiveTangentDist = min(tangentDist, maxTangentDist)
            let effectiveRadius: Double
            if tangentDist > maxTangentDist {
                // Reduce radius to fit
                effectiveRadius = effectiveTangentDist * tan(halfAngle)
            } else {
                effectiveRadius = radius
            }

            // Tangent points on the incoming and outgoing edges
            let tangentIn = curr - dirIn * effectiveTangentDist
            let tangentOut = curr + dirOut * effectiveTangentDist

            // Fillet center: offset from vertex along the bisector
            let bisector = (-dirIn + dirOut).normalized
            // The center is at distance radius/sin(halfAngle) from the vertex,
            // but on the inside of the angle.
            let centerDist = effectiveRadius / sin(halfAngle)

            // Determine which side the center is on
            let center: Vector2D
            if cross > 0 {
                // Left turn (CCW) - center is to the right of the bisector
                center = curr + bisector * centerDist
            } else {
                // Right turn (CW) - center is to the left of the bisector
                center = curr + bisector * centerDist
            }

            // Compute start and end angles for the arc
            let startAngle = (tangentIn - center).angle
            let endAngle = (tangentOut - center).angle

            // Determine arc direction: the arc should be on the inside of the corner
            let isClockwise = cross > 0

            // Add line to tangent point, then arc, then continue
            if newStart == nil {
                newStart = tangentIn
            } else {
                newSegments.append(.lineTo(tangentIn))
            }

            newSegments.append(.arcTo(
                center: center,
                radius: effectiveRadius,
                startAngle: startAngle,
                endAngle: endAngle,
                clockwise: isClockwise
            ))

            // If this is not the last vertex being processed, we will add
            // a line to the next tangent-in point in the next iteration.
            // The tangentOut point is already the endpoint of the arc.
        }

        // For open paths, add the final point
        if !isClosed {
            newSegments.append(.lineTo(points[count - 1]))
        }

        return VectorPath(
            startPoint: newStart ?? points[0],
            segments: newSegments,
            isClosed: isClosed
        )
    }

    /// Add chamfers (straight-cut corners) to a path at each vertex.
    ///
    /// Similar to filleting, but instead of inserting a circular arc, a straight
    /// line connects two points at `distance` from the vertex along each adjacent
    /// edge.
    ///
    /// - Parameters:
    ///   - path: The path to chamfer.
    ///   - distance: The chamfer distance from each vertex (must be positive).
    /// - Returns: A new path with chamfered corners.
    public static func chamfer(_ path: VectorPath, distance: Double) -> VectorPath {
        guard distance > epsilon else { return path }

        let points = path.flattenedPoints(tolerance: flattenTolerance)
        guard points.count >= 3 else { return path }

        let isClosed = path.isClosed
        let count = points.count

        var newStart: Vector2D?
        var newSegments: [PathSegment] = []

        let startIndex = isClosed ? 0 : 1
        let endIndex = isClosed ? count : count - 1

        if !isClosed {
            newStart = points[0]
        }

        for i in startIndex..<endIndex {
            let prevIdx: Int
            let nextIdx: Int

            if isClosed {
                prevIdx = (i - 1 + count) % count
                nextIdx = (i + 1) % count
            } else {
                prevIdx = i - 1
                nextIdx = i + 1
            }

            let prev = points[prevIdx]
            let curr = points[i]
            let next = points[nextIdx]

            let dirIn = (curr - prev).normalized
            let dirOut = (next - curr).normalized

            let cross = dirIn.cross(dirOut)

            // Skip nearly straight segments
            if abs(cross) < 1e-6 {
                if newStart == nil {
                    newStart = curr
                } else {
                    newSegments.append(.lineTo(curr))
                }
                continue
            }

            // Distance from vertex to chamfer point along each edge
            let edgeInLen = prev.distance(to: curr)
            let edgeOutLen = curr.distance(to: next)
            let maxDist = min(edgeInLen, edgeOutLen) * 0.5
            let effectiveDist = min(distance, maxDist)

            let chamferIn = curr - dirIn * effectiveDist
            let chamferOut = curr + dirOut * effectiveDist

            if newStart == nil {
                newStart = chamferIn
            } else {
                newSegments.append(.lineTo(chamferIn))
            }

            // Straight line chamfer
            newSegments.append(.lineTo(chamferOut))
        }

        if !isClosed {
            newSegments.append(.lineTo(points[count - 1]))
        }

        return VectorPath(
            startPoint: newStart ?? points[0],
            segments: newSegments,
            isClosed: isClosed
        )
    }

    // MARK: - Path Operations

    /// Join open paths end-to-end where endpoints are within tolerance.
    ///
    /// Examines all pairs of open path endpoints and merges paths when:
    /// - An endpoint of one path is within `tolerance` of the startpoint of another (append).
    /// - Two startpoints are within tolerance (reverse one, then append).
    /// - Two endpoints are within tolerance (reverse one, then append).
    ///
    /// The process repeats until no more joins can be made.
    ///
    /// - Parameters:
    ///   - paths: Paths to attempt to join.
    ///   - tolerance: Maximum distance between endpoints to consider them joinable.
    /// - Returns: Joined paths (some may remain un-joined if no matching endpoints exist).
    public static func joinPaths(_ paths: [VectorPath], tolerance: Double = 0.1) -> [VectorPath] {
        let tolSq = tolerance * tolerance

        // Separate closed paths (which cannot be joined) from open paths
        var closedPaths = paths.filter { $0.isClosed }
        var openPaths = paths.filter { !$0.isClosed }

        guard openPaths.count >= 2 else { return paths }

        var changed = true
        while changed {
            changed = false

            outerLoop: for i in 0..<openPaths.count {
                for j in (i + 1)..<openPaths.count {
                    let pathA = openPaths[i]
                    let pathB = openPaths[j]

                    let startA = pathA.startPoint
                    let endA = pathA.segments.last?.endPoint ?? startA
                    let startB = pathB.startPoint
                    let endB = pathB.segments.last?.endPoint ?? startB

                    var merged: VectorPath?

                    if endA.distanceSquared(to: startB) <= tolSq {
                        // endA -> startB: append B to A
                        merged = appendPaths(pathA, pathB)
                    } else if endB.distanceSquared(to: startA) <= tolSq {
                        // endB -> startA: append A to B
                        merged = appendPaths(pathB, pathA)
                    } else if startA.distanceSquared(to: startB) <= tolSq {
                        // startA ~ startB: reverse A, then append B
                        let reversedA = pathA.reversed()
                        merged = appendPaths(reversedA, pathB)
                    } else if endA.distanceSquared(to: endB) <= tolSq {
                        // endA ~ endB: append reversed B to A
                        let reversedB = pathB.reversed()
                        merged = appendPaths(pathA, reversedB)
                    }

                    if let mergedPath = merged {
                        openPaths.remove(at: j)
                        openPaths.remove(at: i)
                        openPaths.insert(mergedPath, at: i)
                        changed = true
                        break outerLoop
                    }
                }
            }
        }

        // Check if any joined open paths now form a closed loop
        var finalOpen: [VectorPath] = []
        for var path in openPaths {
            let start = path.startPoint
            let end = path.segments.last?.endPoint ?? start
            if start.distanceSquared(to: end) <= tolSq && path.segments.count >= 2 {
                path = VectorPath(
                    id: path.id,
                    startPoint: path.startPoint,
                    segments: path.segments,
                    isClosed: true
                )
                closedPaths.append(path)
            } else {
                finalOpen.append(path)
            }
        }

        return closedPaths + finalOpen
    }

    /// Close open paths by adding a closing segment from the last point back to the start.
    ///
    /// Paths that are already closed are returned unchanged.
    ///
    /// - Parameter paths: Paths to close.
    /// - Returns: Closed versions of the input paths.
    public static func closePaths(_ paths: [VectorPath]) -> [VectorPath] {
        return paths.map { path in
            guard !path.isClosed else { return path }
            guard !path.segments.isEmpty else { return path }

            var segments = path.segments

            // Add closing segment if the last endpoint is not already at the start
            let lastEnd = segments.last?.endPoint ?? path.startPoint
            if lastEnd.distanceSquared(to: path.startPoint) > epsilonSq {
                segments.append(.lineTo(path.startPoint))
            }

            return VectorPath(
                id: path.id,
                startPoint: path.startPoint,
                segments: segments,
                isClosed: true
            )
        }
    }

    /// Split a closed path at specified points into open sub-paths.
    ///
    /// Each split point is matched to the nearest point on the path within
    /// `tolerance`. The path is then divided at those locations.
    ///
    /// - Parameters:
    ///   - path: The closed path to split.
    ///   - points: Points at which to split the path.
    ///   - tolerance: Maximum distance to snap a split point to the path.
    /// - Returns: Open sub-paths resulting from the split.
    public static func split(_ path: VectorPath, at points: [Vector2D], tolerance: Double = 0.5) -> [VectorPath] {
        guard !points.isEmpty else { return [path] }

        let poly = path.flattenedPoints(tolerance: flattenTolerance)
        guard poly.count >= 3 else { return [path] }

        let isClosed = path.isClosed
        let tolSq = tolerance * tolerance
        let segCount = isClosed ? poly.count : poly.count - 1

        // Find parameters for each split point on the polygon edges
        var splitParams: [(Int, Double, Vector2D)] = [] // (edgeIndex, parameter, exact point)

        for splitPt in points {
            var bestDist = Double.infinity
            var bestEdge = -1
            var bestParam = 0.0
            var bestPoint = splitPt

            for i in 0..<segCount {
                let j = (i + 1) % poly.count
                let (param, closest) = closestPointOnSegment(splitPt, poly[i], poly[j])
                let dist = splitPt.distanceSquared(to: closest)
                if dist < bestDist {
                    bestDist = dist
                    bestEdge = i
                    bestParam = param
                    bestPoint = closest
                }
            }

            if bestDist <= tolSq && bestEdge >= 0 {
                splitParams.append((bestEdge, bestParam, bestPoint))
            }
        }

        guard !splitParams.isEmpty else { return [path] }

        // Sort split parameters by edge index then parameter
        splitParams.sort { a, b in
            if a.0 != b.0 { return a.0 < b.0 }
            return a.1 < b.1
        }

        // Remove near-duplicate split points
        var uniqueSplits: [(Int, Double, Vector2D)] = []
        for sp in splitParams {
            if let last = uniqueSplits.last {
                if last.0 == sp.0 && abs(last.1 - sp.1) < paramEpsilon {
                    continue
                }
            }
            uniqueSplits.append(sp)
        }

        guard uniqueSplits.count >= (isClosed ? 2 : 1) else {
            if !isClosed {
                return splitOpenPath(poly: poly, splits: uniqueSplits)
            }
            return [path]
        }

        return splitClosedPath(poly: poly, splits: uniqueSplits)
    }

    // MARK: - Vector Validation

    /// Check for self-intersections in a path.
    ///
    /// Flattens the path to line segments and tests all non-adjacent segment pairs
    /// for intersections.
    ///
    /// - Parameter path: The path to check.
    /// - Returns: Intersection points where the path crosses itself.
    public static func findSelfIntersections(_ path: VectorPath) -> [Vector2D] {
        let poly = path.flattenedPoints(tolerance: flattenTolerance)
        guard poly.count >= 4 else { return [] }

        let isClosed = path.isClosed
        let segCount = isClosed ? poly.count : poly.count - 1
        var intersections: [Vector2D] = []

        for i in 0..<segCount {
            let a1 = poly[i]
            let a2 = poly[(i + 1) % poly.count]

            // Start j at i+2 to skip adjacent segments
            for j in (i + 2)..<segCount {
                // Skip the pair (last, first) for closed polygons since they share a vertex
                if isClosed && i == 0 && j == segCount - 1 { continue }

                let b1 = poly[j]
                let b2 = poly[(j + 1) % poly.count]

                if let (t, u) = segmentIntersection(a1, a2, b1, b2) {
                    // Exclude intersections at shared endpoints
                    if (t < paramEpsilon || t > 1.0 - paramEpsilon) &&
                       (u < paramEpsilon || u > 1.0 - paramEpsilon) {
                        continue
                    }
                    let point = a1.lerp(to: a2, t: t)

                    // Deduplicate
                    let isDuplicate = intersections.contains { $0.distanceSquared(to: point) < epsilonSq }
                    if !isDuplicate {
                        intersections.append(point)
                    }
                }
            }
        }

        return intersections
    }

    /// Check for intersections between two paths.
    ///
    /// Flattens both paths and tests all segment pairs for intersections.
    ///
    /// - Parameters:
    ///   - path1: First path.
    ///   - path2: Second path.
    /// - Returns: All intersection points between the two paths.
    public static func findIntersections(_ path1: VectorPath, _ path2: VectorPath) -> [Vector2D] {
        let poly1 = path1.flattenedPoints(tolerance: flattenTolerance)
        let poly2 = path2.flattenedPoints(tolerance: flattenTolerance)

        let intersections = findPolygonIntersections(
            poly1, poly2,
            closedA: path1.isClosed,
            closedB: path2.isClosed
        )

        // Deduplicate
        var result: [Vector2D] = []
        for inter in intersections {
            let isDuplicate = result.contains { $0.distanceSquared(to: inter.point) < epsilonSq }
            if !isDuplicate {
                result.append(inter.point)
            }
        }

        return result
    }

    /// Remove duplicate or overlapping paths.
    ///
    /// Two paths are considered duplicates if their flattened points are all within
    /// `tolerance` of each other (accounting for potential start-point offsets in
    /// closed paths).
    ///
    /// - Parameters:
    ///   - paths: Paths to deduplicate.
    ///   - tolerance: Maximum point-to-point distance to consider paths as duplicates.
    /// - Returns: Paths with duplicates removed.
    public static func removeDuplicates(_ paths: [VectorPath], tolerance: Double = 0.01) -> [VectorPath] {
        let tolSq = tolerance * tolerance
        var result: [VectorPath] = []

        for path in paths {
            let poly = path.flattenedPoints(tolerance: flattenTolerance)

            var isDuplicate = false
            for existing in result {
                let existingPoly = existing.flattenedPoints(tolerance: flattenTolerance)

                if arePolygonsDuplicate(poly, existingPoly, toleranceSq: tolSq, closedA: path.isClosed, closedB: existing.isClosed) {
                    isDuplicate = true
                    break
                }
            }

            if !isDuplicate {
                result.append(path)
            }
        }

        return result
    }

    /// Fix winding order of paths for CNC conventions.
    ///
    /// Outer boundaries are set to counter-clockwise (positive signed area).
    /// Inner boundaries (holes) are set to clockwise (negative signed area).
    /// Containment is detected by testing whether a point of one path lies inside
    /// another.
    ///
    /// - Parameter paths: Paths to fix.
    /// - Returns: Paths with corrected winding order.
    public static func fixWindingOrder(_ paths: [VectorPath]) -> [VectorPath] {
        guard !paths.isEmpty else { return [] }

        let closedPaths = paths.filter { $0.isClosed }
        let openPaths = paths.filter { !$0.isClosed }

        // Determine containment hierarchy
        var isHole = [Bool](repeating: false, count: closedPaths.count)
        let polys = closedPaths.map { $0.flattenedPoints(tolerance: flattenTolerance) }

        for i in 0..<closedPaths.count {
            guard polys[i].count >= 3 else { continue }

            // Count how many other polygons contain this one
            var containmentCount = 0
            let testPoint = polys[i][0]

            for j in 0..<closedPaths.count where j != i {
                guard polys[j].count >= 3 else { continue }
                if pointInPolygon(testPoint, polys[j]) {
                    containmentCount += 1
                }
            }

            // Odd containment depth = hole, even = outer boundary
            isHole[i] = (containmentCount % 2) != 0
        }

        // Fix winding order
        var result: [VectorPath] = []
        for i in 0..<closedPaths.count {
            let path = closedPaths[i]
            let area = path.signedArea

            if isHole[i] {
                // Holes should be CW (negative area)
                if area > 0 {
                    result.append(path.reversed())
                } else {
                    result.append(path)
                }
            } else {
                // Outer boundaries should be CCW (positive area)
                if area < 0 {
                    result.append(path.reversed())
                } else {
                    result.append(path)
                }
            }
        }

        // Open paths are returned unchanged
        return result + openPaths
    }

    // MARK: - Private Helpers: Polygon Boolean Union

    /// Union two polygons into one or more result polygons.
    private static func unionPolygons(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        guard a.count >= 3, b.count >= 3 else {
            if a.count >= 3 { return [a] }
            if b.count >= 3 { return [b] }
            return []
        }

        let intersections = findPolygonIntersections(a, b, closedA: true, closedB: true)

        if intersections.isEmpty {
            // No intersections: check containment
            if pointInPolygon(b[0], a) {
                return [a] // b inside a
            } else if pointInPolygon(a[0], b) {
                return [b] // a inside b
            } else {
                return [a, b] // disjoint
            }
        }

        // Build union by tracing the outermost boundary
        let result = traceUnionBoundary(a, b, intersections: intersections)
        if result.isEmpty {
            return [a, b]
        }
        return result
    }

    /// Trace the outer union boundary of two intersecting polygons.
    ///
    /// Walks along polygon edges, switching polygons at intersection points,
    /// always choosing the segment that stays outside the other polygon.
    private static func traceUnionBoundary(_ a: [Vector2D], _ b: [Vector2D], intersections: [IntersectionInfo]) -> [[Vector2D]] {
        guard !intersections.isEmpty else { return [] }

        // Build augmented edge lists with intersection points inserted
        let augA = buildAugmentedEdges(a, intersections: intersections, isA: true)
        let augB = buildAugmentedEdges(b, intersections: intersections, isA: false)

        // Find a starting intersection where we are leaving polygon B (going outside B)
        // This means the segment of A just after this intersection is outside B.
        var startIdx = -1
        for (i, inter) in intersections.enumerated() {
            let nextPointOnA = pointSlightlyAfter(inter.point, inter.edgeA, inter.paramA, a, closed: true)
            if !pointInPolygon(nextPointOnA, b) {
                startIdx = i
                break
            }
        }

        if startIdx < 0 {
            // Try reverse: look for intersection where A exits B
            for (i, inter) in intersections.enumerated() {
                let nextPointOnB = pointSlightlyAfter(inter.point, inter.edgeB, inter.paramB, b, closed: true)
                if !pointInPolygon(nextPointOnB, a) {
                    startIdx = i
                    break
                }
            }
        }

        guard startIdx >= 0 else { return [] }

        // Trace the boundary
        var polygon: [Vector2D] = []
        var visited = Set<Int>()
        var currentInter = startIdx
        var onA = true
        let maxIterations = intersections.count * 2 + a.count + b.count

        for _ in 0..<maxIterations {
            if visited.contains(currentInter) && !polygon.isEmpty {
                break
            }
            visited.insert(currentInter)

            let inter = intersections[currentInter]
            appendDistinct(&polygon, inter.point)

            if onA {
                // Walk along A until next intersection
                let (points, nextInterIdx) = walkUntilIntersection(
                    from: inter.point, edgeIndex: inter.edgeA, param: inter.paramA,
                    polygon: a, augmented: augA, intersections: intersections, isA: true
                )
                for p in points { appendDistinct(&polygon, p) }

                if let next = nextInterIdx {
                    currentInter = next
                    // Decide whether to continue on A or switch to B
                    let nextInter = intersections[next]
                    let afterOnA = pointSlightlyAfter(nextInter.point, nextInter.edgeA, nextInter.paramA, a, closed: true)
                    if pointInPolygon(afterOnA, b) {
                        // Going inside B: switch to B
                        onA = false
                    }
                } else {
                    break
                }
            } else {
                // Walk along B until next intersection
                let (points, nextInterIdx) = walkUntilIntersection(
                    from: inter.point, edgeIndex: inter.edgeB, param: inter.paramB,
                    polygon: b, augmented: augB, intersections: intersections, isA: false
                )
                for p in points { appendDistinct(&polygon, p) }

                if let next = nextInterIdx {
                    currentInter = next
                    let nextInter = intersections[next]
                    let afterOnB = pointSlightlyAfter(nextInter.point, nextInter.edgeB, nextInter.paramB, b, closed: true)
                    if pointInPolygon(afterOnB, a) {
                        // Going inside A: switch to A
                        onA = true
                    }
                } else {
                    break
                }
            }

            if currentInter == startIdx {
                break
            }
        }

        if polygon.count >= 3 {
            return [polygon]
        }
        return []
    }

    // MARK: - Private Helpers: Intersection Finding

    /// Raw intersection data between two polygon edge sets.
    private struct IntersectionInfo: Sendable {
        let point: Vector2D
        let edgeA: Int
        let paramA: Double
        let edgeB: Int
        let paramB: Double
    }

    /// Find all intersection points between edges of two polygons.
    private static func findPolygonIntersections(
        _ a: [Vector2D], _ b: [Vector2D],
        closedA: Bool, closedB: Bool
    ) -> [IntersectionInfo] {
        var results: [IntersectionInfo] = []

        let segCountA = closedA ? a.count : a.count - 1
        let segCountB = closedB ? b.count : b.count - 1

        for i in 0..<segCountA {
            let a1 = a[i]
            let a2 = a[(i + 1) % a.count]

            for j in 0..<segCountB {
                let b1 = b[j]
                let b2 = b[(j + 1) % b.count]

                if let (t, u) = segmentIntersection(a1, a2, b1, b2) {
                    let point = a1.lerp(to: a2, t: t)

                    // Deduplicate
                    let isDuplicate = results.contains { $0.point.distanceSquared(to: point) < epsilonSq }
                    if !isDuplicate {
                        results.append(IntersectionInfo(
                            point: point,
                            edgeA: i,
                            paramA: t,
                            edgeB: j,
                            paramB: u
                        ))
                    }
                }
            }
        }

        return results
    }

    /// Compute the intersection of two line segments.
    /// Returns `(t, u)` parametric values in `[0, 1]`, or `nil` if no intersection.
    private static func segmentIntersection(
        _ p1: Vector2D, _ p2: Vector2D,
        _ p3: Vector2D, _ p4: Vector2D
    ) -> (Double, Double)? {
        let d1 = p2 - p1
        let d2 = p4 - p3
        let cross = d1.cross(d2)

        guard abs(cross) > crossEpsilon else { return nil }

        let d3 = p3 - p1
        let t = d3.cross(d2) / cross
        let u = d3.cross(d1) / cross

        guard t >= -paramEpsilon && t <= 1.0 + paramEpsilon &&
              u >= -paramEpsilon && u <= 1.0 + paramEpsilon else {
            return nil
        }

        return (min(max(t, 0.0), 1.0), min(max(u, 0.0), 1.0))
    }

    // MARK: - Private Helpers: Point-in-Polygon

    /// Ray-casting point-in-polygon test.
    private static func pointInPolygon(_ point: Vector2D, _ polygon: [Vector2D]) -> Bool {
        let n = polygon.count
        guard n >= 3 else { return false }

        var inside = false
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

    // MARK: - Private Helpers: Polygon Splitting

    /// Split a polygon (or polyline) at the given parameters into sub-segments.
    private static func splitPolygonAtParameters(
        _ poly: [Vector2D],
        splitPoints: [(Int, Double)],
        isClosed: Bool
    ) -> [[Vector2D]] {
        guard !splitPoints.isEmpty else { return [poly] }

        let segCount = isClosed ? poly.count : poly.count - 1

        // Sort by edge index then parameter
        var sorted = splitPoints.sorted { a, b in
            if a.0 != b.0 { return a.0 < b.0 }
            return a.1 < b.1
        }

        // Remove duplicates
        var unique: [(Int, Double)] = []
        for sp in sorted {
            if let last = unique.last {
                if last.0 == sp.0 && abs(last.1 - sp.1) < paramEpsilon * 10 {
                    continue
                }
            }
            unique.append(sp)
        }
        sorted = unique

        // Compute actual split points on the polygon
        var splitVertices: [(Int, Double, Vector2D)] = []
        for (edge, param) in sorted {
            guard edge < segCount else { continue }
            let p1 = poly[edge]
            let p2 = poly[(edge + 1) % poly.count]
            let point = p1.lerp(to: p2, t: param)
            splitVertices.append((edge, param, point))
        }

        guard !splitVertices.isEmpty else { return [poly] }

        // Build ordered list of all points with split markers
        var allPoints: [(Vector2D, Bool)] = [] // (point, isSplitPoint)

        var splitIdx = 0
        for i in 0..<segCount {
            allPoints.append((poly[i], false))

            // Insert any split points on this edge
            while splitIdx < splitVertices.count && splitVertices[splitIdx].0 == i {
                let sp = splitVertices[splitIdx]
                // Only insert if not coincident with edge endpoints
                let p1 = poly[i]
                let p2 = poly[(i + 1) % poly.count]
                if sp.2.distanceSquared(to: p1) > epsilonSq &&
                   sp.2.distanceSquared(to: p2) > epsilonSq {
                    allPoints.append((sp.2, true))
                } else if sp.2.distanceSquared(to: p2) < epsilonSq {
                    // Mark the next vertex as a split point instead
                    // We will handle this by marking it when we reach it
                } else {
                    // Coincident with p1, mark the current point
                    if let lastIdx = allPoints.indices.last {
                        allPoints[lastIdx] = (allPoints[lastIdx].0, true)
                    }
                }
                splitIdx += 1
            }
        }

        // For open paths, add the last point
        if !isClosed {
            allPoints.append((poly[poly.count - 1], false))
        }

        // Split into segments at split points
        var segments: [[Vector2D]] = []
        var current: [Vector2D] = []

        for (point, isSplit) in allPoints {
            current.append(point)
            if isSplit && current.count >= 2 {
                segments.append(current)
                current = [point] // Start new segment from split point
            }
        }

        // For closed paths, connect the last segment to the first
        if isClosed && !segments.isEmpty {
            if current.count >= 1 {
                // Append remaining points to form the last segment
                // that wraps around to the first split point
                if let firstSeg = segments.first, let firstPt = firstSeg.first {
                    current.append(firstPt)
                    if current.count >= 2 {
                        // Merge with the first segment
                        segments[0] = current + Array(segments[0].dropFirst())
                    }
                }
            }
        } else if current.count >= 2 {
            segments.append(current)
        }

        return segments.filter { $0.count >= 2 }
    }

    // MARK: - Private Helpers: Augmented Edge Walking

    /// An augmented edge point that includes intersection markers.
    private struct AugmentedPoint: Sendable {
        let point: Vector2D
        let isIntersection: Bool
        let intersectionIndex: Int // Index into the intersections array, or -1
    }

    /// Build an augmented list of points for a polygon with intersection points inserted.
    private static func buildAugmentedEdges(
        _ poly: [Vector2D],
        intersections: [IntersectionInfo],
        isA: Bool
    ) -> [AugmentedPoint] {
        let n = poly.count
        var result: [AugmentedPoint] = []

        // Group intersections by edge
        var byEdge: [Int: [(Int, Double)]] = [:] // edge -> [(interIndex, param)]
        for (idx, inter) in intersections.enumerated() {
            let edge = isA ? inter.edgeA : inter.edgeB
            let param = isA ? inter.paramA : inter.paramB
            byEdge[edge, default: []].append((idx, param))
        }

        // Sort intersections on each edge by parameter
        for key in byEdge.keys {
            byEdge[key]?.sort { $0.1 < $1.1 }
        }

        for i in 0..<n {
            result.append(AugmentedPoint(point: poly[i], isIntersection: false, intersectionIndex: -1))

            if let edgeInters = byEdge[i] {
                for (interIdx, _) in edgeInters {
                    let inter = intersections[interIdx]
                    result.append(AugmentedPoint(
                        point: inter.point,
                        isIntersection: true,
                        intersectionIndex: interIdx
                    ))
                }
            }
        }

        return result
    }

    /// Walk along augmented edges from a given position until the next intersection is reached.
    /// Returns the points traversed and the index of the next intersection.
    private static func walkUntilIntersection(
        from startPoint: Vector2D,
        edgeIndex: Int,
        param: Double,
        polygon: [Vector2D],
        augmented: [AugmentedPoint],
        intersections: [IntersectionInfo],
        isA: Bool
    ) -> ([Vector2D], Int?) {
        var points: [Vector2D] = []

        // Find the starting position in the augmented list
        var startAugIdx = -1
        for (i, ap) in augmented.enumerated() {
            if ap.isIntersection && ap.point.distanceSquared(to: startPoint) < epsilonSq {
                startAugIdx = i
                break
            }
        }

        guard startAugIdx >= 0 else { return ([], nil) }

        // Walk forward from the start position
        let augCount = augmented.count
        var current = (startAugIdx + 1) % augCount
        var steps = 0

        while steps < augCount {
            let ap = augmented[current]
            points.append(ap.point)

            if ap.isIntersection && ap.intersectionIndex >= 0 {
                return (points, ap.intersectionIndex)
            }

            current = (current + 1) % augCount
            steps += 1
        }

        return (points, nil)
    }

    /// Get a point slightly after a given parameter on a polygon edge.
    private static func pointSlightlyAfter(
        _ point: Vector2D,
        _ edgeIndex: Int,
        _ param: Double,
        _ polygon: [Vector2D],
        closed: Bool
    ) -> Vector2D {
        let n = polygon.count
        let p1 = polygon[edgeIndex]
        let p2 = polygon[(edgeIndex + 1) % n]

        let nudge = 0.001
        if param + nudge <= 1.0 {
            return p1.lerp(to: p2, t: param + nudge)
        } else {
            // Move into the next edge
            let nextEdge = (edgeIndex + 1) % n
            let nextP2 = polygon[(nextEdge + 1) % n]
            return polygon[nextEdge].lerp(to: nextP2, t: nudge)
        }
    }

    // MARK: - Private Helpers: Path Construction

    /// Convert a polygon (array of points) to a `VectorPath` with `.lineTo` segments.
    private static func polygonToPath(_ points: [Vector2D], closed: Bool) -> VectorPath? {
        guard points.count >= 2 else { return nil }

        var segments: [PathSegment] = []
        for i in 1..<points.count {
            // Skip zero-length segments
            if points[i].distanceSquared(to: points[i - 1]) > epsilonSq {
                segments.append(.lineTo(points[i]))
            }
        }

        guard !segments.isEmpty else { return nil }

        return VectorPath(
            startPoint: points[0],
            segments: segments,
            isClosed: closed
        )
    }

    /// Append path B to the end of path A, creating a single continuous open path.
    private static func appendPaths(_ a: VectorPath, _ b: VectorPath) -> VectorPath {
        var segments = a.segments
        // Add a line from A's end to B's start if needed
        let endA = a.segments.last?.endPoint ?? a.startPoint
        if endA.distanceSquared(to: b.startPoint) > epsilonSq {
            segments.append(.lineTo(b.startPoint))
        }
        segments.append(contentsOf: b.segments)

        return VectorPath(
            startPoint: a.startPoint,
            segments: segments,
            isClosed: false
        )
    }

    /// Compute the midpoint of a segment (array of points) for inside/outside classification.
    private static func segmentMidpoint(_ segment: [Vector2D]) -> Vector2D {
        guard segment.count >= 2 else { return segment.first ?? .zero }

        // Use the midpoint of the middle edge for better classification
        let midIdx = segment.count / 2
        let idx = max(0, min(midIdx, segment.count - 2))
        return segment[idx].lerp(to: segment[idx + 1], t: 0.5)
    }

    /// Append a point to an array only if it is distinct from the last point.
    private static func appendDistinct(_ array: inout [Vector2D], _ point: Vector2D) {
        if let last = array.last {
            guard last.distanceSquared(to: point) > epsilonSq else { return }
        }
        array.append(point)
    }

    // MARK: - Private Helpers: Closest Point on Segment

    /// Find the closest point on a line segment to a given point.
    /// Returns `(parameter, closestPoint)`.
    private static func closestPointOnSegment(
        _ point: Vector2D,
        _ segStart: Vector2D,
        _ segEnd: Vector2D
    ) -> (Double, Vector2D) {
        let d = segEnd - segStart
        let lenSq = d.lengthSquared
        guard lenSq > epsilonSq else {
            return (0.0, segStart)
        }

        let t = max(0.0, min(1.0, (point - segStart).dot(d) / lenSq))
        let closest = segStart.lerp(to: segEnd, t: t)
        return (t, closest)
    }

    // MARK: - Private Helpers: Ray Intersection for Extend

    /// Find the first intersection of a ray with any of the given polygons or boundary.
    private static func findFirstIntersectionAlongRay(
        from origin: Vector2D,
        direction: Vector2D,
        maxDistance: Double,
        polygons: [[Vector2D]],
        excludeIndex: Int,
        boundary: [Vector2D]?
    ) -> Vector2D? {
        let rayEnd = origin + direction * maxDistance
        var bestDist = Double.infinity
        var bestPoint: Vector2D?

        // Test against all polygon edges
        for (idx, poly) in polygons.enumerated() {
            if idx == excludeIndex { continue }
            let segCount = poly.count >= 3 ? poly.count : poly.count - 1
            guard segCount >= 1 else { continue }

            for i in 0..<segCount {
                let j = (i + 1) % poly.count
                if let (t, _) = segmentIntersection(origin, rayEnd, poly[i], poly[j]) {
                    let point = origin.lerp(to: rayEnd, t: t)
                    let dist = origin.distanceSquared(to: point)
                    if dist > epsilonSq && dist < bestDist {
                        bestDist = dist
                        bestPoint = point
                    }
                }
            }
        }

        // Test against boundary
        if let boundary = boundary, boundary.count >= 3 {
            for i in 0..<boundary.count {
                let j = (i + 1) % boundary.count
                if let (t, _) = segmentIntersection(origin, rayEnd, boundary[i], boundary[j]) {
                    let point = origin.lerp(to: rayEnd, t: t)
                    let dist = origin.distanceSquared(to: point)
                    if dist > epsilonSq && dist < bestDist {
                        bestDist = dist
                        bestPoint = point
                    }
                }
            }
        }

        return bestPoint
    }

    // MARK: - Private Helpers: Duplicate Detection

    /// Check whether two polygons represent the same geometry within tolerance.
    private static func arePolygonsDuplicate(
        _ a: [Vector2D], _ b: [Vector2D],
        toleranceSq: Double,
        closedA: Bool, closedB: Bool
    ) -> Bool {
        guard a.count == b.count else { return false }
        guard a.count >= 2 else { return true }

        // For closed paths, try all rotational offsets
        if closedA && closedB {
            let n = a.count
            for offset in 0..<n {
                var matches = true
                for i in 0..<n {
                    if a[i].distanceSquared(to: b[(i + offset) % n]) > toleranceSq {
                        matches = false
                        break
                    }
                }
                if matches { return true }

                // Also try reversed direction
                matches = true
                for i in 0..<n {
                    if a[i].distanceSquared(to: b[(offset - i + n) % n]) > toleranceSq {
                        matches = false
                        break
                    }
                }
                if matches { return true }
            }
            return false
        }

        // For open paths, compare in order and reverse
        var forwardMatch = true
        for i in 0..<a.count {
            if a[i].distanceSquared(to: b[i]) > toleranceSq {
                forwardMatch = false
                break
            }
        }
        if forwardMatch { return true }

        var reverseMatch = true
        let lastIdx = a.count - 1
        for i in 0..<a.count {
            if a[i].distanceSquared(to: b[lastIdx - i]) > toleranceSq {
                reverseMatch = false
                break
            }
        }
        return reverseMatch
    }

    // MARK: - Private Helpers: Path Splitting

    /// Split a closed polygon at the given sorted split points into open sub-paths.
    private static func splitClosedPath(
        poly: [Vector2D],
        splits: [(Int, Double, Vector2D)]
    ) -> [VectorPath] {
        let n = poly.count
        guard !splits.isEmpty else {
            return [polygonToPath(poly, closed: true)].compactMap { $0 }
        }

        // Build an ordered sequence of all vertices and split points around the polygon
        var orderedPoints: [(Vector2D, Bool)] = [] // (point, isSplit)
        var splitIdx = 0

        for i in 0..<n {
            orderedPoints.append((poly[i], false))

            while splitIdx < splits.count && splits[splitIdx].0 == i {
                let sp = splits[splitIdx]
                // Insert if not coincident with edge start or end
                if sp.2.distanceSquared(to: poly[i]) > epsilonSq &&
                   sp.2.distanceSquared(to: poly[(i + 1) % n]) > epsilonSq {
                    orderedPoints.append((sp.2, true))
                }
                splitIdx += 1
            }
        }

        // Find split point positions in the ordered list
        var splitIndices: [Int] = []
        for (i, op) in orderedPoints.enumerated() {
            if op.1 { splitIndices.append(i) }
        }

        guard splitIndices.count >= 2 else {
            return [polygonToPath(poly, closed: true)].compactMap { $0 }
        }

        // Extract sub-paths between consecutive split points
        var paths: [VectorPath] = []
        let totalPoints = orderedPoints.count

        for s in 0..<splitIndices.count {
            let startIdx = splitIndices[s]
            let endIdx = splitIndices[(s + 1) % splitIndices.count]

            var segment: [Vector2D] = []
            var current = startIdx

            repeat {
                segment.append(orderedPoints[current].0)
                current = (current + 1) % totalPoints
            } while current != (endIdx + 1) % totalPoints

            // Include the end split point
            if segment.last?.distanceSquared(to: orderedPoints[endIdx].0) ?? Double.infinity > epsilonSq {
                segment.append(orderedPoints[endIdx].0)
            }

            if segment.count >= 2 {
                if let p = polygonToPath(segment, closed: false) {
                    paths.append(p)
                }
            }
        }

        return paths
    }

    /// Split an open polyline at the given sorted split points.
    private static func splitOpenPath(
        poly: [Vector2D],
        splits: [(Int, Double, Vector2D)]
    ) -> [VectorPath] {
        guard !splits.isEmpty else {
            return [polygonToPath(poly, closed: false)].compactMap { $0 }
        }

        var result: [VectorPath] = []
        var currentSegment: [Vector2D] = [poly[0]]
        var splitIdx = 0

        for i in 0..<(poly.count - 1) {
            // Add any split points on this edge
            while splitIdx < splits.count && splits[splitIdx].0 == i {
                let sp = splits[splitIdx]
                currentSegment.append(sp.2)

                if currentSegment.count >= 2 {
                    if let p = polygonToPath(currentSegment, closed: false) {
                        result.append(p)
                    }
                }
                currentSegment = [sp.2]
                splitIdx += 1
            }

            currentSegment.append(poly[i + 1])
        }

        if currentSegment.count >= 2 {
            if let p = polygonToPath(currentSegment, closed: false) {
                result.append(p)
            }
        }

        return result
    }
}
