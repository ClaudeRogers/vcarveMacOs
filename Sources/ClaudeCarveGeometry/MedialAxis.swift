import Foundation
import ClaudeCarveCore

/// A point on the medial axis with its associated inscribed circle radius.
public struct MedialAxisPoint: Sendable {
    public let position: Vector2D
    public let radius: Double  // Distance to nearest boundary (inscribed circle radius)

    public init(position: Vector2D, radius: Double) {
        self.position = position
        self.radius = radius
    }
}

/// A segment of the medial axis connecting two medial axis points.
public struct MedialAxisSegment: Sendable {
    public let start: MedialAxisPoint
    public let end: MedialAxisPoint

    public init(start: MedialAxisPoint, end: MedialAxisPoint) {
        self.start = start
        self.end = end
    }

    /// Interpolate along the segment.
    public func interpolate(t: Double) -> MedialAxisPoint {
        let pos = start.position.lerp(to: end.position, t: t)
        let r = start.radius + (end.radius - start.radius) * t
        return MedialAxisPoint(position: pos, radius: r)
    }
}

/// The medial axis (skeleton) of a 2D shape.
/// This is the core data structure for V-carving toolpath generation.
/// The medial axis is the locus of centers of maximal inscribed circles.
public struct MedialAxis: Sendable {
    public var segments: [MedialAxisSegment]

    public init(segments: [MedialAxisSegment] = []) {
        self.segments = segments
    }

    /// Compute the medial axis of a closed polygon using the Voronoi-based approach.
    ///
    /// Algorithm:
    /// 1. Sample points densely along the boundary
    /// 2. Compute approximate Voronoi diagram of boundary samples
    /// 3. Filter Voronoi vertices to retain only those inside the polygon
    /// 4. Connect vertices to form the medial axis graph
    /// 5. Record the inscribed circle radius at each vertex
    public static func compute(for polygon: [Vector2D], resolution: Double = 0.5) -> MedialAxis {
        guard polygon.count >= 3 else { return MedialAxis() }

        // Step 1: Sample boundary points densely
        let boundaryPoints = sampleBoundary(polygon, spacing: resolution)
        guard boundaryPoints.count >= 6 else { return MedialAxis() }

        // Step 2: Build boundary segments for distance queries
        let boundarySegments = buildBoundarySegments(polygon)

        // Step 3: Compute approximate medial axis using distance field sampling
        let medialPoints = computeMedialPoints(
            polygon: polygon,
            boundarySegments: boundarySegments,
            resolution: resolution
        )

        // Step 4: Connect medial axis points into segments
        let segments = connectMedialPoints(medialPoints, maxGap: resolution * 3)

        return MedialAxis(segments: segments)
    }

    /// Sample points evenly along the boundary of a polygon.
    private static func sampleBoundary(_ polygon: [Vector2D], spacing: Double) -> [Vector2D] {
        var samples: [Vector2D] = []
        let n = polygon.count

        for i in 0..<n {
            let start = polygon[i]
            let end = polygon[(i + 1) % n]
            let edgeLength = start.distance(to: end)
            let steps = max(1, Int(edgeLength / spacing))

            for j in 0..<steps {
                let t = Double(j) / Double(steps)
                samples.append(start.lerp(to: end, t: t))
            }
        }

        return samples
    }

    /// Build boundary segments as (start, end) pairs.
    private static func buildBoundarySegments(_ polygon: [Vector2D]) -> [(Vector2D, Vector2D)] {
        var segments: [(Vector2D, Vector2D)] = []
        for i in 0..<polygon.count {
            segments.append((polygon[i], polygon[(i + 1) % polygon.count]))
        }
        return segments
    }

    /// Find the minimum distance from a point to any boundary segment.
    private static func distanceToBoundary(_ point: Vector2D, segments: [(Vector2D, Vector2D)]) -> Double {
        var minDist = Double.infinity

        for (a, b) in segments {
            let d = pointToSegmentDistance(point, a, b)
            minDist = min(minDist, d)
        }

        return minDist
    }

    /// Distance from a point to a line segment.
    private static func pointToSegmentDistance(_ p: Vector2D, _ a: Vector2D, _ b: Vector2D) -> Double {
        let ab = b - a
        let ap = p - a
        let lenSq = ab.lengthSquared

        if lenSq < 1e-12 {
            return ap.length
        }

        let t = max(0, min(1, ap.dot(ab) / lenSq))
        let projection = a + ab * t
        return p.distance(to: projection)
    }

    /// Check if a point is inside a polygon using ray casting.
    private static func isInsidePolygon(_ point: Vector2D, _ polygon: [Vector2D]) -> Bool {
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

    /// Compute medial axis points using distance field ridge detection.
    /// A point is on the medial axis if it has two or more equidistant nearest boundary points.
    private static func computeMedialPoints(
        polygon: [Vector2D],
        boundarySegments: [(Vector2D, Vector2D)],
        resolution: Double
    ) -> [MedialAxisPoint] {
        let bb = BoundingBox2D.enclosing(polygon)
        let margin = resolution
        var medialPoints: [MedialAxisPoint] = []

        // Sample the distance field on a grid
        let nx = max(10, Int((bb.width + 2 * margin) / resolution))
        let ny = max(10, Int((bb.height + 2 * margin) / resolution))

        // Build distance field
        var distanceField = Array(repeating: Array(repeating: 0.0, count: nx), count: ny)

        for iy in 0..<ny {
            let y = bb.min.y - margin + (Double(iy) + 0.5) * (bb.height + 2 * margin) / Double(ny)
            for ix in 0..<nx {
                let x = bb.min.x - margin + (Double(ix) + 0.5) * (bb.width + 2 * margin) / Double(nx)
                let p = Vector2D(x, y)

                if isInsidePolygon(p, polygon) {
                    distanceField[iy][ix] = distanceToBoundary(p, segments: boundarySegments)
                } else {
                    distanceField[iy][ix] = -1  // Outside
                }
            }
        }

        // Find ridge points (local maxima of distance field)
        for iy in 1..<(ny - 1) {
            for ix in 1..<(nx - 1) {
                let d = distanceField[iy][ix]
                if d <= 0 { continue }

                // Check if this is a ridge point (local maximum in at least one direction)
                let dx = distanceField[iy][ix + 1] - distanceField[iy][ix - 1]
                let dy = distanceField[iy + 1][ix] - distanceField[iy - 1][ix]
                let dxx = distanceField[iy][ix + 1] + distanceField[iy][ix - 1] - 2 * d
                let dyy = distanceField[iy + 1][ix] + distanceField[iy - 1][ix] - 2 * d

                // Ridge detection: negative curvature in at least one direction
                let isRidge = (dxx < -1e-6 || dyy < -1e-6) && d > resolution * 0.3

                // Also detect saddle points and branching
                let gradMag = sqrt(dx * dx + dy * dy)
                let isSaddle = gradMag < 0.1 * resolution && d > resolution * 0.3

                if isRidge || isSaddle {
                    let x = bb.min.x - margin + (Double(ix) + 0.5) * (bb.width + 2 * margin) / Double(nx)
                    let y = bb.min.y - margin + (Double(iy) + 0.5) * (bb.height + 2 * margin) / Double(ny)
                    medialPoints.append(MedialAxisPoint(position: Vector2D(x, y), radius: d))
                }
            }
        }

        return medialPoints
    }

    /// Connect medial points into segments using nearest-neighbor chains.
    private static func connectMedialPoints(_ points: [MedialAxisPoint], maxGap: Double) -> [MedialAxisSegment] {
        guard points.count >= 2 else { return [] }

        var segments: [MedialAxisSegment] = []
        var used = Set<Int>()

        for i in 0..<points.count {
            if used.contains(i) { continue }

            // Find nearest unvisited neighbor
            var bestJ = -1
            var bestDist = Double.infinity

            for j in 0..<points.count {
                if i == j || used.contains(j) { continue }
                let d = points[i].position.distance(to: points[j].position)
                if d < bestDist && d < maxGap {
                    bestDist = d
                    bestJ = j
                }
            }

            if bestJ >= 0 {
                segments.append(MedialAxisSegment(start: points[i], end: points[bestJ]))
                used.insert(bestJ)
            }
        }

        return segments
    }
}
