import Foundation
import ClaudeCarveCore

// MARK: - Weiler-Atherton Polygon Boolean Operations

/// Boolean operations on polygons using the Weiler-Atherton algorithm.
/// Designed for CNC toolpath computation where geometric precision is critical.
///
/// Supports union, intersection, and difference operations on simple (non-self-intersecting)
/// polygons. Handles edge cases including containment, tangency, shared edges, and
/// degenerate configurations.
///
/// All polygons are assumed to use counter-clockwise winding for outer boundaries.
public struct PolygonBoolean: Sendable {

    // MARK: - Tolerances

    /// Absolute spatial tolerance for coincidence checks (in user units, typically mm/inches).
    /// For CNC applications, 1e-9 is well below any machine resolution.
    private static let epsilon: Double = 1e-9

    /// Squared epsilon to avoid sqrt in distance comparisons.
    private static let epsilonSquared: Double = 1e-18

    /// Tolerance for parametric values along edges [0, 1].
    private static let paramEpsilon: Double = 1e-10

    /// Tolerance for cross-product collinearity checks.
    private static let crossEpsilon: Double = 1e-10

    // MARK: - Public API

    /// Compute the union of two polygons.
    ///
    /// Returns one or more polygons representing the area covered by either `a` or `b`.
    /// If the polygons are disjoint, both are returned. If one contains the other,
    /// the outer polygon is returned.
    ///
    /// - Parameters:
    ///   - a: First polygon vertices in counter-clockwise order.
    ///   - b: Second polygon vertices in counter-clockwise order.
    /// - Returns: Array of polygons representing `a ∪ b`.
    public static func union(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        guard a.count >= 3, b.count >= 3 else {
            if a.count >= 3 { return [a] }
            if b.count >= 3 { return [b] }
            return []
        }

        let intersections = findIntersections(a, b)

        if intersections.isEmpty {
            if isPointInsidePolygon(b[0], a) {
                return [a] // b entirely inside a
            } else if isPointInsidePolygon(a[0], b) {
                return [b] // a entirely inside b
            } else {
                return [a, b] // disjoint
            }
        }

        let result = weilerAtherton(a, b, operation: .union)
        return result.isEmpty ? [a, b] : result
    }

    /// Compute the intersection of two polygons.
    ///
    /// Returns one or more polygons representing the area covered by both `a` and `b`.
    /// If the polygons are disjoint, returns an empty array. If one contains the other,
    /// the inner polygon is returned.
    ///
    /// - Parameters:
    ///   - a: First polygon vertices in counter-clockwise order.
    ///   - b: Second polygon vertices in counter-clockwise order.
    /// - Returns: Array of polygons representing `a ∩ b`.
    public static func intersection(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        guard a.count >= 3, b.count >= 3 else { return [] }

        let intersections = findIntersections(a, b)

        if intersections.isEmpty {
            if isPointInsidePolygon(b[0], a) {
                return [b] // b entirely inside a
            } else if isPointInsidePolygon(a[0], b) {
                return [a] // a entirely inside b
            } else {
                return [] // disjoint
            }
        }

        let result = weilerAtherton(a, b, operation: .intersection)
        return result
    }

    /// Compute the difference `a - b` (subtract `b` from `a`).
    ///
    /// Returns one or more polygons representing the area of `a` not covered by `b`.
    /// If `a` is entirely inside `b`, returns empty. If they are disjoint, returns `a`.
    ///
    /// - Parameters:
    ///   - a: Subject polygon vertices in counter-clockwise order.
    ///   - b: Clip polygon vertices in counter-clockwise order.
    /// - Returns: Array of polygons representing `a \ b`.
    public static func difference(_ a: [Vector2D], _ b: [Vector2D]) -> [[Vector2D]] {
        guard a.count >= 3, b.count >= 3 else {
            if a.count >= 3 { return [a] }
            return []
        }

        let intersections = findIntersections(a, b)

        if intersections.isEmpty {
            if isPointInsidePolygon(a[0], b) {
                return [] // a entirely inside b
            } else if isPointInsidePolygon(b[0], a) {
                // b entirely inside a: result is a with a hole (return both contours)
                return [a, b.reversed()]
            } else {
                return [a] // disjoint
            }
        }

        let result = weilerAtherton(a, b, operation: .difference)
        return result.isEmpty ? [a] : result
    }

    /// Test whether a point lies inside a polygon using the ray casting algorithm.
    ///
    /// Uses a horizontal ray cast to the right. Points exactly on an edge are
    /// considered inside (within tolerance).
    ///
    /// - Parameters:
    ///   - point: The point to test.
    ///   - polygon: The polygon vertices.
    /// - Returns: `true` if the point is inside or on the boundary of the polygon.
    public static func isPointInsidePolygon(_ point: Vector2D, _ polygon: [Vector2D]) -> Bool {
        let n = polygon.count
        guard n >= 3 else { return false }

        // First check if point is on any edge
        for i in 0..<n {
            let j = (i + 1) % n
            if isPointOnSegment(point, polygon[i], polygon[j]) {
                return true
            }
        }

        // Ray casting
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

    /// Compute the convex hull of a set of points using the Graham scan algorithm.
    ///
    /// - Parameter points: An array of 2D points.
    /// - Returns: Vertices of the convex hull in counter-clockwise order.
    ///   Returns the input if fewer than 3 points are provided
    ///   (or 2 points / 1 point / empty as appropriate).
    public static func convexHull(_ points: [Vector2D]) -> [Vector2D] {
        guard points.count >= 3 else { return points }

        // Find the point with the lowest y (and leftmost if tied)
        var pivot = points[0]
        for p in points {
            if p.y < pivot.y || (p.y == pivot.y && p.x < pivot.x) {
                pivot = p
            }
        }

        // Sort by polar angle relative to pivot
        var sorted = points.filter { $0.distanceSquared(to: pivot) > epsilonSquared }
        sorted.sort { a, b in
            let da = a - pivot
            let db = b - pivot
            let cross = da.cross(db)
            if abs(cross) < crossEpsilon {
                // Collinear: closer point comes first
                return da.lengthSquared < db.lengthSquared
            }
            return cross > 0
        }

        // Build hull
        var hull: [Vector2D] = [pivot]

        for point in sorted {
            // Remove points that make a clockwise turn
            while hull.count >= 2 {
                let top = hull[hull.count - 1]
                let nextToTop = hull[hull.count - 2]
                let cross = (top - nextToTop).cross(point - nextToTop)
                if cross <= crossEpsilon {
                    hull.removeLast()
                } else {
                    break
                }
            }
            hull.append(point)
        }

        // Remove degenerate hull
        if hull.count < 3 {
            return hull
        }

        return hull
    }

    // MARK: - Internal Types

    /// Classification of an intersection point relative to the traversal direction.
    private enum IntersectionKind: Sendable {
        case entering  // Entering the other polygon
        case leaving   // Leaving the other polygon
    }

    /// Boolean operation type.
    private enum BooleanOp: Sendable {
        case union
        case intersection
        case difference
    }

    /// A vertex node in the Weiler-Atherton linked polygon structure.
    /// Each node is either an original polygon vertex or an inserted intersection point.
    private final class VertexNode: @unchecked Sendable {
        let point: Vector2D
        var next: VertexNode?
        var prev: VertexNode?

        /// If this is an intersection vertex, a reference to the corresponding node
        /// in the other polygon's linked list.
        var neighbor: VertexNode?

        /// Classification for intersection vertices.
        var kind: IntersectionKind?

        /// Whether this node is an intersection point.
        var isIntersection: Bool

        /// Whether this node has been visited during traversal.
        var visited: Bool = false

        /// Parametric position along the edge for sorting intersections on the same edge.
        var param: Double = 0.0

        /// Index of the edge this intersection lies on (in the original polygon).
        var edgeIndex: Int = -1

        init(point: Vector2D, isIntersection: Bool = false) {
            self.point = point
            self.isIntersection = isIntersection
        }
    }

    /// Raw intersection data from the geometric intersection finder.
    private struct IntersectionData: Sendable {
        let point: Vector2D
        let edgeA: Int
        let paramA: Double
        let edgeB: Int
        let paramB: Double
    }

    // MARK: - Intersection Finding

    /// Find all intersection points between edges of polygon `a` and polygon `b`.
    ///
    /// Intersections at polygon vertices are handled carefully: if an intersection
    /// falls within `paramEpsilon` of an endpoint, it is snapped to that endpoint
    /// to prevent near-degenerate slivers.
    private static func findIntersections(_ a: [Vector2D], _ b: [Vector2D]) -> [IntersectionData] {
        var results: [IntersectionData] = []

        for i in 0..<a.count {
            let a1 = a[i]
            let a2 = a[(i + 1) % a.count]

            for j in 0..<b.count {
                let b1 = b[j]
                let b2 = b[(j + 1) % b.count]

                if let (t, u) = segmentIntersection(a1, a2, b1, b2) {
                    // Snap parameters near 0 or 1 to avoid degenerate slivers
                    let tSnapped = snapParam(t)
                    let uSnapped = snapParam(u)

                    // Skip intersections that map to the same vertex via snapping
                    // to avoid duplicates from shared vertices
                    let point = a1.lerp(to: a2, t: tSnapped)

                    // Check for duplicate intersection points
                    let isDuplicate = results.contains { existing in
                        existing.point.distanceSquared(to: point) < epsilonSquared
                    }

                    if !isDuplicate {
                        results.append(IntersectionData(
                            point: point,
                            edgeA: i,
                            paramA: tSnapped,
                            edgeB: j,
                            paramB: uSnapped
                        ))
                    }
                }
            }
        }

        return results
    }

    /// Compute the intersection of two line segments.
    ///
    /// Returns `(t, u)` where `t` is the parameter along segment `(p1, p2)`
    /// and `u` is the parameter along segment `(p3, p4)`, both in `[0, 1]`.
    ///
    /// Returns `nil` if the segments are parallel or do not intersect.
    private static func segmentIntersection(
        _ p1: Vector2D, _ p2: Vector2D,
        _ p3: Vector2D, _ p4: Vector2D
    ) -> (Double, Double)? {
        let d1 = p2 - p1
        let d2 = p4 - p3
        let cross = d1.cross(d2)

        // Parallel or collinear segments
        guard abs(cross) > crossEpsilon else { return nil }

        let d3 = p3 - p1
        let t = d3.cross(d2) / cross
        let u = d3.cross(d1) / cross

        // Allow a small tolerance outside [0, 1] for numerical robustness,
        // but reject clearly non-intersecting segments
        guard t >= -paramEpsilon && t <= 1.0 + paramEpsilon &&
              u >= -paramEpsilon && u <= 1.0 + paramEpsilon else {
            return nil
        }

        return (clampParam(t), clampParam(u))
    }

    /// Snap a parametric value to 0 or 1 if it is within tolerance.
    private static func snapParam(_ t: Double) -> Double {
        if t < paramEpsilon { return 0.0 }
        if t > 1.0 - paramEpsilon { return 1.0 }
        return t
    }

    /// Clamp a parametric value to [0, 1].
    private static func clampParam(_ t: Double) -> Double {
        min(max(t, 0.0), 1.0)
    }

    // MARK: - Linked List Construction

    /// Build a circular doubly-linked list from a polygon's vertices.
    private static func buildLinkedList(from polygon: [Vector2D]) -> VertexNode? {
        guard !polygon.isEmpty else { return nil }

        let head = VertexNode(point: polygon[0])
        head.edgeIndex = 0
        var current = head

        for i in 1..<polygon.count {
            let node = VertexNode(point: polygon[i])
            node.edgeIndex = i
            current.next = node
            node.prev = current
            current = node
        }

        // Close the loop
        current.next = head
        head.prev = current

        return head
    }

    /// Insert intersection nodes into a polygon's linked list at their proper positions.
    ///
    /// Intersections on the same edge are sorted by their parametric value to ensure
    /// correct ordering.
    ///
    /// - Parameters:
    ///   - head: The head of the polygon's linked list.
    ///   - intersections: Sorted intersection data for this polygon.
    ///   - edgeKeyPath: KeyPath to extract the edge index from IntersectionData.
    ///   - paramKeyPath: KeyPath to extract the parameter from IntersectionData.
    /// - Returns: Array of the newly inserted intersection nodes.
    private static func insertIntersections(
        into head: VertexNode,
        intersections: [IntersectionData],
        edgeKeyPath: KeyPath<IntersectionData, Int>,
        paramKeyPath: KeyPath<IntersectionData, Double>
    ) -> [VertexNode] {
        // Group intersections by edge and sort by parameter within each edge
        var byEdge: [Int: [(IntersectionData, Int)]] = [:]
        for (idx, inter) in intersections.enumerated() {
            let edgeIdx = inter[keyPath: edgeKeyPath]
            byEdge[edgeIdx, default: []].append((inter, idx))
        }

        var nodes: [VertexNode?] = Array(repeating: nil, count: intersections.count)

        for (_, edgeInters) in byEdge {
            // Sort by parameter along the edge
            let sorted = edgeInters.sorted { $0.0[keyPath: paramKeyPath] < $1.0[keyPath: paramKeyPath] }

            for (inter, globalIdx) in sorted {
                let edgeIdx = inter[keyPath: edgeKeyPath]
                let param = inter[keyPath: paramKeyPath]

                // Find the edge start node
                let edgeStart = findNode(in: head, atEdgeIndex: edgeIdx)

                // Find the correct position: after the edge start and any previously
                // inserted intersections with smaller parameter
                var insertAfter = edgeStart
                while let nextNode = insertAfter?.next,
                      nextNode.isIntersection,
                      nextNode.edgeIndex == edgeIdx,
                      nextNode.param < param {
                    insertAfter = nextNode
                }

                // Create and insert the intersection node
                let node = VertexNode(point: inter.point, isIntersection: true)
                node.param = param
                node.edgeIndex = edgeIdx

                let nextNode = insertAfter?.next
                insertAfter?.next = node
                node.prev = insertAfter
                node.next = nextNode
                nextNode?.prev = node

                nodes[globalIdx] = node
            }
        }

        return nodes.compactMap { $0 }
    }

    /// Find the vertex node at a given original edge index by traversing the linked list.
    private static func findNode(in head: VertexNode, atEdgeIndex edgeIndex: Int) -> VertexNode? {
        var current: VertexNode? = head
        let start = head

        repeat {
            if !current!.isIntersection && current!.edgeIndex == edgeIndex {
                return current
            }
            current = current?.next
        } while current !== nil && current !== start

        return nil
    }

    // MARK: - Intersection Classification

    /// Classify each intersection as entering or leaving relative to the other polygon.
    ///
    /// An intersection is "entering" polygon B if, when traversing polygon A in its
    /// winding direction, we cross from outside B to inside B at that point.
    private static func classifyIntersections(
        listA: VertexNode,
        listB: VertexNode,
        intersectionsA: [VertexNode],
        intersectionsB: [VertexNode],
        polygonA: [Vector2D],
        polygonB: [Vector2D]
    ) {
        for nodeA in intersectionsA {
            // Determine if traversal of A at this intersection goes into or out of B.
            // Look at the midpoint of the tiny segment just after the intersection
            // along polygon A's traversal direction.
            let pointAfter = midpointAfter(nodeA)
            let entering = isPointInsidePolygon(pointAfter, polygonB)
            nodeA.kind = entering ? .entering : .leaving
        }

        for nodeB in intersectionsB {
            let pointAfter = midpointAfter(nodeB)
            let entering = isPointInsidePolygon(pointAfter, polygonA)
            nodeB.kind = entering ? .entering : .leaving
        }
    }

    /// Get a point slightly after the given node along its polygon's linked list traversal.
    /// Used for inside/outside classification near intersection points.
    private static func midpointAfter(_ node: VertexNode) -> Vector2D {
        guard let next = node.next else { return node.point }

        // Use a small fraction along the edge to the next vertex
        let t = 0.001
        return node.point.lerp(to: next.point, t: t)
    }

    // MARK: - Weiler-Atherton Traversal

    /// Execute the Weiler-Atherton algorithm for the specified boolean operation.
    private static func weilerAtherton(
        _ a: [Vector2D],
        _ b: [Vector2D],
        operation: BooleanOp
    ) -> [[Vector2D]] {
        let intersections = findIntersections(a, b)
        guard !intersections.isEmpty else { return [] }

        // Build linked lists
        guard let listA = buildLinkedList(from: a),
              let listB = buildLinkedList(from: b) else {
            return []
        }

        // Insert intersection nodes into both lists
        let interNodesA = insertIntersections(
            into: listA,
            intersections: intersections,
            edgeKeyPath: \.edgeA,
            paramKeyPath: \.paramA
        )

        let interNodesB = insertIntersections(
            into: listB,
            intersections: intersections,
            edgeKeyPath: \.edgeB,
            paramKeyPath: \.paramB
        )

        // Link corresponding intersection nodes between lists
        guard interNodesA.count == interNodesB.count else { return [] }

        for i in 0..<interNodesA.count {
            interNodesA[i].neighbor = interNodesB[i]
            interNodesB[i].neighbor = interNodesA[i]
        }

        // Classify intersections
        classifyIntersections(
            listA: listA,
            listB: listB,
            intersectionsA: interNodesA,
            intersectionsB: interNodesB,
            polygonA: a,
            polygonB: b
        )

        // Traverse based on operation type
        var result: [[Vector2D]] = []

        switch operation {
        case .intersection:
            // Start from entering intersections on A (entering B), follow A until leaving,
            // then switch to B, follow B until entering, switch back to A, repeat.
            result = traverseForIntersection(interNodesA: interNodesA)

        case .union:
            // Start from leaving intersections on A (leaving B), follow A until entering,
            // then switch to B (going backward from the neighbor), follow B until leaving,
            // switch back to A, repeat.
            result = traverseForUnion(interNodesA: interNodesA)

        case .difference:
            // A - B: Start from leaving intersections on A (leaving B), follow A until entering B,
            // then switch to B going in reverse direction, follow until leaving A, switch back.
            result = traverseForDifference(interNodesA: interNodesA, interNodesB: interNodesB)
        }

        // Filter out degenerate polygons
        result = result.filter { polygon in
            polygon.count >= 3 && abs(polygonArea(polygon)) > epsilon
        }

        return result
    }

    /// Traverse the linked structure to extract intersection polygons.
    ///
    /// For INTERSECTION: start at entering intersections on A, follow A forward until
    /// a leaving intersection, switch to B forward until an entering intersection, repeat.
    private static func traverseForIntersection(
        interNodesA: [VertexNode]
    ) -> [[Vector2D]] {
        var result: [[Vector2D]] = []

        // Reset visited flags
        for node in interNodesA {
            node.visited = false
        }

        for startNode in interNodesA {
            guard startNode.kind == .entering, !startNode.visited else { continue }

            var polygon: [Vector2D] = []
            var current: VertexNode? = startNode
            var onA = true
            var safetyCounter = 0
            let maxIterations = 10000

            repeat {
                guard let node = current else { break }

                safetyCounter += 1
                if safetyCounter > maxIterations { break }

                if node.isIntersection {
                    node.visited = true
                    node.neighbor?.visited = true
                }

                appendIfDistinct(&polygon, node.point)

                // Check if we should switch polygons
                if let next = node.next, next.isIntersection {
                    if onA && next.kind == .leaving {
                        // Switch from A to B
                        appendIfDistinct(&polygon, next.point)
                        next.visited = true
                        next.neighbor?.visited = true
                        current = next.neighbor?.next
                        onA = false
                        continue
                    } else if !onA && next.kind == .entering {
                        // Switch from B to A
                        appendIfDistinct(&polygon, next.point)
                        next.visited = true
                        next.neighbor?.visited = true
                        current = next.neighbor?.next
                        onA = true

                        // Check if we've returned to start
                        if current === startNode {
                            break
                        }
                        continue
                    }
                }

                current = node.next

                // Check if we've returned to start
                if current === startNode {
                    break
                }

            } while current !== startNode && current != nil

            if polygon.count >= 3 {
                result.append(polygon)
            }
        }

        return result
    }

    /// Traverse for union operation.
    ///
    /// For UNION: start at leaving intersections on A, follow A forward until
    /// entering B, switch to B forward until leaving B, switch back to A.
    private static func traverseForUnion(
        interNodesA: [VertexNode]
    ) -> [[Vector2D]] {
        var result: [[Vector2D]] = []

        for node in interNodesA {
            node.visited = false
        }

        for startNode in interNodesA {
            guard startNode.kind == .leaving, !startNode.visited else { continue }

            var polygon: [Vector2D] = []
            var current: VertexNode? = startNode
            var onA = true
            var safetyCounter = 0
            let maxIterations = 10000

            repeat {
                guard let node = current else { break }

                safetyCounter += 1
                if safetyCounter > maxIterations { break }

                if node.isIntersection {
                    node.visited = true
                    node.neighbor?.visited = true
                }

                appendIfDistinct(&polygon, node.point)

                if let next = node.next, next.isIntersection {
                    if onA && next.kind == .entering {
                        // Entering B from A side means we switch to B
                        appendIfDistinct(&polygon, next.point)
                        next.visited = true
                        next.neighbor?.visited = true
                        current = next.neighbor?.next
                        onA = false
                        continue
                    } else if !onA && next.kind == .leaving {
                        // Leaving B from B side means we switch back to A
                        appendIfDistinct(&polygon, next.point)
                        next.visited = true
                        next.neighbor?.visited = true
                        current = next.neighbor?.next
                        onA = true

                        if current === startNode {
                            break
                        }
                        continue
                    }
                }

                current = node.next

                if current === startNode {
                    break
                }

            } while current !== startNode && current != nil

            if polygon.count >= 3 {
                result.append(polygon)
            }
        }

        return result
    }

    /// Traverse for difference (A - B).
    ///
    /// Follow A at regions outside B. At intersections where we enter B, switch to B
    /// and follow it in reverse until we leave B, then switch back to A.
    private static func traverseForDifference(
        interNodesA: [VertexNode],
        interNodesB: [VertexNode]
    ) -> [[Vector2D]] {
        var result: [[Vector2D]] = []

        for node in interNodesA {
            node.visited = false
        }

        for startNode in interNodesA {
            // For A - B: start at leaving intersections on A (where we exit B and
            // re-enter the region that is in A but not in B)
            guard startNode.kind == .leaving, !startNode.visited else { continue }

            var polygon: [Vector2D] = []
            var current: VertexNode? = startNode
            var onA = true
            var safetyCounter = 0
            let maxIterations = 10000

            repeat {
                guard let node = current else { break }

                safetyCounter += 1
                if safetyCounter > maxIterations { break }

                if node.isIntersection {
                    node.visited = true
                    node.neighbor?.visited = true
                }

                appendIfDistinct(&polygon, node.point)

                if let next = node.next, next.isIntersection {
                    if onA && next.kind == .entering {
                        // We are entering B: switch to B and traverse in reverse
                        appendIfDistinct(&polygon, next.point)
                        next.visited = true
                        next.neighbor?.visited = true
                        current = next.neighbor?.prev
                        onA = false
                        continue
                    } else if !onA {
                        // On B (reversed), check if the prev node is an intersection
                        // that corresponds to a leaving intersection on A
                        // We need to check the current node's prev for intersection
                    }
                }

                if onA {
                    current = node.next
                } else {
                    // Traversing B in reverse
                    let prevNode = node.prev
                    if let pn = prevNode, pn.isIntersection {
                        // Check if this intersection on B corresponds to a leaving on A
                        appendIfDistinct(&polygon, pn.point)
                        pn.visited = true
                        pn.neighbor?.visited = true

                        // Switch back to A
                        current = pn.neighbor?.next
                        onA = true

                        if current === startNode {
                            break
                        }
                        continue
                    }
                    current = prevNode
                }

                if current === startNode {
                    break
                }

            } while current !== startNode && current != nil

            if polygon.count >= 3 {
                result.append(polygon)
            }
        }

        return result
    }

    // MARK: - Geometric Utilities

    /// Compute the signed area of a polygon.
    /// Positive for counter-clockwise, negative for clockwise.
    private static func polygonArea(_ polygon: [Vector2D]) -> Double {
        var area = 0.0
        let n = polygon.count
        for i in 0..<n {
            let j = (i + 1) % n
            area += polygon[i].x * polygon[j].y
            area -= polygon[j].x * polygon[i].y
        }
        return area * 0.5
    }

    /// Check whether a point lies on a line segment within tolerance.
    private static func isPointOnSegment(
        _ point: Vector2D,
        _ segStart: Vector2D,
        _ segEnd: Vector2D
    ) -> Bool {
        let d = segEnd - segStart
        let lengthSq = d.lengthSquared
        guard lengthSq > epsilonSquared else {
            // Degenerate segment (single point)
            return point.distanceSquared(to: segStart) < epsilonSquared
        }

        // Check that the cross product (distance from line) is near zero
        let cross = (point - segStart).cross(d)
        let distFromLine = (cross * cross) / lengthSq
        guard distFromLine < epsilon * epsilon * 100.0 else { return false }

        // Check that the point projection falls within the segment
        let t = (point - segStart).dot(d) / lengthSq
        return t >= -paramEpsilon && t <= 1.0 + paramEpsilon
    }

    /// Append a point to a polygon, skipping it if it is coincident with the last point.
    private static func appendIfDistinct(_ polygon: inout [Vector2D], _ point: Vector2D) {
        if let last = polygon.last {
            guard last.distanceSquared(to: point) > epsilonSquared else { return }
        }
        // Also check against the first point to avoid closing-duplicate
        if polygon.count > 1 {
            if polygon[0].distanceSquared(to: point) < epsilonSquared {
                return
            }
        }
        polygon.append(point)
    }
}
