import Foundation
import ClaudeCarveCore

// MARK: - Public Data Types

/// A vertex of the Voronoi diagram, located equidistant from three or more sites.
public struct VoronoiVertex: Sendable {
    /// Position of this vertex in 2D space.
    public let position: Vector2D

    /// Indices of the sites that are equidistant from this vertex.
    /// Typically three sites for a non-degenerate configuration.
    public let siteIndices: [Int]

    public init(position: Vector2D, siteIndices: [Int]) {
        self.position = position
        self.siteIndices = siteIndices
    }
}

/// An edge of the Voronoi diagram, separating two adjacent Voronoi cells.
public struct VoronoiEdge: Sendable {
    /// Start point of this edge (clipped to bounding box).
    public let start: Vector2D

    /// End point of this edge (clipped to bounding box).
    public let end: Vector2D

    /// Index of the site to the left of this edge.
    public let leftSiteIndex: Int

    /// Index of the site to the right of this edge.
    public let rightSiteIndex: Int

    public init(start: Vector2D, end: Vector2D, leftSiteIndex: Int, rightSiteIndex: Int) {
        self.start = start
        self.end = end
        self.leftSiteIndex = leftSiteIndex
        self.rightSiteIndex = rightSiteIndex
    }
}

/// A Voronoi cell: the region of space closest to a particular site.
public struct VoronoiCell: Sendable {
    /// The generating site for this cell.
    public let site: Vector2D

    /// Indices into the parent diagram's `edges` array for edges bounding this cell.
    public let edgeIndices: [Int]

    public init(site: Vector2D, edgeIndices: [Int]) {
        self.site = site
        self.edgeIndices = edgeIndices
    }
}

/// The Voronoi diagram of a set of 2D point sites, computed using Fortune's sweep line algorithm.
///
/// The Voronoi diagram partitions the plane into cells, one per site, such that every point
/// in a cell is closer to its site than to any other. For CNC V-carving, the Voronoi diagram
/// of boundary sample points yields the medial axis (skeleton) of the shape, which determines
/// the toolpath centerline and carving depth at each point.
public struct VoronoiDiagram: Sendable {
    /// Voronoi vertices (points equidistant from three or more sites).
    public let vertices: [VoronoiVertex]

    /// Voronoi edges (segments separating adjacent cells).
    public let edges: [VoronoiEdge]

    /// Voronoi cells (one per input site).
    public let cells: [VoronoiCell]

    public init(vertices: [VoronoiVertex], edges: [VoronoiEdge], cells: [VoronoiCell]) {
        self.vertices = vertices
        self.edges = edges
        self.cells = cells
    }

    /// Compute the Voronoi diagram of the given sites, clipped to the specified bounding box.
    ///
    /// Uses Fortune's sweep line algorithm (O(n log n) event processing with a linked-list
    /// beach line) for robust computation. Handles degenerate cases including coincident sites,
    /// collinear sites, and fewer than three sites.
    ///
    /// - Parameters:
    ///   - sites: The generating points. Duplicate and near-duplicate sites are filtered.
    ///   - bounds: The bounding box to clip infinite edges to.
    /// - Returns: The computed Voronoi diagram.
    public static func compute(sites: [Vector2D], bounds: BoundingBox2D) -> VoronoiDiagram {
        let builder = FortunesSweep(sites: sites, bounds: bounds)
        return builder.compute()
    }
}

// MARK: - Fortune's Sweep Line Algorithm

/// Implementation of Fortune's sweep line algorithm for Voronoi diagram construction.
///
/// The algorithm sweeps a horizontal line from top (max y) to bottom (min y). It maintains:
/// - A priority queue of events (site events and circle events), ordered by decreasing y.
/// - A beach line: the lower envelope of parabolas defined by each site and the sweep line.
///   Neighboring parabolas intersect at breakpoints that trace out Voronoi edges as the
///   sweep advances.
///
/// Two types of events:
/// - **Site event**: The sweep line reaches a new site. A new parabolic arc is inserted
///   into the beach line, splitting an existing arc. Two new half-edges begin being traced.
/// - **Circle event**: Three consecutive arcs on the beach line become co-circular (the
///   circumscribed circle of their three sites is tangent to the sweep line from above).
///   The middle arc vanishes, a Voronoi vertex is created at the circumcenter, and the
///   two converging edges terminate there while a new edge begins.
private final class FortunesSweep: @unchecked Sendable {

    /// Geometric tolerance for floating-point comparisons.
    static let epsilon: Double = 1e-10

    /// The deduplicated input sites.
    let sites: [Vector2D]

    /// Bounding box for clipping semi-infinite edges.
    let bounds: BoundingBox2D

    /// Priority queue of sweep events, ordered by decreasing y (top-to-bottom sweep).
    private var eventQueue = EventQueue()

    /// Head of the doubly-linked list of arcs forming the beach line.
    private var beachLineHead: Arc?

    /// Completed half-edge records (both endpoints known).
    private var completedEdges: [HalfEdgeRecord] = []

    /// Voronoi vertices discovered during circle events.
    private var voronoiVertices: [VoronoiVertex] = []

    // MARK: - Initialization

    init(sites inputSites: [Vector2D], bounds: BoundingBox2D) {
        self.sites = Self.deduplicateSites(inputSites)
        self.bounds = bounds
    }

    // MARK: - Main Entry Point

    func compute() -> VoronoiDiagram {
        let n = sites.count

        // Degenerate cases.
        if n == 0 {
            return VoronoiDiagram(vertices: [], edges: [], cells: [])
        }
        if n == 1 {
            return VoronoiDiagram(
                vertices: [],
                edges: [],
                cells: [VoronoiCell(site: sites[0], edgeIndices: [])]
            )
        }
        if n == 2 {
            return computeTwoSites()
        }
        if allSitesCollinear() {
            return computeCollinear()
        }

        // Enqueue all site events.
        for i in 0..<n {
            eventQueue.insert(.site(index: i, position: sites[i]))
        }

        // Process events in sweep order (decreasing y).
        while let event = eventQueue.extractMax() {
            switch event {
            case .site(let index, _):
                handleSiteEvent(siteIndex: index)
            case .circle(let arc, let center, let bottomY, _):
                handleCircleEvent(arc: arc, circumcenter: center, bottomY: bottomY)
            }
        }

        // Clip remaining open-ended edges to the bounding box.
        finishEdges()

        return buildDiagram()
    }

    // MARK: - Deduplication

    /// Remove duplicate sites (exact matches and points closer than epsilon).
    /// Returns sites in their original order with duplicates removed.
    private static func deduplicateSites(_ sites: [Vector2D]) -> [Vector2D] {
        guard !sites.isEmpty else { return [] }

        var unique: [Vector2D] = []
        for site in sites {
            let isDuplicate = unique.contains { existing in
                existing.distanceSquared(to: site) < epsilon * epsilon
            }
            if !isDuplicate {
                unique.append(site)
            }
        }
        return unique
    }

    // MARK: - Collinearity Detection

    /// Check whether all sites are collinear (lie on a single line).
    private func allSitesCollinear() -> Bool {
        guard sites.count >= 3 else { return true }

        let a = sites[0]
        let b = sites[1]
        let ab = b - a
        let abLen = ab.length

        if abLen < Self.epsilon {
            return true
        }

        for i in 2..<sites.count {
            let ac = sites[i] - a
            let dist = abs(ab.cross(ac)) / abLen
            if dist > Self.epsilon * 100 {
                return false
            }
        }
        return true
    }

    // MARK: - Special Case: Two Sites

    private func computeTwoSites() -> VoronoiDiagram {
        let s0 = sites[0]
        let s1 = sites[1]
        let mid = s0.lerp(to: s1, t: 0.5)
        let dir = (s1 - s0).perpendicular.normalized

        if let (p0, p1) = clipLineToBounds(origin: mid, direction: dir) {
            let edge = VoronoiEdge(start: p0, end: p1, leftSiteIndex: 0, rightSiteIndex: 1)
            return VoronoiDiagram(
                vertices: [],
                edges: [edge],
                cells: [
                    VoronoiCell(site: s0, edgeIndices: [0]),
                    VoronoiCell(site: s1, edgeIndices: [0]),
                ]
            )
        }

        return VoronoiDiagram(
            vertices: [],
            edges: [],
            cells: [
                VoronoiCell(site: s0, edgeIndices: []),
                VoronoiCell(site: s1, edgeIndices: []),
            ]
        )
    }

    // MARK: - Special Case: Collinear Sites

    private func computeCollinear() -> VoronoiDiagram {
        let dir = (sites[1] - sites[0]).normalized
        let sortedIndices = (0..<sites.count).sorted {
            (sites[$0] - sites[0]).dot(dir) < (sites[$1] - sites[0]).dot(dir)
        }

        var edges: [VoronoiEdge] = []
        var cellEdgeMap = Array(repeating: [Int](), count: sites.count)

        for i in 0..<(sortedIndices.count - 1) {
            let li = sortedIndices[i]
            let ri = sortedIndices[i + 1]
            let mid = sites[li].lerp(to: sites[ri], t: 0.5)
            let perpDir = (sites[ri] - sites[li]).perpendicular.normalized

            if let (p0, p1) = clipLineToBounds(origin: mid, direction: perpDir) {
                let idx = edges.count
                edges.append(VoronoiEdge(start: p0, end: p1, leftSiteIndex: li, rightSiteIndex: ri))
                cellEdgeMap[li].append(idx)
                cellEdgeMap[ri].append(idx)
            }
        }

        let cells = (0..<sites.count).map { VoronoiCell(site: sites[$0], edgeIndices: cellEdgeMap[$0]) }
        return VoronoiDiagram(vertices: [], edges: edges, cells: cells)
    }

    // MARK: - Site Event

    /// Insert a new parabolic arc for `siteIndex` into the beach line.
    private func handleSiteEvent(siteIndex: Int) {
        let site = sites[siteIndex]

        // First arc: just create it.
        guard let _ = beachLineHead else {
            beachLineHead = Arc(siteIndex: siteIndex)
            return
        }

        // Find the arc on the beach line directly above this new site.
        guard let arcAbove = findArcAbove(x: site.x, sweepY: site.y) else {
            // Should not happen after the first arc, but handle gracefully.
            let newArc = Arc(siteIndex: siteIndex)
            // Append at end.
            var last = beachLineHead!
            while let n = last.next { last = n }
            last.next = newArc
            newArc.prev = last
            return
        }

        // Invalidate arcAbove's pending circle event (if any), since it is being split.
        invalidateCircleEvent(arcAbove)

        // Compute the foot: where the new site's vertical line meets arcAbove's parabola.
        let footY = parabolaY(focus: sites[arcAbove.siteIndex], directrixY: site.y, atX: site.x)
        let foot = Vector2D(site.x, footY)

        // Split arcAbove into three arcs: [arcAbove | newArc | rightCopy]
        let newArc = Arc(siteIndex: siteIndex)
        let rightCopy = Arc(siteIndex: arcAbove.siteIndex)

        // Preserve arcAbove's old right edge for the right copy.
        let oldRightEdge = arcAbove.rightHalfEdge

        // Create two new half-edge records starting at the foot.
        let leftEdge = HalfEdgeRecord(
            start: foot,
            leftSiteIndex: arcAbove.siteIndex,
            rightSiteIndex: siteIndex
        )
        let rightEdge = HalfEdgeRecord(
            start: foot,
            leftSiteIndex: siteIndex,
            rightSiteIndex: arcAbove.siteIndex
        )

        // Link: ... <-> arcAbove <-> newArc <-> rightCopy <-> (arcAbove.next) <-> ...
        let afterArc = arcAbove.next

        arcAbove.next = newArc
        newArc.prev = arcAbove
        newArc.next = rightCopy
        rightCopy.prev = newArc
        rightCopy.next = afterArc
        afterArc?.prev = rightCopy

        // Set edge pointers.
        arcAbove.rightHalfEdge = leftEdge
        newArc.leftHalfEdge = leftEdge
        newArc.rightHalfEdge = rightEdge
        rightCopy.leftHalfEdge = rightEdge
        rightCopy.rightHalfEdge = oldRightEdge

        // Check for new circle events involving the changed triples.
        // Left triple: (arcAbove.prev, arcAbove, newArc)
        if let leftNeighbor = arcAbove.prev {
            checkCircleEvent(leftNeighbor, arcAbove, newArc)
        }
        // Right triple: (newArc, rightCopy, rightCopy.next)
        if let rightNeighbor = rightCopy.next {
            checkCircleEvent(newArc, rightCopy, rightNeighbor)
        }
    }

    // MARK: - Circle Event

    /// Remove the middle arc, create a Voronoi vertex at the circumcenter, and start a new edge.
    private func handleCircleEvent(arc middleArc: Arc, circumcenter: Vector2D, bottomY: Double) {
        guard middleArc.circleEvent != nil else { return }

        guard let leftArc = middleArc.prev, let rightArc = middleArc.next else { return }

        // Invalidate circle events on the neighbors (their triples are changing).
        invalidateCircleEvent(leftArc)
        invalidateCircleEvent(rightArc)

        // Record the Voronoi vertex.
        let vtxSites = [leftArc.siteIndex, middleArc.siteIndex, rightArc.siteIndex]
        voronoiVertices.append(VoronoiVertex(position: circumcenter, siteIndices: vtxSites))

        // Terminate the two half-edges that converge at this vertex.
        if let leftEdge = middleArc.leftHalfEdge {
            leftEdge.end = circumcenter
            completedEdges.append(leftEdge)
        }
        if let rightEdge = middleArc.rightHalfEdge {
            rightEdge.end = circumcenter
            completedEdges.append(rightEdge)
        }

        // Start a new half-edge from the vertex, between leftArc and rightArc.
        let newEdge = HalfEdgeRecord(
            start: circumcenter,
            leftSiteIndex: leftArc.siteIndex,
            rightSiteIndex: rightArc.siteIndex
        )

        // Remove the middle arc from the linked list.
        leftArc.next = rightArc
        rightArc.prev = leftArc

        if middleArc === beachLineHead {
            beachLineHead = rightArc
        }

        middleArc.prev = nil
        middleArc.next = nil
        middleArc.circleEvent = nil

        // Update edge pointers for the new adjacency.
        leftArc.rightHalfEdge = newEdge
        rightArc.leftHalfEdge = newEdge

        // Check for new circle events.
        if let ll = leftArc.prev {
            checkCircleEvent(ll, leftArc, rightArc)
        }
        if let rr = rightArc.next {
            checkCircleEvent(leftArc, rightArc, rr)
        }
    }

    // MARK: - Circle Event Detection

    /// Check whether three consecutive arcs form a valid circle event, and if so, enqueue it.
    private func checkCircleEvent(_ left: Arc, _ middle: Arc, _ right: Arc) {
        // Sites must be distinct.
        guard left.siteIndex != middle.siteIndex,
              middle.siteIndex != right.siteIndex,
              left.siteIndex != right.siteIndex else { return }

        let a = sites[left.siteIndex]
        let b = sites[middle.siteIndex]
        let c = sites[right.siteIndex]

        // The three sites must make a right turn (clockwise) for the arcs to converge.
        // In our coordinate system (y increases upward), a negative cross product means
        // a clockwise turn.
        let cross = (b - a).cross(c - a)
        if cross >= 0 {
            return
        }

        // Compute circumcenter.
        guard let center = circumcenter(a, b, c) else { return }

        let radius = center.distance(to: a)
        let bottomY = center.y - radius

        // Create and enqueue the circle event.
        let token = CircleEventToken()
        let event = Event.circle(arc: middle, center: center, bottomY: bottomY, token: token)
        token.isValid = true
        middle.circleEvent = token

        eventQueue.insert(event)
    }

    /// Invalidate a pending circle event on an arc (if any).
    private func invalidateCircleEvent(_ arc: Arc) {
        if let token = arc.circleEvent {
            token.isValid = false
            arc.circleEvent = nil
        }
    }

    // MARK: - Circumcenter

    /// Compute the circumcenter of three points. Returns nil if the points are collinear.
    private func circumcenter(_ a: Vector2D, _ b: Vector2D, _ c: Vector2D) -> Vector2D? {
        let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y))
        guard abs(d) > Self.epsilon else { return nil }

        let aSq = a.x * a.x + a.y * a.y
        let bSq = b.x * b.x + b.y * b.y
        let cSq = c.x * c.x + c.y * c.y

        let ux = (aSq * (b.y - c.y) + bSq * (c.y - a.y) + cSq * (a.y - b.y)) / d
        let uy = (aSq * (c.x - b.x) + bSq * (a.x - c.x) + cSq * (b.x - a.x)) / d

        return Vector2D(ux, uy)
    }

    // MARK: - Beach Line Arc Lookup

    /// Find the arc on the beach line directly above the point (x, sweepY).
    /// Traverses the linked list, computing breakpoints between consecutive arcs.
    private func findArcAbove(x: Double, sweepY: Double) -> Arc? {
        var current = beachLineHead
        while let arc = current {
            guard let nextArc = arc.next else {
                // Last arc extends to +infinity on the right.
                return arc
            }

            let breakX = breakpointX(
                leftSite: sites[arc.siteIndex],
                rightSite: sites[nextArc.siteIndex],
                sweepY: sweepY
            )

            if x <= breakX {
                return arc
            }

            current = nextArc
        }
        return current
    }

    // MARK: - Parabola Mathematics

    /// Evaluate the parabola with given focus at horizontal position `x`, with sweep line at `directrixY`.
    ///
    /// The parabola is the locus of points equidistant from the focus and the directrix.
    /// For a focus at (fx, fy) and directrix y = d:
    ///   y = (x - fx)^2 / (2*(fy - d)) + (fy + d) / 2
    private func parabolaY(focus: Vector2D, directrixY: Double, atX x: Double) -> Double {
        let dp = 2.0 * (focus.y - directrixY)
        if abs(dp) < Self.epsilon {
            return focus.y
        }
        let dx = x - focus.x
        return dx * dx / dp + (focus.y + directrixY) / 2.0
    }

    /// Compute the x-coordinate of the breakpoint between two adjacent parabolas
    /// (the left/right intersection point).
    ///
    /// Given two foci and the sweep line position, solves for the x where the two
    /// parabolas have equal y-values.
    private func breakpointX(leftSite f1: Vector2D, rightSite f2: Vector2D, sweepY: Double) -> Double {
        // If both foci have the same y, the breakpoint is at their x-midpoint.
        if abs(f1.y - f2.y) < Self.epsilon {
            return (f1.x + f2.x) / 2.0
        }

        // If one focus is on the sweep line, its parabola degenerates to a vertical ray
        // and the breakpoint is directly below that focus.
        if abs(f1.y - sweepY) < Self.epsilon {
            return f1.x
        }
        if abs(f2.y - sweepY) < Self.epsilon {
            return f2.x
        }

        // General case: solve the quadratic equation from equating the two parabola formulas.
        //   1/(2*(f1.y - d)) * (x - f1.x)^2 + (f1.y + d)/2
        // = 1/(2*(f2.y - d)) * (x - f2.x)^2 + (f2.y + d)/2
        let a1 = 1.0 / (2.0 * (f1.y - sweepY))
        let a2 = 1.0 / (2.0 * (f2.y - sweepY))

        let a = a1 - a2
        let b = 2.0 * (f2.x * a2 - f1.x * a1)
        let c = a1 * f1.x * f1.x - a2 * f2.x * f2.x + (f1.y - f2.y) / 2.0

        if abs(a) < Self.epsilon {
            if abs(b) < Self.epsilon { return (f1.x + f2.x) / 2.0 }
            return -c / b
        }

        let disc = b * b - 4.0 * a * c
        let sqrtDisc = sqrt(max(0.0, disc))

        let x1 = (-b - sqrtDisc) / (2.0 * a)
        let x2 = (-b + sqrtDisc) / (2.0 * a)

        // The correct root depends on which focus is higher.
        // When f1.y > f2.y, the left breakpoint is the smaller root.
        return f1.y > f2.y ? x1 : x2
    }

    // MARK: - Finish & Clip

    /// Traverse the beach line and clip any remaining open-ended edges to the bounding box.
    private func finishEdges() {
        var arc = beachLineHead
        while let current = arc {
            if let edge = current.rightHalfEdge, edge.end == nil {
                clipOpenEdge(edge)
            }
            arc = current.next
        }
    }

    /// Assign an endpoint to an open-ended half-edge by projecting it to the bounding box.
    private func clipOpenEdge(_ edge: HalfEdgeRecord) {
        guard let start = edge.start else { return }
        guard edge.leftSiteIndex >= 0, edge.rightSiteIndex >= 0 else { return }

        let leftSite = sites[edge.leftSiteIndex]
        let rightSite = sites[edge.rightSiteIndex]

        // The edge lies on the perpendicular bisector of the two sites.
        // Its direction is perpendicular to (rightSite - leftSite).
        let bisectorDir = (rightSite - leftSite).perpendicular.normalized

        // Determine the direction the edge extends from its start point.
        // The edge moves away from the circumcenter (start) along the bisector.
        // We choose the direction such that the edge moves outward.
        let mid = leftSite.lerp(to: rightSite, t: 0.5)
        let startToMid = mid - start
        let dir: Vector2D
        if startToMid.dot(bisectorDir) > 0 {
            dir = bisectorDir
        } else {
            dir = -bisectorDir
        }

        // Find where the ray exits the bounding box.
        if let endpoint = rayBoxExit(origin: start, direction: dir) {
            edge.end = endpoint
            completedEdges.append(edge)
        }
    }

    /// Find the exit point of a ray from the bounding box. Returns nil if the ray doesn't
    /// intersect the box (origin outside and pointing away).
    private func rayBoxExit(origin: Vector2D, direction: Vector2D) -> Vector2D? {
        let bMin = bounds.min
        let bMax = bounds.max

        var tMin = 0.0
        var tMax = Double.infinity

        // X slab.
        if abs(direction.x) > Self.epsilon {
            let t1 = (bMin.x - origin.x) / direction.x
            let t2 = (bMax.x - origin.x) / direction.x
            tMin = max(tMin, min(t1, t2))
            tMax = min(tMax, max(t1, t2))
        } else if origin.x < bMin.x || origin.x > bMax.x {
            return nil
        }

        // Y slab.
        if abs(direction.y) > Self.epsilon {
            let t1 = (bMin.y - origin.y) / direction.y
            let t2 = (bMax.y - origin.y) / direction.y
            tMin = max(tMin, min(t1, t2))
            tMax = min(tMax, max(t1, t2))
        } else if origin.y < bMin.y || origin.y > bMax.y {
            return nil
        }

        if tMax < tMin || tMax < 0 {
            return nil
        }

        return origin + direction * max(tMax, 0.0)
    }

    /// Clip a full line (extending in both directions) to the bounding box.
    /// Returns the two intersection points, or nil if the line misses the box.
    private func clipLineToBounds(origin: Vector2D, direction: Vector2D) -> (Vector2D, Vector2D)? {
        let bMin = bounds.min
        let bMax = bounds.max

        var tMin = -Double.infinity
        var tMax = Double.infinity

        if abs(direction.x) > Self.epsilon {
            let t1 = (bMin.x - origin.x) / direction.x
            let t2 = (bMax.x - origin.x) / direction.x
            tMin = max(tMin, min(t1, t2))
            tMax = min(tMax, max(t1, t2))
        } else if origin.x < bMin.x || origin.x > bMax.x {
            return nil
        }

        if abs(direction.y) > Self.epsilon {
            let t1 = (bMin.y - origin.y) / direction.y
            let t2 = (bMax.y - origin.y) / direction.y
            tMin = max(tMin, min(t1, t2))
            tMax = min(tMax, max(t1, t2))
        } else if origin.y < bMin.y || origin.y > bMax.y {
            return nil
        }

        if tMax < tMin {
            return nil
        }

        return (origin + direction * tMin, origin + direction * tMax)
    }

    // MARK: - Build Output

    /// Assemble the public VoronoiDiagram from internal records.
    private func buildDiagram() -> VoronoiDiagram {
        var finalEdges: [VoronoiEdge] = []
        var cellEdgeMap = Array(repeating: [Int](), count: sites.count)

        for record in completedEdges {
            guard let start = record.start, let end = record.end else { continue }

            // Clip segment to bounding box.
            guard let (s, e) = clipSegment(start: start, end: end) else { continue }

            // Skip degenerate (zero-length) edges.
            if s.distanceSquared(to: e) < Self.epsilon * Self.epsilon { continue }

            let idx = finalEdges.count
            let li = record.leftSiteIndex
            let ri = record.rightSiteIndex

            finalEdges.append(VoronoiEdge(start: s, end: e, leftSiteIndex: li, rightSiteIndex: ri))

            if li >= 0 && li < sites.count { cellEdgeMap[li].append(idx) }
            if ri >= 0 && ri < sites.count { cellEdgeMap[ri].append(idx) }
        }

        let cells = (0..<sites.count).map {
            VoronoiCell(site: sites[$0], edgeIndices: cellEdgeMap[$0])
        }

        return VoronoiDiagram(vertices: voronoiVertices, edges: finalEdges, cells: cells)
    }

    /// Clip a line segment to the bounding box using Cohen-Sutherland.
    private func clipSegment(start: Vector2D, end: Vector2D) -> (Vector2D, Vector2D)? {
        var x0 = start.x, y0 = start.y
        var x1 = end.x, y1 = end.y

        let xMin = bounds.min.x, yMin = bounds.min.y
        let xMax = bounds.max.x, yMax = bounds.max.y

        func outcode(_ x: Double, _ y: Double) -> UInt8 {
            var code: UInt8 = 0
            if x < xMin { code |= 1 }
            if x > xMax { code |= 2 }
            if y < yMin { code |= 4 }
            if y > yMax { code |= 8 }
            return code
        }

        var c0 = outcode(x0, y0)
        var c1 = outcode(x1, y1)

        for _ in 0..<20 {
            if (c0 | c1) == 0 { return (Vector2D(x0, y0), Vector2D(x1, y1)) }
            if (c0 & c1) != 0 { return nil }

            let cOut = c0 != 0 ? c0 : c1
            var x = 0.0, y = 0.0

            if (cOut & 8) != 0 {
                x = x0 + (x1 - x0) * (yMax - y0) / (y1 - y0); y = yMax
            } else if (cOut & 4) != 0 {
                x = x0 + (x1 - x0) * (yMin - y0) / (y1 - y0); y = yMin
            } else if (cOut & 2) != 0 {
                y = y0 + (y1 - y0) * (xMax - x0) / (x1 - x0); x = xMax
            } else if (cOut & 1) != 0 {
                y = y0 + (y1 - y0) * (xMin - x0) / (x1 - x0); x = xMin
            }

            if cOut == c0 { x0 = x; y0 = y; c0 = outcode(x0, y0) }
            else { x1 = x; y1 = y; c1 = outcode(x1, y1) }
        }

        return nil
    }
}

// MARK: - Internal Data Structures

/// A parabolic arc on the beach line, part of a doubly-linked list.
private final class Arc {
    /// Index of the generating site in the `sites` array.
    let siteIndex: Int

    /// Previous arc (to the left) on the beach line.
    var prev: Arc?

    /// Next arc (to the right) on the beach line.
    var next: Arc?

    /// Half-edge being traced on this arc's left breakpoint (between prev and self).
    var leftHalfEdge: FortunesSweep.HalfEdgeRecord?

    /// Half-edge being traced on this arc's right breakpoint (between self and next).
    var rightHalfEdge: FortunesSweep.HalfEdgeRecord?

    /// Token for this arc's pending circle event, or nil if none. Used for O(1) invalidation.
    var circleEvent: CircleEventToken?

    init(siteIndex: Int) {
        self.siteIndex = siteIndex
    }
}

/// Token allowing O(1) invalidation of a circle event in the priority queue.
private final class CircleEventToken {
    var isValid: Bool = true
}

extension FortunesSweep {

    /// A record for a Voronoi edge being traced during the sweep.
    final class HalfEdgeRecord {
        var start: Vector2D?
        var end: Vector2D?
        var leftSiteIndex: Int
        var rightSiteIndex: Int

        init(start: Vector2D? = nil, leftSiteIndex: Int = -1, rightSiteIndex: Int = -1) {
            self.start = start
            self.leftSiteIndex = leftSiteIndex
            self.rightSiteIndex = rightSiteIndex
        }
    }

    // MARK: - Event Type

    /// An event in the sweep line algorithm.
    enum Event {
        /// A site event at the given position.
        case site(index: Int, position: Vector2D)

        /// A circle event. `center` is the circumcenter (becomes the Voronoi vertex).
        /// `bottomY` is the y-coordinate of the bottom of the circumscribed circle,
        /// which is when this event fires during the sweep.
        case circle(arc: Arc, center: Vector2D, bottomY: Double, token: CircleEventToken)

        /// The y-coordinate at which this event should fire.
        var eventY: Double {
            switch self {
            case .site(_, let pos): return pos.y
            case .circle(_, _, let bottomY, _): return bottomY
            }
        }

        /// Tie-breaking x-coordinate.
        var eventX: Double {
            switch self {
            case .site(_, let pos): return pos.x
            case .circle(_, let center, _, _): return center.x
            }
        }

        var isValid: Bool {
            switch self {
            case .site: return true
            case .circle(_, _, _, let token): return token.isValid
            }
        }
    }

    // MARK: - Priority Queue (Max-Heap by Y)

    /// Max-heap priority queue ordered by decreasing y (top-to-bottom sweep).
    /// Ties broken by increasing x (left-to-right).
    struct EventQueue {
        private var heap: [Event] = []

        var isEmpty: Bool { heap.isEmpty }

        mutating func insert(_ event: Event) {
            heap.append(event)
            siftUp(heap.count - 1)
        }

        /// Extract the event with the largest y (highest on the plane), skipping invalidated events.
        mutating func extractMax() -> Event? {
            while !heap.isEmpty {
                let top = heap[0]
                if heap.count == 1 {
                    heap.removeLast()
                    if top.isValid { return top }
                    continue
                }
                heap[0] = heap.removeLast()
                siftDown(0)
                if top.isValid { return top }
            }
            return nil
        }

        /// Returns true if event `a` has higher priority (should be processed first) than `b`.
        private func higherPriority(_ a: Event, _ b: Event) -> Bool {
            let ay = a.eventY
            let by = b.eventY
            if abs(ay - by) > FortunesSweep.epsilon {
                return ay > by  // Larger y processed first (sweep downward).
            }
            return a.eventX < b.eventX  // Tie-break: leftmost first.
        }

        private mutating func siftUp(_ index: Int) {
            var i = index
            while i > 0 {
                let parent = (i - 1) / 2
                if higherPriority(heap[i], heap[parent]) {
                    heap.swapAt(i, parent)
                    i = parent
                } else {
                    break
                }
            }
        }

        private mutating func siftDown(_ index: Int) {
            var i = index
            let n = heap.count
            while true {
                var best = i
                let left = 2 * i + 1
                let right = 2 * i + 2
                if left < n && higherPriority(heap[left], heap[best]) { best = left }
                if right < n && higherPriority(heap[right], heap[best]) { best = right }
                if best == i { break }
                heap.swapAt(i, best)
                i = best
            }
        }
    }
}
