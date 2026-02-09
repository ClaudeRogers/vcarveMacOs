import Foundation
import ClaudeCarveCore

// MARK: - NodeHandle

/// Identifies a specific editable node on a path.
public struct NodeHandle: Hashable, Sendable {
    public let pathID: UUID
    public let nodeType: NodeType
    public let segmentIndex: Int // -1 for start point

    public init(pathID: UUID, nodeType: NodeType, segmentIndex: Int) {
        self.pathID = pathID
        self.nodeType = nodeType
        self.segmentIndex = segmentIndex
    }

    public enum NodeType: Hashable, Sendable {
        /// An endpoint (start point or segment endpoint).
        case onCurve
        /// First control point of a bezier.
        case controlPoint1
        /// Second control point of a bezier.
        case controlPoint2
        /// Center of an arc.
        case arcCenter
    }
}

// MARK: - NodeDisplayType

/// The type of a node for display purposes.
public enum NodeDisplayType: Sendable {
    /// Smooth node (tangent-continuous).
    case smooth
    /// Sharp corner node.
    case sharp
    /// Symmetric control points.
    case symmetric
}

// MARK: - NodeEditAction

/// Actions that can be performed during node editing.
public enum NodeEditAction: Sendable {
    case moveNode(handle: NodeHandle, to: Vector2D)
    case addNode(pathID: UUID, afterSegment: Int, at: Vector2D)
    case deleteNode(handle: NodeHandle)
    case convertToLine(pathID: UUID, segmentIndex: Int)
    case convertToCurve(pathID: UUID, segmentIndex: Int)
    case convertToArc(pathID: UUID, segmentIndex: Int)
    case toggleSmooth(handle: NodeHandle)
    case breakPath(pathID: UUID, atSegment: Int)
}

// MARK: - NodeEditor

/// Provides node-level editing operations for vector paths.
public struct NodeEditor: Sendable {

    // Small epsilon for floating-point comparisons.
    private static let epsilon: Double = 1e-9

    // MARK: - getNodes

    /// Get all editable node handles for a path.
    public static func getNodes(for path: VectorPath) -> [NodeHandle] {
        var handles: [NodeHandle] = []
        let pid = path.id

        // Start point is always an on-curve node at segmentIndex -1.
        handles.append(NodeHandle(pathID: pid, nodeType: .onCurve, segmentIndex: -1))

        for (index, segment) in path.segments.enumerated() {
            switch segment {
            case .lineTo:
                handles.append(NodeHandle(pathID: pid, nodeType: .onCurve, segmentIndex: index))

            case .quadTo:
                handles.append(NodeHandle(pathID: pid, nodeType: .controlPoint1, segmentIndex: index))
                handles.append(NodeHandle(pathID: pid, nodeType: .onCurve, segmentIndex: index))

            case .cubicTo:
                handles.append(NodeHandle(pathID: pid, nodeType: .controlPoint1, segmentIndex: index))
                handles.append(NodeHandle(pathID: pid, nodeType: .controlPoint2, segmentIndex: index))
                handles.append(NodeHandle(pathID: pid, nodeType: .onCurve, segmentIndex: index))

            case .arcTo:
                handles.append(NodeHandle(pathID: pid, nodeType: .arcCenter, segmentIndex: index))
                handles.append(NodeHandle(pathID: pid, nodeType: .onCurve, segmentIndex: index))
            }
        }

        return handles
    }

    // MARK: - position

    /// Get the position of a node handle.
    public static func position(of handle: NodeHandle, in path: VectorPath) -> Vector2D? {
        guard handle.pathID == path.id else { return nil }

        if handle.segmentIndex == -1 {
            guard handle.nodeType == .onCurve else { return nil }
            return path.startPoint
        }

        guard handle.segmentIndex >= 0, handle.segmentIndex < path.segments.count else {
            return nil
        }

        let segment = path.segments[handle.segmentIndex]

        switch (segment, handle.nodeType) {
        case (.lineTo(let p), .onCurve):
            return p

        case (.quadTo(let control, _), .controlPoint1):
            return control
        case (.quadTo(_, let end), .onCurve):
            return end

        case (.cubicTo(let c1, _, _), .controlPoint1):
            return c1
        case (.cubicTo(_, let c2, _), .controlPoint2):
            return c2
        case (.cubicTo(_, _, let end), .onCurve):
            return end

        case (.arcTo(let center, _, _, _, _), .arcCenter):
            return center
        case (.arcTo(let center, let radius, _, let endAngle, _), .onCurve):
            return center + Vector2D.fromAngle(endAngle) * radius

        default:
            return nil
        }
    }

    // MARK: - displayType

    /// Get the display type of a node (smooth/sharp/symmetric).
    ///
    /// For an on-curve node, examines the tangent directions from the adjacent segments
    /// to determine if the node is smooth, symmetric, or sharp.
    public static func displayType(of handle: NodeHandle, in path: VectorPath) -> NodeDisplayType {
        guard handle.nodeType == .onCurve else {
            // Control points and arc centers are always displayed as sharp.
            return .sharp
        }

        let inTangent = incomingTangent(at: handle, in: path)
        let outTangent = outgoingTangent(at: handle, in: path)

        guard let inT = inTangent, let outT = outTangent else {
            return .sharp
        }

        let inLen = inT.length
        let outLen = outT.length

        guard inLen > epsilon, outLen > epsilon else {
            return .sharp
        }

        let inNorm = inT / inLen
        let outNorm = outT / outLen

        // Check collinearity: the tangent vectors should be anti-parallel
        // (incoming points toward the node, outgoing points away).
        let dotProduct = inNorm.dot(outNorm)

        // Anti-parallel means dot ~ -1 (incoming tangent direction vs outgoing tangent direction).
        // But since inTangent points into the node and outTangent points out of the node,
        // they should be roughly parallel (dot ~ 1) for smooth.
        if dotProduct > 1.0 - 1e-4 {
            // Tangents are collinear and same direction => smooth.
            // Check if distances to control points are equal => symmetric.
            let inDist = incomingControlDistance(at: handle, in: path)
            let outDist = outgoingControlDistance(at: handle, in: path)

            if let inD = inDist, let outD = outDist {
                let ratio = inD / outD
                if ratio > 0.99 && ratio < 1.01 {
                    return .symmetric
                }
            }
            return .smooth
        }

        return .sharp
    }

    // MARK: - apply

    /// Apply a node edit action to a path, returning the modified path.
    public static func apply(action: NodeEditAction, to path: VectorPath) -> VectorPath {
        switch action {
        case .moveNode(let handle, let newPos):
            return applyMoveNode(handle: handle, to: newPos, path: path)

        case .addNode(let pathID, let afterSegment, let at):
            guard pathID == path.id else { return path }
            return applyAddNode(afterSegment: afterSegment, at: at, path: path)

        case .deleteNode(let handle):
            return applyDeleteNode(handle: handle, path: path)

        case .convertToLine(let pathID, let segmentIndex):
            guard pathID == path.id else { return path }
            return applyConvertToLine(segmentIndex: segmentIndex, path: path)

        case .convertToCurve(let pathID, let segmentIndex):
            guard pathID == path.id else { return path }
            return applyConvertToCurve(segmentIndex: segmentIndex, path: path)

        case .convertToArc(let pathID, let segmentIndex):
            guard pathID == path.id else { return path }
            return applyConvertToArc(segmentIndex: segmentIndex, path: path)

        case .toggleSmooth(let handle):
            return applyToggleSmooth(handle: handle, path: path)

        case .breakPath(let pathID, let atSegment):
            guard pathID == path.id else { return path }
            return applyBreakPath(atSegment: atSegment, path: path)
        }
    }

    // MARK: - hitTest

    /// Find the nearest node handle to a point, within a given radius.
    /// Prioritizes on-curve points over control points.
    public static func hitTest(point: Vector2D, path: VectorPath, radius: Double) -> NodeHandle? {
        let handles = getNodes(for: path)
        let radiusSq = radius * radius

        var bestOnCurve: (NodeHandle, Double)? = nil
        var bestControl: (NodeHandle, Double)? = nil

        for handle in handles {
            guard let pos = position(of: handle, in: path) else { continue }
            let distSq = point.distanceSquared(to: pos)
            guard distSq <= radiusSq else { continue }

            switch handle.nodeType {
            case .onCurve:
                if let (_, bestDistSq) = bestOnCurve {
                    if distSq < bestDistSq {
                        bestOnCurve = (handle, distSq)
                    }
                } else {
                    bestOnCurve = (handle, distSq)
                }
            case .controlPoint1, .controlPoint2, .arcCenter:
                if let (_, bestDistSq) = bestControl {
                    if distSq < bestDistSq {
                        bestControl = (handle, distSq)
                    }
                } else {
                    bestControl = (handle, distSq)
                }
            }
        }

        // Prioritize on-curve points.
        if let (handle, _) = bestOnCurve {
            return handle
        }
        if let (handle, _) = bestControl {
            return handle
        }
        return nil
    }

    // MARK: - nearestPointOnPath

    /// Find the nearest point on a path to a given point (for "add node at cursor" feature).
    /// Returns the segment index, parametric t value, and the closest point.
    public static func nearestPointOnPath(
        _ point: Vector2D,
        path: VectorPath
    ) -> (segmentIndex: Int, t: Double, point: Vector2D)? {
        guard !path.segments.isEmpty else { return nil }

        let sampleCount = 64
        var bestSegment = 0
        var bestT = 0.0
        var bestDistSq = Double.infinity

        var current = path.startPoint

        for (segIndex, segment) in path.segments.enumerated() {
            for step in 0...sampleCount {
                let t = Double(step) / Double(sampleCount)
                let samplePoint = evaluateSegment(segment, from: current, at: t)
                let distSq = point.distanceSquared(to: samplePoint)

                if distSq < bestDistSq {
                    bestDistSq = distSq
                    bestSegment = segIndex
                    bestT = t
                }
            }
            current = segment.endPoint
        }

        // Refine with binary search around the best t.
        var lo = max(0.0, bestT - 1.0 / Double(sampleCount))
        var hi = min(1.0, bestT + 1.0 / Double(sampleCount))
        let segStart = segmentStartPoint(segmentIndex: bestSegment, path: path)
        let seg = path.segments[bestSegment]

        for _ in 0..<20 {
            let mid1 = lo + (hi - lo) / 3.0
            let mid2 = hi - (hi - lo) / 3.0
            let p1 = evaluateSegment(seg, from: segStart, at: mid1)
            let p2 = evaluateSegment(seg, from: segStart, at: mid2)
            let d1 = point.distanceSquared(to: p1)
            let d2 = point.distanceSquared(to: p2)

            if d1 < d2 {
                hi = mid2
            } else {
                lo = mid1
            }
        }

        let refinedT = (lo + hi) / 2.0
        let refinedPoint = evaluateSegment(seg, from: segStart, at: refinedT)

        return (segmentIndex: bestSegment, t: refinedT, point: refinedPoint)
    }

    // MARK: - controlPointLines

    /// Get control point connections for visualization (lines from on-curve to control points).
    public static func controlPointLines(for path: VectorPath) -> [(Vector2D, Vector2D)] {
        var lines: [(Vector2D, Vector2D)] = []
        var current = path.startPoint

        for segment in path.segments {
            switch segment {
            case .lineTo(let p):
                current = p

            case .quadTo(let control, let end):
                lines.append((current, control))
                lines.append((control, end))
                current = end

            case .cubicTo(let c1, let c2, let end):
                lines.append((current, c1))
                lines.append((end, c2))
                current = end

            case .arcTo(let center, let radius, _, let endAngle, _):
                lines.append((current, center))
                let endPt = center + Vector2D.fromAngle(endAngle) * radius
                lines.append((endPt, center))
                current = endPt
            }
        }

        return lines
    }

    // MARK: - Private: Move Node

    private static func applyMoveNode(handle: NodeHandle, to newPos: Vector2D, path: VectorPath) -> VectorPath {
        guard handle.pathID == path.id else { return path }

        var result = path

        if handle.segmentIndex == -1 && handle.nodeType == .onCurve {
            // Moving the start point.
            let delta = newPos - result.startPoint
            result.startPoint = newPos

            // Adjust the control point of the first segment if it is a curve
            // to maintain tangent continuity.
            if !result.segments.isEmpty {
                adjustAdjacentControlPointAfterMove(
                    nodeSegmentIndex: -1,
                    delta: delta,
                    path: &result
                )
            }

            // If the path is closed, adjust the last segment's incoming control point too.
            if result.isClosed, !result.segments.isEmpty {
                adjustAdjacentControlPointBeforeMove(
                    nodeSegmentIndex: -1,
                    delta: delta,
                    path: &result
                )
            }

            return result
        }

        guard handle.segmentIndex >= 0, handle.segmentIndex < result.segments.count else {
            return result
        }

        switch handle.nodeType {
        case .onCurve:
            result = moveOnCurveNode(segmentIndex: handle.segmentIndex, to: newPos, path: result)

        case .controlPoint1:
            result = moveControlPoint1(segmentIndex: handle.segmentIndex, to: newPos, path: result)

        case .controlPoint2:
            result = moveControlPoint2(segmentIndex: handle.segmentIndex, to: newPos, path: result)

        case .arcCenter:
            result = moveArcCenter(segmentIndex: handle.segmentIndex, to: newPos, path: result)
        }

        return result
    }

    private static func moveOnCurveNode(segmentIndex: Int, to newPos: Vector2D, path: VectorPath) -> VectorPath {
        var result = path
        let oldPos: Vector2D

        switch result.segments[segmentIndex] {
        case .lineTo(let p):
            oldPos = p
            result.segments[segmentIndex] = .lineTo(newPos)

        case .quadTo(let control, let end):
            oldPos = end
            result.segments[segmentIndex] = .quadTo(control: control, end: newPos)

        case .cubicTo(let c1, let c2, let end):
            oldPos = end
            result.segments[segmentIndex] = .cubicTo(control1: c1, control2: c2, end: newPos)

        case .arcTo(let center, _, let startAngle, _, let clockwise):
            oldPos = result.segments[segmentIndex].endPoint
            let newRadius = center.distance(to: newPos)
            let newEndAngle = (newPos - center).angle
            result.segments[segmentIndex] = .arcTo(
                center: center,
                radius: newRadius,
                startAngle: startAngle,
                endAngle: newEndAngle,
                clockwise: clockwise
            )
        }

        let delta = newPos - oldPos

        // Adjust the second control point of this segment if it is a cubic.
        if case .cubicTo(let c1, let c2, let end) = result.segments[segmentIndex] {
            let nodeHandle = NodeHandle(pathID: path.id, nodeType: .onCurve, segmentIndex: segmentIndex)
            let dt = displayType(of: nodeHandle, in: path)
            if dt == .smooth || dt == .symmetric {
                result.segments[segmentIndex] = .cubicTo(
                    control1: c1,
                    control2: c2 + delta,
                    end: end
                )
            }
        }

        // Adjust the next segment's first control point.
        adjustAdjacentControlPointAfterMove(
            nodeSegmentIndex: segmentIndex,
            delta: delta,
            path: &result
        )

        return result
    }

    private static func moveControlPoint1(segmentIndex: Int, to newPos: Vector2D, path: VectorPath) -> VectorPath {
        var result = path

        switch result.segments[segmentIndex] {
        case .quadTo(_, let end):
            result.segments[segmentIndex] = .quadTo(control: newPos, end: end)

        case .cubicTo(_, let c2, let end):
            result.segments[segmentIndex] = .cubicTo(control1: newPos, control2: c2, end: end)

        default:
            break
        }

        // If the preceding on-curve node is smooth, mirror to the previous segment's
        // outgoing control point.
        let prevOnCurve: NodeHandle
        if segmentIndex == 0 {
            prevOnCurve = NodeHandle(pathID: path.id, nodeType: .onCurve, segmentIndex: -1)
        } else {
            prevOnCurve = NodeHandle(pathID: path.id, nodeType: .onCurve, segmentIndex: segmentIndex - 1)
        }

        let dt = displayType(of: prevOnCurve, in: path)
        if dt == .smooth || dt == .symmetric {
            guard let anchorPos = position(of: prevOnCurve, in: path) else { return result }
            let direction = anchorPos - newPos
            let mirroredPos: Vector2D

            if dt == .symmetric {
                mirroredPos = anchorPos + direction
            } else {
                // Smooth: maintain direction but keep the original distance.
                let dirLen = direction.length
                guard dirLen > epsilon else { return result }
                let prevControlDist = outgoingControlDistanceFromPrev(
                    segmentIndex: segmentIndex,
                    path: path
                )
                let dist = prevControlDist ?? dirLen
                mirroredPos = anchorPos + direction.normalized * dist
            }

            // Apply the mirror to the previous segment's last control point.
            mirrorToPreviousSegment(
                segmentIndex: segmentIndex,
                mirroredPos: mirroredPos,
                path: &result
            )
        }

        return result
    }

    private static func moveControlPoint2(segmentIndex: Int, to newPos: Vector2D, path: VectorPath) -> VectorPath {
        var result = path

        guard case .cubicTo(let c1, _, let end) = result.segments[segmentIndex] else {
            return result
        }

        result.segments[segmentIndex] = .cubicTo(control1: c1, control2: newPos, end: end)

        // If the on-curve endpoint of this segment is smooth, mirror to the next
        // segment's incoming control point.
        let onCurve = NodeHandle(pathID: path.id, nodeType: .onCurve, segmentIndex: segmentIndex)
        let dt = displayType(of: onCurve, in: path)

        if dt == .smooth || dt == .symmetric {
            let anchorPos = end
            let direction = anchorPos - newPos
            let mirroredPos: Vector2D

            if dt == .symmetric {
                mirroredPos = anchorPos + direction
            } else {
                let dirLen = direction.length
                guard dirLen > epsilon else { return result }
                let nextControlDist = incomingControlDistanceFromNext(
                    segmentIndex: segmentIndex,
                    path: path
                )
                let dist = nextControlDist ?? dirLen
                mirroredPos = anchorPos + direction.normalized * dist
            }

            // Apply to the next segment's first control point.
            mirrorToNextSegment(
                segmentIndex: segmentIndex,
                mirroredPos: mirroredPos,
                path: &result
            )
        }

        return result
    }

    private static func moveArcCenter(segmentIndex: Int, to newPos: Vector2D, path: VectorPath) -> VectorPath {
        var result = path

        guard case .arcTo(_, _, let startAngle, let endAngle, let clockwise) =
            result.segments[segmentIndex] else {
            return result
        }

        // Recalculate the radius based on the start point of this segment.
        let segStart = segmentStartPoint(segmentIndex: segmentIndex, path: result)
        let newRadius = newPos.distance(to: segStart)

        // Recalculate start angle from the new center.
        let newStartAngle = (segStart - newPos).angle

        // Preserve the angular sweep.
        var sweep = endAngle - startAngle
        if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
        if !clockwise && sweep < 0 { sweep += 2.0 * .pi }
        let newEndAngle = newStartAngle + sweep

        result.segments[segmentIndex] = .arcTo(
            center: newPos,
            radius: newRadius,
            startAngle: newStartAngle,
            endAngle: newEndAngle,
            clockwise: clockwise
        )

        return result
    }

    // MARK: - Private: Add Node

    private static func applyAddNode(afterSegment: Int, at point: Vector2D, path: VectorPath) -> VectorPath {
        guard afterSegment >= 0, afterSegment < path.segments.count else { return path }

        // Find the parametric t for the insertion point.
        let segStart = segmentStartPoint(segmentIndex: afterSegment, path: path)
        let segment = path.segments[afterSegment]

        // Find t closest to the given point.
        let t = findClosestT(on: segment, from: segStart, to: point)

        var result = path
        var newSegments: [PathSegment] = []

        for (index, seg) in path.segments.enumerated() {
            if index == afterSegment {
                let start = segmentStartPoint(segmentIndex: index, path: path)
                let (first, second) = splitSegment(seg, from: start, at: t)
                newSegments.append(first)
                newSegments.append(second)
            } else {
                newSegments.append(seg)
            }
        }

        result.segments = newSegments
        return result
    }

    // MARK: - Private: Delete Node

    private static func applyDeleteNode(handle: NodeHandle, path: VectorPath) -> VectorPath {
        guard handle.pathID == path.id, handle.nodeType == .onCurve else { return path }

        var result = path

        if handle.segmentIndex == -1 {
            // Deleting the start point: move start to the first segment's endpoint.
            guard !result.segments.isEmpty else { return path }
            let newStart = result.segments[0].endPoint
            result.startPoint = newStart
            result.segments.removeFirst()
            return result
        }

        let idx = handle.segmentIndex
        guard idx >= 0, idx < result.segments.count else { return path }

        if result.segments.count == 1 {
            // Cannot delete the only segment's endpoint; path would be empty.
            return path
        }

        if idx == result.segments.count - 1 {
            // Last segment: simply remove it.
            result.segments.removeLast()
        } else {
            // Merge segment at idx with segment at idx+1.
            // The merged segment goes from the start of segment idx to the end of segment idx+1.
            let nextEnd = result.segments[idx + 1].endPoint

            // Determine merged segment type based on the more complex of the two segments.
            let merged: PathSegment
            let mergeStart = segmentStartPoint(segmentIndex: idx, path: result)

            switch (result.segments[idx], result.segments[idx + 1]) {
            case (.lineTo, .lineTo):
                merged = .lineTo(nextEnd)

            case (.cubicTo(let c1, _, _), .cubicTo(_, let c2b, let end)):
                // Keep the outer control points, remap to approximate the original shape.
                merged = .cubicTo(control1: c1, control2: c2b, end: end)

            default:
                // Default: create a cubic that approximates a straight connection.
                let c1 = mergeStart.lerp(to: nextEnd, t: 1.0 / 3.0)
                let c2 = mergeStart.lerp(to: nextEnd, t: 2.0 / 3.0)
                merged = .cubicTo(control1: c1, control2: c2, end: nextEnd)
            }

            result.segments[idx] = merged
            result.segments.remove(at: idx + 1)
        }

        return result
    }

    // MARK: - Private: Convert To Line

    private static func applyConvertToLine(segmentIndex: Int, path: VectorPath) -> VectorPath {
        guard segmentIndex >= 0, segmentIndex < path.segments.count else { return path }

        var result = path
        let endPt = result.segments[segmentIndex].endPoint
        result.segments[segmentIndex] = .lineTo(endPt)
        return result
    }

    // MARK: - Private: Convert To Curve

    private static func applyConvertToCurve(segmentIndex: Int, path: VectorPath) -> VectorPath {
        guard segmentIndex >= 0, segmentIndex < path.segments.count else { return path }

        var result = path
        let segment = result.segments[segmentIndex]

        guard case .lineTo(let endPt) = segment else {
            // Already a curve; no conversion needed.
            return result
        }

        let startPt = segmentStartPoint(segmentIndex: segmentIndex, path: result)
        let c1 = startPt.lerp(to: endPt, t: 1.0 / 3.0)
        let c2 = startPt.lerp(to: endPt, t: 2.0 / 3.0)
        result.segments[segmentIndex] = .cubicTo(control1: c1, control2: c2, end: endPt)
        return result
    }

    // MARK: - Private: Convert To Arc

    private static func applyConvertToArc(segmentIndex: Int, path: VectorPath) -> VectorPath {
        guard segmentIndex >= 0, segmentIndex < path.segments.count else { return path }

        var result = path
        let endPt = result.segments[segmentIndex].endPoint
        let startPt = segmentStartPoint(segmentIndex: segmentIndex, path: result)

        // Place the arc center at the midpoint perpendicular to the chord.
        let midpoint = startPt.lerp(to: endPt, t: 0.5)
        let chord = endPt - startPt
        let chordLen = chord.length
        guard chordLen > epsilon else { return path }

        // Default radius: create a semicircular arc with center at the midpoint offset
        // perpendicular to the chord by half the chord length.
        let perpDir = chord.perpendicular.normalized
        let offset = chordLen * 0.5
        let center = midpoint + perpDir * offset

        let radius = center.distance(to: startPt)
        let startAngle = (startPt - center).angle
        let endAngle = (endPt - center).angle

        result.segments[segmentIndex] = .arcTo(
            center: center,
            radius: radius,
            startAngle: startAngle,
            endAngle: endAngle,
            clockwise: false
        )
        return result
    }

    // MARK: - Private: Toggle Smooth

    private static func applyToggleSmooth(handle: NodeHandle, path: VectorPath) -> VectorPath {
        guard handle.pathID == path.id, handle.nodeType == .onCurve else { return path }

        let current = displayType(of: handle, in: path)

        switch current {
        case .sharp:
            // Make smooth: align the adjacent control points to be collinear through this node.
            return makeSmoothAtNode(handle: handle, path: path)
        case .smooth:
            // Make symmetric: equalize distances.
            return makeSymmetricAtNode(handle: handle, path: path)
        case .symmetric:
            // Break smoothness: make sharp (no adjustment).
            return path
        }
    }

    private static func makeSmoothAtNode(handle: NodeHandle, path: VectorPath) -> VectorPath {
        guard let anchorPos = position(of: handle, in: path) else { return path }

        let inT = incomingTangent(at: handle, in: path)
        let outT = outgoingTangent(at: handle, in: path)

        guard let inTangent = inT, let outTangent = outT else { return path }

        let inLen = inTangent.length
        let outLen = outTangent.length
        guard inLen > epsilon, outLen > epsilon else { return path }

        // Compute the average tangent direction.
        let avgDir = (inTangent.normalized + outTangent.normalized).normalized

        guard avgDir.length > epsilon else { return path }

        var result = path

        // Adjust the incoming control point.
        let inControlPos = anchorPos - avgDir * inLen
        setIncomingControlPoint(at: handle, to: inControlPos, path: &result)

        // Adjust the outgoing control point.
        let outControlPos = anchorPos + avgDir * outLen
        setOutgoingControlPoint(at: handle, to: outControlPos, path: &result)

        return result
    }

    private static func makeSymmetricAtNode(handle: NodeHandle, path: VectorPath) -> VectorPath {
        guard let anchorPos = position(of: handle, in: path) else { return path }

        let inT = incomingTangent(at: handle, in: path)
        let outT = outgoingTangent(at: handle, in: path)

        guard let inTangent = inT, let outTangent = outT else { return path }

        let inLen = inTangent.length
        let outLen = outTangent.length
        guard inLen > epsilon, outLen > epsilon else { return path }

        let avgLen = (inLen + outLen) / 2.0
        let avgDir = (inTangent.normalized + outTangent.normalized).normalized

        guard avgDir.length > epsilon else { return path }

        var result = path

        let inControlPos = anchorPos - avgDir * avgLen
        setIncomingControlPoint(at: handle, to: inControlPos, path: &result)

        let outControlPos = anchorPos + avgDir * avgLen
        setOutgoingControlPoint(at: handle, to: outControlPos, path: &result)

        return result
    }

    // MARK: - Private: Break Path

    private static func applyBreakPath(atSegment: Int, path: VectorPath) -> VectorPath {
        guard atSegment >= 0, atSegment < path.segments.count, path.isClosed else {
            return path
        }

        // Break the closed path open at the given segment.
        // The new start point becomes the endpoint of the segment at atSegment.
        var result = path
        result.isClosed = false

        // Rearrange segments so the path starts at the break point.
        let breakEndPoint = result.segments[atSegment].endPoint
        let afterBreak = Array(result.segments[(atSegment + 1)...])
        let beforeBreak = Array(result.segments[0...atSegment])

        result.startPoint = breakEndPoint
        result.segments = afterBreak + beforeBreak

        // Remove the last segment (which was the original break segment, now at the end).
        if !result.segments.isEmpty {
            result.segments.removeLast()
        }

        return result
    }

    // MARK: - Private: Segment Evaluation

    /// Evaluate a segment at parametric value t, given the start point.
    private static func evaluateSegment(_ segment: PathSegment, from start: Vector2D, at t: Double) -> Vector2D {
        switch segment {
        case .lineTo(let end):
            return start.lerp(to: end, t: t)

        case .quadTo(let control, let end):
            let mt = 1.0 - t
            return start * (mt * mt) + control * (2.0 * mt * t) + end * (t * t)

        case .cubicTo(let c1, let c2, let end):
            let mt = 1.0 - t
            let mt2 = mt * mt
            let t2 = t * t
            return start * (mt2 * mt) + c1 * (3.0 * mt2 * t) + c2 * (3.0 * mt * t2) + end * (t2 * t)

        case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
            var sweep = endAngle - startAngle
            if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
            if !clockwise && sweep < 0 { sweep += 2.0 * .pi }
            let angle = startAngle + sweep * t
            return center + Vector2D.fromAngle(angle) * radius
        }
    }

    /// Get the start point of a segment (the endpoint of the previous segment, or path.startPoint).
    private static func segmentStartPoint(segmentIndex: Int, path: VectorPath) -> Vector2D {
        if segmentIndex <= 0 {
            return path.startPoint
        }
        return path.segments[segmentIndex - 1].endPoint
    }

    // MARK: - Private: Segment Splitting

    /// Split a segment at parameter t, returning two new segments.
    private static func splitSegment(
        _ segment: PathSegment,
        from start: Vector2D,
        at t: Double
    ) -> (PathSegment, PathSegment) {
        switch segment {
        case .lineTo(let end):
            let mid = start.lerp(to: end, t: t)
            return (.lineTo(mid), .lineTo(end))

        case .quadTo(let control, let end):
            // De Casteljau for quadratic.
            let a = start.lerp(to: control, t: t)
            let b = control.lerp(to: end, t: t)
            let mid = a.lerp(to: b, t: t)
            return (
                .quadTo(control: a, end: mid),
                .quadTo(control: b, end: end)
            )

        case .cubicTo(let c1, let c2, let end):
            // De Casteljau subdivision for cubic.
            let a = start.lerp(to: c1, t: t)
            let b = c1.lerp(to: c2, t: t)
            let c = c2.lerp(to: end, t: t)
            let d = a.lerp(to: b, t: t)
            let e = b.lerp(to: c, t: t)
            let mid = d.lerp(to: e, t: t)
            return (
                .cubicTo(control1: a, control2: d, end: mid),
                .cubicTo(control1: e, control2: c, end: end)
            )

        case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
            var sweep = endAngle - startAngle
            if clockwise && sweep > 0 { sweep -= 2.0 * .pi }
            if !clockwise && sweep < 0 { sweep += 2.0 * .pi }
            let midAngle = startAngle + sweep * t
            return (
                .arcTo(center: center, radius: radius, startAngle: startAngle,
                       endAngle: midAngle, clockwise: clockwise),
                .arcTo(center: center, radius: radius, startAngle: midAngle,
                       endAngle: endAngle, clockwise: clockwise)
            )
        }
    }

    /// Find the parameter t on a segment closest to a given point.
    private static func findClosestT(
        on segment: PathSegment,
        from start: Vector2D,
        to point: Vector2D
    ) -> Double {
        let sampleCount = 128
        var bestT = 0.5
        var bestDistSq = Double.infinity

        for step in 0...sampleCount {
            let t = Double(step) / Double(sampleCount)
            let p = evaluateSegment(segment, from: start, at: t)
            let distSq = point.distanceSquared(to: p)
            if distSq < bestDistSq {
                bestDistSq = distSq
                bestT = t
            }
        }

        // Refine with ternary search.
        var lo = max(0.0, bestT - 1.0 / Double(sampleCount))
        var hi = min(1.0, bestT + 1.0 / Double(sampleCount))

        for _ in 0..<20 {
            let mid1 = lo + (hi - lo) / 3.0
            let mid2 = hi - (hi - lo) / 3.0
            let d1 = point.distanceSquared(to: evaluateSegment(segment, from: start, at: mid1))
            let d2 = point.distanceSquared(to: evaluateSegment(segment, from: start, at: mid2))
            if d1 < d2 {
                hi = mid2
            } else {
                lo = mid1
            }
        }

        return (lo + hi) / 2.0
    }

    // MARK: - Private: Tangent Helpers

    /// Compute the incoming tangent vector at an on-curve node.
    /// The tangent points from the previous control point toward the node.
    private static func incomingTangent(at handle: NodeHandle, in path: VectorPath) -> Vector2D? {
        guard handle.nodeType == .onCurve else { return nil }

        let nodePos: Vector2D
        let segIndex: Int

        if handle.segmentIndex == -1 {
            nodePos = path.startPoint
            // For start point of a closed path, incoming is from the last segment.
            guard path.isClosed, !path.segments.isEmpty else { return nil }
            segIndex = path.segments.count - 1
        } else {
            guard let pos = position(of: handle, in: path) else { return nil }
            nodePos = pos
            segIndex = handle.segmentIndex
        }

        guard segIndex >= 0, segIndex < path.segments.count else { return nil }

        let segment = path.segments[segIndex]

        switch segment {
        case .lineTo:
            let start = segmentStartPoint(segmentIndex: segIndex, path: path)
            return (nodePos - start).normalized

        case .quadTo(let control, _):
            return (nodePos - control).normalized

        case .cubicTo(_, let c2, _):
            let dir = nodePos - c2
            return dir.length > epsilon ? dir.normalized : nil

        case .arcTo(_, let radius, _, let endAngle, let clockwise):
            // Tangent to arc at end.
            let radialDir = Vector2D.fromAngle(endAngle)
            let tangent = clockwise
                ? Vector2D(radialDir.y, -radialDir.x)
                : Vector2D(-radialDir.y, radialDir.x)
            return radius > epsilon ? tangent : nil
        }
    }

    /// Compute the outgoing tangent vector at an on-curve node.
    /// The tangent points from the node toward the next control point.
    private static func outgoingTangent(at handle: NodeHandle, in path: VectorPath) -> Vector2D? {
        guard handle.nodeType == .onCurve else { return nil }

        let nodePos: Vector2D
        let nextSegIndex: Int

        if handle.segmentIndex == -1 {
            nodePos = path.startPoint
            nextSegIndex = 0
        } else {
            guard let pos = position(of: handle, in: path) else { return nil }
            nodePos = pos

            if handle.segmentIndex + 1 < path.segments.count {
                nextSegIndex = handle.segmentIndex + 1
            } else if path.isClosed {
                nextSegIndex = 0
            } else {
                return nil
            }
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return nil }

        let segment = path.segments[nextSegIndex]

        switch segment {
        case .lineTo(let end):
            return (end - nodePos).normalized

        case .quadTo(let control, _):
            return (control - nodePos).normalized

        case .cubicTo(let c1, _, _):
            let dir = c1 - nodePos
            return dir.length > epsilon ? dir.normalized : nil

        case .arcTo(_, let radius, let startAngle, _, let clockwise):
            let radialDir = Vector2D.fromAngle(startAngle)
            let tangent = clockwise
                ? Vector2D(radialDir.y, -radialDir.x)
                : Vector2D(-radialDir.y, radialDir.x)
            return radius > epsilon ? tangent : nil
        }
    }

    /// Get the distance from an on-curve node to the incoming control point.
    private static func incomingControlDistance(at handle: NodeHandle, in path: VectorPath) -> Double? {
        guard handle.nodeType == .onCurve else { return nil }

        let segIndex: Int
        if handle.segmentIndex == -1 {
            guard path.isClosed, !path.segments.isEmpty else { return nil }
            segIndex = path.segments.count - 1
        } else {
            segIndex = handle.segmentIndex
        }

        guard segIndex >= 0, segIndex < path.segments.count else { return nil }
        guard let nodePos = position(of: handle, in: path) else { return nil }

        let segment = path.segments[segIndex]

        switch segment {
        case .cubicTo(_, let c2, _):
            return nodePos.distance(to: c2)
        case .quadTo(let control, _):
            return nodePos.distance(to: control)
        default:
            return nil
        }
    }

    /// Get the distance from an on-curve node to the outgoing control point.
    private static func outgoingControlDistance(at handle: NodeHandle, in path: VectorPath) -> Double? {
        guard handle.nodeType == .onCurve else { return nil }

        let nextSegIndex: Int
        if handle.segmentIndex == -1 {
            nextSegIndex = 0
        } else if handle.segmentIndex + 1 < path.segments.count {
            nextSegIndex = handle.segmentIndex + 1
        } else if path.isClosed {
            nextSegIndex = 0
        } else {
            return nil
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return nil }
        guard let nodePos = position(of: handle, in: path) else { return nil }

        let segment = path.segments[nextSegIndex]

        switch segment {
        case .cubicTo(let c1, _, _):
            return nodePos.distance(to: c1)
        case .quadTo(let control, _):
            return nodePos.distance(to: control)
        default:
            return nil
        }
    }

    // MARK: - Private: Control Point Mirroring Helpers

    private static func outgoingControlDistanceFromPrev(segmentIndex: Int, path: VectorPath) -> Double? {
        let prevSegIndex: Int
        if segmentIndex == 0 {
            guard path.isClosed, !path.segments.isEmpty else { return nil }
            prevSegIndex = path.segments.count - 1
        } else {
            prevSegIndex = segmentIndex - 1
        }

        guard prevSegIndex >= 0, prevSegIndex < path.segments.count else { return nil }

        let prevSeg = path.segments[prevSegIndex]
        let prevEnd = prevSeg.endPoint

        switch prevSeg {
        case .cubicTo(_, let c2, _):
            return prevEnd.distance(to: c2)
        case .quadTo(let control, _):
            return prevEnd.distance(to: control)
        default:
            return nil
        }
    }

    private static func incomingControlDistanceFromNext(segmentIndex: Int, path: VectorPath) -> Double? {
        let nextSegIndex: Int
        if segmentIndex + 1 < path.segments.count {
            nextSegIndex = segmentIndex + 1
        } else if path.isClosed {
            nextSegIndex = 0
        } else {
            return nil
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return nil }

        let nextSeg = path.segments[nextSegIndex]
        let nextStart = segmentStartPoint(segmentIndex: nextSegIndex, path: path)

        switch nextSeg {
        case .cubicTo(let c1, _, _):
            return nextStart.distance(to: c1)
        case .quadTo(let control, _):
            return nextStart.distance(to: control)
        default:
            return nil
        }
    }

    /// Mirror a position to the previous segment's outgoing control point.
    private static func mirrorToPreviousSegment(
        segmentIndex: Int,
        mirroredPos: Vector2D,
        path: inout VectorPath
    ) {
        let prevSegIndex: Int
        if segmentIndex == 0 {
            guard path.isClosed, !path.segments.isEmpty else { return }
            prevSegIndex = path.segments.count - 1
        } else {
            prevSegIndex = segmentIndex - 1
        }

        guard prevSegIndex >= 0, prevSegIndex < path.segments.count else { return }

        switch path.segments[prevSegIndex] {
        case .cubicTo(let c1, _, let end):
            path.segments[prevSegIndex] = .cubicTo(control1: c1, control2: mirroredPos, end: end)
        case .quadTo(_, let end):
            path.segments[prevSegIndex] = .quadTo(control: mirroredPos, end: end)
        default:
            break
        }
    }

    /// Mirror a position to the next segment's incoming control point.
    private static func mirrorToNextSegment(
        segmentIndex: Int,
        mirroredPos: Vector2D,
        path: inout VectorPath
    ) {
        let nextSegIndex: Int
        if segmentIndex + 1 < path.segments.count {
            nextSegIndex = segmentIndex + 1
        } else if path.isClosed {
            nextSegIndex = 0
        } else {
            return
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return }

        switch path.segments[nextSegIndex] {
        case .cubicTo(_, let c2, let end):
            path.segments[nextSegIndex] = .cubicTo(control1: mirroredPos, control2: c2, end: end)
        case .quadTo(_, let end):
            path.segments[nextSegIndex] = .quadTo(control: mirroredPos, end: end)
        default:
            break
        }
    }

    // MARK: - Private: Adjacent Control Point Adjustment

    /// Adjust the control point of the next segment after an on-curve node is moved.
    private static func adjustAdjacentControlPointAfterMove(
        nodeSegmentIndex: Int,
        delta: Vector2D,
        path: inout VectorPath
    ) {
        let nextSegIndex: Int
        if nodeSegmentIndex == -1 {
            nextSegIndex = 0
        } else if nodeSegmentIndex + 1 < path.segments.count {
            nextSegIndex = nodeSegmentIndex + 1
        } else if path.isClosed {
            nextSegIndex = 0
        } else {
            return
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return }

        switch path.segments[nextSegIndex] {
        case .cubicTo(let c1, let c2, let end):
            path.segments[nextSegIndex] = .cubicTo(control1: c1 + delta, control2: c2, end: end)
        case .quadTo(let control, let end):
            path.segments[nextSegIndex] = .quadTo(control: control + delta, end: end)
        default:
            break
        }
    }

    /// Adjust the control point of the previous segment before an on-curve node is moved.
    private static func adjustAdjacentControlPointBeforeMove(
        nodeSegmentIndex: Int,
        delta: Vector2D,
        path: inout VectorPath
    ) {
        let prevSegIndex: Int
        if nodeSegmentIndex == -1 {
            guard path.isClosed, !path.segments.isEmpty else { return }
            prevSegIndex = path.segments.count - 1
        } else if nodeSegmentIndex > 0 {
            prevSegIndex = nodeSegmentIndex - 1
        } else if path.isClosed {
            prevSegIndex = path.segments.count - 1
        } else {
            return
        }

        guard prevSegIndex >= 0, prevSegIndex < path.segments.count else { return }

        switch path.segments[prevSegIndex] {
        case .cubicTo(let c1, let c2, let end):
            path.segments[prevSegIndex] = .cubicTo(control1: c1, control2: c2 + delta, end: end)
        case .quadTo(let control, let end):
            path.segments[prevSegIndex] = .quadTo(control: control + delta, end: end)
        default:
            break
        }
    }

    // MARK: - Private: Set Control Points at a Node

    /// Set the incoming control point for the segment ending at an on-curve node.
    private static func setIncomingControlPoint(
        at handle: NodeHandle,
        to pos: Vector2D,
        path: inout VectorPath
    ) {
        let segIndex: Int
        if handle.segmentIndex == -1 {
            guard path.isClosed, !path.segments.isEmpty else { return }
            segIndex = path.segments.count - 1
        } else {
            segIndex = handle.segmentIndex
        }

        guard segIndex >= 0, segIndex < path.segments.count else { return }

        switch path.segments[segIndex] {
        case .cubicTo(let c1, _, let end):
            path.segments[segIndex] = .cubicTo(control1: c1, control2: pos, end: end)
        case .quadTo(_, let end):
            path.segments[segIndex] = .quadTo(control: pos, end: end)
        default:
            break
        }
    }

    /// Set the outgoing control point for the segment starting at an on-curve node.
    private static func setOutgoingControlPoint(
        at handle: NodeHandle,
        to pos: Vector2D,
        path: inout VectorPath
    ) {
        let nextSegIndex: Int
        if handle.segmentIndex == -1 {
            nextSegIndex = 0
        } else if handle.segmentIndex + 1 < path.segments.count {
            nextSegIndex = handle.segmentIndex + 1
        } else if path.isClosed {
            nextSegIndex = 0
        } else {
            return
        }

        guard nextSegIndex >= 0, nextSegIndex < path.segments.count else { return }

        switch path.segments[nextSegIndex] {
        case .cubicTo(_, let c2, let end):
            path.segments[nextSegIndex] = .cubicTo(control1: pos, control2: c2, end: end)
        case .quadTo(_, let end):
            path.segments[nextSegIndex] = .quadTo(control: pos, end: end)
        default:
            break
        }
    }
}
