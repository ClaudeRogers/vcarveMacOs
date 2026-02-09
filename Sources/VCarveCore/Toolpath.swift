import Foundation

/// A single motion command in a toolpath.
public enum MotionType: Codable, Sendable {
    case rapid          // G0 - rapid positioning
    case linear         // G1 - linear interpolation at feed rate
    case cwArc          // G2 - clockwise arc
    case ccwArc         // G3 - counter-clockwise arc
}

/// A single move within a toolpath.
public struct ToolpathMove: Codable, Sendable {
    public var type: MotionType
    public var position: Vector3D
    public var feedRate: Double?    // nil = use rapid/previous feed
    public var arcCenter: Vector2D? // Only for arc moves (I, J values)
    public var spindleSpeed: Double?

    public init(
        type: MotionType,
        position: Vector3D,
        feedRate: Double? = nil,
        arcCenter: Vector2D? = nil,
        spindleSpeed: Double? = nil
    ) {
        self.type = type
        self.position = position
        self.feedRate = feedRate
        self.arcCenter = arcCenter
        self.spindleSpeed = spindleSpeed
    }

    public static func rapid(to position: Vector3D) -> ToolpathMove {
        ToolpathMove(type: .rapid, position: position)
    }

    public static func linear(to position: Vector3D, feed: Double) -> ToolpathMove {
        ToolpathMove(type: .linear, position: position, feedRate: feed)
    }

    public static func plunge(to z: Double, at xy: Vector2D, feed: Double) -> ToolpathMove {
        ToolpathMove(type: .linear, position: Vector3D(xy, z: z), feedRate: feed)
    }
}

/// A computed toolpath — the result of toolpath generation.
public struct ComputedToolpath: Identifiable, Codable, Sendable {
    public let id: UUID
    public var configID: UUID       // Link back to the ToolpathConfig
    public var toolID: UUID
    public var moves: [ToolpathMove]
    public var estimatedTime: TimeInterval  // seconds
    public var totalDistance: Double         // in job units

    public init(
        id: UUID = UUID(),
        configID: UUID,
        toolID: UUID,
        moves: [ToolpathMove] = [],
        estimatedTime: TimeInterval = 0,
        totalDistance: Double = 0
    ) {
        self.id = id
        self.configID = configID
        self.toolID = toolID
        self.moves = moves
        self.estimatedTime = estimatedTime
        self.totalDistance = totalDistance
    }

    /// Calculate estimated machining time from moves.
    public mutating func calculateEstimates(rapidRate: Double = 5000) {
        var time = 0.0
        var distance = 0.0
        var prev = Vector3D.zero

        for move in moves {
            let d = prev.distance(to: move.position)
            distance += d

            let rate: Double
            switch move.type {
            case .rapid:
                rate = rapidRate
            case .linear, .cwArc, .ccwArc:
                rate = move.feedRate ?? rapidRate
            }

            if rate > 0 {
                time += d / rate  // time in minutes
            }
            prev = move.position
        }

        estimatedTime = time * 60  // convert to seconds
        totalDistance = distance
    }

    /// The bounding box of all moves.
    public var boundingBox: BoundingBox3D {
        var bb = BoundingBox3D()
        for move in moves {
            bb.expand(toInclude: move.position)
        }
        return bb
    }
}
