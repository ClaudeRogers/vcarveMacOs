import Foundation

/// Axis-aligned bounding box in 2D.
public struct BoundingBox2D: Hashable, Codable, Sendable {
    public var min: Vector2D
    public var max: Vector2D

    public init(min: Vector2D, max: Vector2D) {
        self.min = min
        self.max = max
    }

    public init() {
        self.min = Vector2D(.infinity, .infinity)
        self.max = Vector2D(-.infinity, -.infinity)
    }

    public var width: Double { max.x - min.x }
    public var height: Double { max.y - min.y }
    public var center: Vector2D { (min + max) / 2 }
    public var size: Vector2D { Vector2D(width, height) }
    public var isEmpty: Bool { width <= 0 || height <= 0 }

    public mutating func expand(toInclude point: Vector2D) {
        min = Vector2D(Swift.min(min.x, point.x), Swift.min(min.y, point.y))
        max = Vector2D(Swift.max(max.x, point.x), Swift.max(max.y, point.y))
    }

    public mutating func expand(toInclude other: BoundingBox2D) {
        expand(toInclude: other.min)
        expand(toInclude: other.max)
    }

    public mutating func expand(by margin: Double) {
        min -= Vector2D(margin, margin)
        max += Vector2D(margin, margin)
    }

    public func expanded(by margin: Double) -> BoundingBox2D {
        var copy = self
        copy.expand(by: margin)
        return copy
    }

    public func contains(_ point: Vector2D) -> Bool {
        point.x >= min.x && point.x <= max.x &&
        point.y >= min.y && point.y <= max.y
    }

    public func intersects(_ other: BoundingBox2D) -> Bool {
        min.x <= other.max.x && max.x >= other.min.x &&
        min.y <= other.max.y && max.y >= other.min.y
    }

    /// Create from an array of points.
    public static func enclosing(_ points: [Vector2D]) -> BoundingBox2D {
        var bb = BoundingBox2D()
        for p in points {
            bb.expand(toInclude: p)
        }
        return bb
    }
}

/// Axis-aligned bounding box in 3D.
public struct BoundingBox3D: Hashable, Codable, Sendable {
    public var min: Vector3D
    public var max: Vector3D

    public init(min: Vector3D, max: Vector3D) {
        self.min = min
        self.max = max
    }

    public init() {
        self.min = Vector3D(.infinity, .infinity, .infinity)
        self.max = Vector3D(-.infinity, -.infinity, -.infinity)
    }

    public var width: Double { max.x - min.x }
    public var height: Double { max.y - min.y }
    public var depth: Double { max.z - min.z }
    public var center: Vector3D { (min + max) / 2 }

    public mutating func expand(toInclude point: Vector3D) {
        min = Vector3D(Swift.min(min.x, point.x), Swift.min(min.y, point.y), Swift.min(min.z, point.z))
        max = Vector3D(Swift.max(max.x, point.x), Swift.max(max.y, point.y), Swift.max(max.z, point.z))
    }
}
