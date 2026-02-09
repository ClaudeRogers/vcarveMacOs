import Foundation
import simd

/// A 2D point/vector using double precision.
public struct Vector2D: Hashable, Codable, Sendable {
    public var x: Double
    public var y: Double

    public init(_ x: Double, _ y: Double) {
        self.x = x
        self.y = y
    }

    public init(x: Double, y: Double) {
        self.x = x
        self.y = y
    }

    public static let zero = Vector2D(0, 0)

    // MARK: - Arithmetic

    public static func + (lhs: Vector2D, rhs: Vector2D) -> Vector2D {
        Vector2D(lhs.x + rhs.x, lhs.y + rhs.y)
    }

    public static func - (lhs: Vector2D, rhs: Vector2D) -> Vector2D {
        Vector2D(lhs.x - rhs.x, lhs.y - rhs.y)
    }

    public static func * (lhs: Vector2D, rhs: Double) -> Vector2D {
        Vector2D(lhs.x * rhs, lhs.y * rhs)
    }

    public static func * (lhs: Double, rhs: Vector2D) -> Vector2D {
        Vector2D(lhs * rhs.x, lhs * rhs.y)
    }

    public static func / (lhs: Vector2D, rhs: Double) -> Vector2D {
        Vector2D(lhs.x / rhs, lhs.y / rhs)
    }

    public static func += (lhs: inout Vector2D, rhs: Vector2D) {
        lhs.x += rhs.x
        lhs.y += rhs.y
    }

    public static func -= (lhs: inout Vector2D, rhs: Vector2D) {
        lhs.x -= rhs.x
        lhs.y -= rhs.y
    }

    public static prefix func - (v: Vector2D) -> Vector2D {
        Vector2D(-v.x, -v.y)
    }

    // MARK: - Geometric operations

    public var length: Double {
        sqrt(x * x + y * y)
    }

    public var lengthSquared: Double {
        x * x + y * y
    }

    public var normalized: Vector2D {
        let len = length
        guard len > .ulpOfOne else { return .zero }
        return self / len
    }

    public func dot(_ other: Vector2D) -> Double {
        x * other.x + y * other.y
    }

    /// 2D cross product (returns scalar z-component).
    public func cross(_ other: Vector2D) -> Double {
        x * other.y - y * other.x
    }

    public func distance(to other: Vector2D) -> Double {
        (self - other).length
    }

    public func distanceSquared(to other: Vector2D) -> Double {
        (self - other).lengthSquared
    }

    /// Perpendicular vector (rotated 90° counter-clockwise).
    public var perpendicular: Vector2D {
        Vector2D(-y, x)
    }

    /// Angle in radians from positive x-axis.
    public var angle: Double {
        atan2(y, x)
    }

    /// Create a unit vector at the given angle.
    public static func fromAngle(_ radians: Double) -> Vector2D {
        Vector2D(cos(radians), sin(radians))
    }

    /// Linear interpolation between two points.
    public func lerp(to other: Vector2D, t: Double) -> Vector2D {
        self + (other - self) * t
    }

    /// Rotate this vector by the given angle in radians.
    public func rotated(by angle: Double) -> Vector2D {
        let c = cos(angle)
        let s = sin(angle)
        return Vector2D(x * c - y * s, x * s + y * c)
    }
}

extension Vector2D: CustomStringConvertible {
    public var description: String {
        String(format: "(%.4f, %.4f)", x, y)
    }
}
