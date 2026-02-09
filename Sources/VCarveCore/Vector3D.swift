import Foundation

/// A 3D point/vector using double precision.
public struct Vector3D: Hashable, Codable, Sendable {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public init(_ v2: Vector2D, z: Double = 0) {
        self.x = v2.x
        self.y = v2.y
        self.z = z
    }

    public static let zero = Vector3D(0, 0, 0)

    public var xy: Vector2D {
        Vector2D(x, y)
    }

    // MARK: - Arithmetic

    public static func + (lhs: Vector3D, rhs: Vector3D) -> Vector3D {
        Vector3D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z)
    }

    public static func - (lhs: Vector3D, rhs: Vector3D) -> Vector3D {
        Vector3D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)
    }

    public static func * (lhs: Vector3D, rhs: Double) -> Vector3D {
        Vector3D(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs)
    }

    public static func * (lhs: Double, rhs: Vector3D) -> Vector3D {
        Vector3D(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z)
    }

    public static func / (lhs: Vector3D, rhs: Double) -> Vector3D {
        Vector3D(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs)
    }

    public static prefix func - (v: Vector3D) -> Vector3D {
        Vector3D(-v.x, -v.y, -v.z)
    }

    // MARK: - Geometric operations

    public var length: Double {
        sqrt(x * x + y * y + z * z)
    }

    public var lengthSquared: Double {
        x * x + y * y + z * z
    }

    public var normalized: Vector3D {
        let len = length
        guard len > .ulpOfOne else { return .zero }
        return self / len
    }

    public func dot(_ other: Vector3D) -> Double {
        x * other.x + y * other.y + z * other.z
    }

    public func cross(_ other: Vector3D) -> Vector3D {
        Vector3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        )
    }

    public func distance(to other: Vector3D) -> Double {
        (self - other).length
    }

    public func lerp(to other: Vector3D, t: Double) -> Vector3D {
        self + (other - self) * t
    }
}

extension Vector3D: CustomStringConvertible {
    public var description: String {
        String(format: "(%.4f, %.4f, %.4f)", x, y, z)
    }
}
