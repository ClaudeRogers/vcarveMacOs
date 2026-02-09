import Testing
@testable import VCarveCore

@Suite("Vector2D Tests")
struct Vector2DTests {
    @Test func addition() {
        let a = Vector2D(1, 2)
        let b = Vector2D(3, 4)
        let result = a + b
        #expect(result.x == 4)
        #expect(result.y == 6)
    }

    @Test func subtraction() {
        let a = Vector2D(5, 7)
        let b = Vector2D(2, 3)
        let result = a - b
        #expect(result.x == 3)
        #expect(result.y == 4)
    }

    @Test func dotProduct() {
        let a = Vector2D(1, 0)
        let b = Vector2D(0, 1)
        #expect(a.dot(b) == 0) // perpendicular

        let c = Vector2D(1, 0)
        let d = Vector2D(1, 0)
        #expect(c.dot(d) == 1) // parallel
    }

    @Test func crossProduct() {
        let a = Vector2D(1, 0)
        let b = Vector2D(0, 1)
        #expect(a.cross(b) == 1) // CCW

        let c = Vector2D(0, 1)
        let d = Vector2D(1, 0)
        #expect(c.cross(d) == -1) // CW
    }

    @Test func length() {
        let v = Vector2D(3, 4)
        #expect(abs(v.length - 5.0) < 1e-10)
    }

    @Test func normalized() {
        let v = Vector2D(3, 4)
        let n = v.normalized
        #expect(abs(n.length - 1.0) < 1e-10)
    }

    @Test func distance() {
        let a = Vector2D(0, 0)
        let b = Vector2D(3, 4)
        #expect(abs(a.distance(to: b) - 5.0) < 1e-10)
    }

    @Test func perpendicular() {
        let v = Vector2D(1, 0)
        let p = v.perpendicular
        #expect(abs(p.x - 0) < 1e-10)
        #expect(abs(p.y - 1) < 1e-10)
    }

    @Test func rotation() {
        let v = Vector2D(1, 0)
        let rotated = v.rotated(by: .pi / 2)
        #expect(abs(rotated.x) < 1e-10)
        #expect(abs(rotated.y - 1) < 1e-10)
    }

    @Test func lerp() {
        let a = Vector2D(0, 0)
        let b = Vector2D(10, 10)
        let mid = a.lerp(to: b, t: 0.5)
        #expect(abs(mid.x - 5) < 1e-10)
        #expect(abs(mid.y - 5) < 1e-10)
    }
}
