import Testing
@testable import VCarveCore
@testable import VCarveGeometry

@Suite("PolygonOffset Tests")
struct PolygonOffsetTests {
    @Test func offsetSquareOutward() {
        let square: [Vector2D] = [
            Vector2D(0, 0),
            Vector2D(10, 0),
            Vector2D(10, 10),
            Vector2D(0, 10),
        ]

        let result = PolygonOffset.offset(polygon: square, distance: 1.0)
        #expect(!result.isEmpty)

        // Outward offset should produce larger polygon
        if let first = result.first {
            #expect(first.count >= 4)
        }
    }

    @Test func offsetSquareInward() {
        let square: [Vector2D] = [
            Vector2D(0, 0),
            Vector2D(10, 0),
            Vector2D(10, 10),
            Vector2D(0, 10),
        ]

        let result = PolygonOffset.offset(polygon: square, distance: -1.0)
        #expect(!result.isEmpty)
    }

    @Test func concentricOffsets() {
        let square: [Vector2D] = [
            Vector2D(0, 0),
            Vector2D(20, 0),
            Vector2D(20, 20),
            Vector2D(0, 20),
        ]

        let result = PolygonOffset.concentricOffsets(polygon: square, stepover: 2.0, inward: true)
        #expect(!result.isEmpty)
    }
}
