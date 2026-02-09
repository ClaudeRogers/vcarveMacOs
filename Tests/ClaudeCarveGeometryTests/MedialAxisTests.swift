import Testing
@testable import ClaudeCarveCore
@testable import ClaudeCarveGeometry

@Suite("MedialAxis Tests")
struct MedialAxisTests {
    @Test func computeMedialAxisForSquare() {
        let square: [Vector2D] = [
            Vector2D(0, 0),
            Vector2D(20, 0),
            Vector2D(20, 20),
            Vector2D(0, 20),
        ]

        let medialAxis = MedialAxis.compute(for: square, resolution: 1.0)
        #expect(!medialAxis.segments.isEmpty)
    }

    @Test func medialAxisPointHasRadius() {
        let square: [Vector2D] = [
            Vector2D(0, 0),
            Vector2D(20, 0),
            Vector2D(20, 20),
            Vector2D(0, 20),
        ]

        let medialAxis = MedialAxis.compute(for: square, resolution: 1.0)

        // All medial axis points should have positive radius
        for segment in medialAxis.segments {
            #expect(segment.start.radius > 0)
            #expect(segment.end.radius > 0)
        }
    }

    @Test func medialAxisInterpolation() {
        let start = MedialAxisPoint(position: Vector2D(0, 0), radius: 2.0)
        let end = MedialAxisPoint(position: Vector2D(10, 0), radius: 5.0)
        let segment = MedialAxisSegment(start: start, end: end)

        let mid = segment.interpolate(t: 0.5)
        #expect(abs(mid.position.x - 5.0) < 0.001)
        #expect(abs(mid.radius - 3.5) < 0.001) // Linearly interpolated
    }

    @Test func emptyPolygonReturnsEmpty() {
        let empty: [Vector2D] = []
        let result = MedialAxis.compute(for: empty)
        #expect(result.segments.isEmpty)
    }

    @Test func degeneratePolygonReturnsEmpty() {
        let line: [Vector2D] = [Vector2D(0, 0), Vector2D(10, 0)]
        let result = MedialAxis.compute(for: line)
        #expect(result.segments.isEmpty)
    }
}
