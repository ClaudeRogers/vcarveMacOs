import Testing
@testable import ClaudeCarveCore

@Suite("VectorPath Tests")
struct PathTests {
    @Test func closedSquarePath() {
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(10, 0)),
                .lineTo(Vector2D(10, 10)),
                .lineTo(Vector2D(0, 10)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        #expect(path.isClosed)
        #expect(path.points.count == 5) // start + 4 segments
    }

    @Test func pathLength() {
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(10, 0)),
                .lineTo(Vector2D(10, 10)),
            ],
            isClosed: false
        )

        let length = path.length
        #expect(abs(length - 20.0) < 0.1) // 10 + 10
    }

    @Test func signedAreaCCW() {
        // Counter-clockwise square
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(10, 0)),
                .lineTo(Vector2D(10, 10)),
                .lineTo(Vector2D(0, 10)),
            ],
            isClosed: true
        )

        #expect(path.signedArea > 0) // CCW = positive area
        #expect(path.isCounterClockwise)
    }

    @Test func signedAreaCW() {
        // Clockwise square
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(0, 10)),
                .lineTo(Vector2D(10, 10)),
                .lineTo(Vector2D(10, 0)),
            ],
            isClosed: true
        )

        #expect(path.signedArea < 0) // CW = negative area
        #expect(!path.isCounterClockwise)
    }

    @Test func boundingBox() {
        let path = VectorPath(
            startPoint: Vector2D(-5, -3),
            segments: [
                .lineTo(Vector2D(15, -3)),
                .lineTo(Vector2D(15, 7)),
                .lineTo(Vector2D(-5, 7)),
            ],
            isClosed: true
        )

        let bb = path.boundingBox
        #expect(abs(bb.min.x - (-5)) < 0.5)
        #expect(abs(bb.max.x - 15) < 0.5)
        #expect(abs(bb.width - 20) < 1.0)
        #expect(abs(bb.height - 10) < 1.0)
    }

    @Test func flattenedPoints() {
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [.lineTo(Vector2D(10, 0))],
            isClosed: false
        )

        let points = path.flattenedPoints(tolerance: 1.0)
        #expect(points.count == 2)
        #expect(abs(points[0].x) < 0.001)
        #expect(abs(points[1].x - 10) < 0.001)
    }

    @Test func reversedPath() {
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(10, 0)),
                .lineTo(Vector2D(10, 10)),
            ],
            isClosed: false
        )

        let reversed = path.reversed()
        let points = reversed.flattenedPoints(tolerance: 0.01)
        #expect(abs(points.first!.x - 10) < 0.1)
        #expect(abs(points.first!.y - 10) < 0.1)
    }
}
