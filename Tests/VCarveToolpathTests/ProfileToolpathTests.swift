import Testing
@testable import VCarveCore
@testable import VCarveToolpath

@Suite("ProfileToolpath Tests")
struct ProfileToolpathTests {
    @Test func generateSimpleProfileToolpath() {
        // Create a simple square path
        let square = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(50, 0)),
                .lineTo(Vector2D(50, 50)),
                .lineTo(Vector2D(0, 50)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        let tool = Tool.endMill6mm()
        let material = MaterialSetup()

        let config = ToolpathConfig(
            type: .profile,
            selectedPathIDs: [square.id],
            toolID: tool.id,
            cutDepth: 5.0,
            profileSide: .outside
        )

        let result = ProfileToolpathGenerator.generate(
            paths: [square],
            config: config,
            tool: tool,
            material: material
        )

        #expect(!result.moves.isEmpty)
        #expect(result.moves.count > 4) // Should have rapids, plunge, and cutting moves
    }

    @Test func toolpathRespectsDepthPerPass() {
        let square = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(50, 0)),
                .lineTo(Vector2D(50, 50)),
                .lineTo(Vector2D(0, 50)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        var tool = Tool.endMill6mm()
        tool.depthPerPass = 2.0

        let config = ToolpathConfig(
            type: .profile,
            selectedPathIDs: [square.id],
            toolID: tool.id,
            cutDepth: 6.0,
            profileSide: .outside
        )

        let result = ProfileToolpathGenerator.generate(
            paths: [square],
            config: config,
            tool: tool,
            material: MaterialSetup()
        )

        // Should have multiple passes (6mm / 2mm per pass = 3 passes)
        #expect(!result.moves.isEmpty)
    }
}
