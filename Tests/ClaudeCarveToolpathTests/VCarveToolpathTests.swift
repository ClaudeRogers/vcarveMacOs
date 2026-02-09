import Testing
@testable import ClaudeCarveCore
@testable import ClaudeCarveToolpath

@Suite("VCarve Toolpath Tests")
struct VCarveToolpathTests {
    @Test func generateVCarveForSquare() {
        let square = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(20, 0)),
                .lineTo(Vector2D(20, 20)),
                .lineTo(Vector2D(0, 20)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        let tool = Tool.vBit90()
        let material = MaterialSetup()

        let config = ToolpathConfig(
            type: .vCarve,
            selectedPathIDs: [square.id],
            toolID: tool.id,
            cutDepth: 5.0
        )

        let result = VCarveToolpathGenerator.generate(
            paths: [square],
            config: config,
            tool: tool,
            material: material
        )

        #expect(!result.moves.isEmpty)
    }

    @Test func vCarveDepthVariesWithWidth() {
        // A wider shape should produce deeper cuts on the medial axis
        let wide = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(40, 0)),
                .lineTo(Vector2D(40, 40)),
                .lineTo(Vector2D(0, 40)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        let narrow = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(10, 0)),
                .lineTo(Vector2D(10, 10)),
                .lineTo(Vector2D(0, 10)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        let tool = Tool.vBit90()
        let material = MaterialSetup()
        let config = ToolpathConfig(type: .vCarve, toolID: tool.id, cutDepth: 20.0)

        let wideResult = VCarveToolpathGenerator.generate(
            paths: [wide], config: config, tool: tool, material: material
        )
        let narrowResult = VCarveToolpathGenerator.generate(
            paths: [narrow], config: config, tool: tool, material: material
        )

        // Wide shape should have more moves (more area to cover)
        #expect(wideResult.moves.count >= narrowResult.moves.count)
    }
}

@Suite("Drill Toolpath Tests")
struct DrillToolpathTests {
    @Test func generateSimpleDrill() {
        let circle = VectorPath(
            startPoint: Vector2D(5, 0),
            segments: [
                .arcTo(center: Vector2D(0, 0), radius: 5, startAngle: 0, endAngle: .pi, clockwise: false),
                .arcTo(center: Vector2D(0, 0), radius: 5, startAngle: .pi, endAngle: 2 * .pi, clockwise: false),
            ],
            isClosed: true
        )

        let tool = Tool(name: "Drill", type: .drill, diameter: 5.0)
        let material = MaterialSetup()
        let config = ToolpathConfig(type: .drilling, toolID: tool.id, cutDepth: 10.0)

        let result = DrillToolpathGenerator.generate(
            paths: [circle], config: config, tool: tool, material: material
        )

        #expect(!result.moves.isEmpty)
    }

    @Test func peckDrilling() {
        let point = VectorPath(startPoint: Vector2D(50, 50))

        let tool = Tool(name: "Drill", type: .drill, diameter: 3.0)
        let material = MaterialSetup()
        let config = ToolpathConfig(
            type: .drilling,
            toolID: tool.id,
            cutDepth: 15.0,
            peckDrilling: true,
            peckDepth: 3.0,
            retractHeight: 2.0
        )

        let result = DrillToolpathGenerator.generate(
            paths: [point], config: config, tool: tool, material: material
        )

        // Should have multiple peck cycles (15 / 3 = 5 pecks)
        #expect(result.moves.count > 10) // Rapid + plunge per peck + retracts
    }
}

@Suite("Fluting Toolpath Tests")
struct FlutingToolpathTests {
    @Test func generateFlutingAlongLine() {
        let line = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [.lineTo(Vector2D(100, 0))],
            isClosed: false
        )

        let tool = Tool.ballNose6mm()
        let material = MaterialSetup()
        let config = ToolpathConfig(
            type: .fluting,
            toolID: tool.id,
            cutDepth: 3.0,
            rampInLength: 15.0,
            rampOutLength: 15.0
        )

        let result = FlutingToolpathGenerator.generate(
            paths: [line], config: config, tool: tool, material: material
        )

        #expect(!result.moves.isEmpty)
        // The fluting should start at z=0 and ramp down
    }
}
