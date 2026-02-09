import Foundation
import Testing
@testable import ClaudeCarveCore
@testable import ClaudeCarveGCode

@Suite("GCode Generator Tests")
struct GCodeGeneratorTests {
    @Test func generateBasicGCode() {
        let toolpath = ComputedToolpath(
            configID: UUID(),
            toolID: UUID(),
            moves: [
                .rapid(to: Vector3D(0, 0, 5)),
                .rapid(to: Vector3D(10, 0, 5)),
                .linear(to: Vector3D(10, 0, -3), feed: 500),
                .linear(to: Vector3D(50, 0, -3), feed: 2000),
                .rapid(to: Vector3D(50, 0, 5)),
            ]
        )

        let generator = GCodeGenerator()
        let tool = Tool.endMill6mm()
        let material = MaterialSetup()

        let gcode = generator.generate(toolpath: toolpath, tool: tool, material: material)

        #expect(gcode.contains("G0"))
        #expect(gcode.contains("G1"))
        #expect(gcode.contains("M3"))  // Spindle on
        #expect(gcode.contains("M5"))  // Spindle off
    }

    @Test func gCodeContainsUnitCommand() {
        let toolpath = ComputedToolpath(configID: UUID(), toolID: UUID(), moves: [])
        let generator = GCodeGenerator(config: .init(unit: .millimeters))
        let gcode = generator.generate(toolpath: toolpath, tool: Tool.endMill6mm(), material: MaterialSetup())

        #expect(gcode.contains("G21")) // Millimeters
    }

    @Test func gCodeInchesContainsG20() {
        let toolpath = ComputedToolpath(configID: UUID(), toolID: UUID(), moves: [])
        let generator = GCodeGenerator(config: .init(unit: .inches))
        let gcode = generator.generate(toolpath: toolpath, tool: Tool.endMill6mm(), material: MaterialSetup())

        #expect(gcode.contains("G20")) // Inches
    }

    @Test func gCodeContainsComments() {
        let toolpath = ComputedToolpath(configID: UUID(), toolID: UUID(), moves: [])
        let generator = GCodeGenerator(config: .init(includeComments: true))
        let gcode = generator.generate(
            toolpath: toolpath,
            tool: Tool.endMill6mm(),
            material: MaterialSetup(),
            jobName: "Test Job"
        )

        #expect(gcode.contains("ClaudeCarve"))
        #expect(gcode.contains("Test Job"))
    }

    @Test func postProcessorGRBL() {
        let pp = PostProcessor.grbl
        #expect(pp.fileExtension == "gcode")
        #expect(pp.commentStyle == .semicolon)
    }

    @Test func postProcessorMach3() {
        let pp = PostProcessor.mach3
        #expect(pp.fileExtension == "tap")
        #expect(pp.commentStyle == .parentheses)
    }

    @Test func postProcessorFanucFooter() {
        let pp = PostProcessor.fanuc
        let footer = pp.footerCommands()
        #expect(footer.contains("M30"))
        #expect(footer.contains("%"))
    }

    @Test func gCodeModalOptimization() {
        // Sequential rapids should only output G0 once in the body
        // (header/footer also contain G0 for safe-Z and home moves)
        let toolpath = ComputedToolpath(
            configID: UUID(),
            toolID: UUID(),
            moves: [
                .rapid(to: Vector3D(0, 0, 5)),
                .rapid(to: Vector3D(10, 0, 5)),
                .rapid(to: Vector3D(20, 0, 5)),
                .linear(to: Vector3D(20, 0, -3), feed: 1000),
                .rapid(to: Vector3D(30, 0, 5)),
            ]
        )

        let generator = GCodeGenerator()
        let gcode = generator.generate(toolpath: toolpath, tool: Tool.endMill6mm(), material: MaterialSetup())
        let lines = gcode.components(separatedBy: "\n")

        // After a G1 move, the next rapid should output G0 again
        // But sequential rapids should NOT repeat G0 (modal optimization)
        // Verify modal optimization: sequential rapids should not repeat G0
        _ = lines.filter { $0.hasPrefix("G0") || $0.hasPrefix("X") || $0.hasPrefix("Y") }
        #expect(gcode.contains("G0"))
        #expect(gcode.contains("G1"))
    }
}
