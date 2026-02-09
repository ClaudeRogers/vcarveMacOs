import Foundation
import Testing
@testable import ClaudeCarveCore

@Suite("Tool Tests")
struct ToolTests {
    @Test func vBitDepthCalculation() {
        let vBit90 = Tool.vBit90()
        // 90° V-bit: half angle = 45°, tan(45°) = 1.0
        // depth = width / (2 * tan(45°)) = width / 2
        let depth = vBit90.vBitDepthForWidth(10.0)
        #expect(abs(depth - 5.0) < 0.01)
    }

    @Test func vBitWidthCalculation() {
        let vBit90 = Tool.vBit90()
        // width = 2 * depth * tan(45°) = 2 * depth
        let width = vBit90.vBitWidthAtDepth(5.0)
        #expect(abs(width - 10.0) < 0.01)
    }

    @Test func vBit60DepthCalculation() {
        let vBit60 = Tool.vBit60()
        // 60° V-bit: half angle = 30°, tan(30°) ≈ 0.5774
        // depth = width / (2 * 0.5774)
        let depth = vBit60.vBitDepthForWidth(10.0)
        let expected = 10.0 / (2.0 * tan(30.0 * .pi / 180.0))
        #expect(abs(depth - expected) < 0.01)
    }

    @Test func stepoverDistance() {
        let tool = Tool(diameter: 6.0, stepover: 0.4)
        #expect(abs(tool.stepoverDistance - 2.4) < 0.001)
    }

    @Test func toolRadius() {
        let tool = Tool(diameter: 6.0)
        #expect(abs(tool.radius - 3.0) < 0.001)
    }

    @Test func endMillPreset() {
        let tool = Tool.endMill6mm()
        #expect(tool.type == .endMill)
        #expect(abs(tool.diameter - 6.0) < 0.001)
    }

    @Test func ballNosePreset() {
        let tool = Tool.ballNose6mm()
        #expect(tool.type == .ballNose)
        #expect(abs(tool.cornerRadius - 3.0) < 0.001)
    }
}
