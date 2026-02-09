import Foundation

/// The geometric shape/type of a CNC cutting tool.
public enum ToolType: String, Codable, Sendable, CaseIterable {
    case endMill        // Flat-bottom cylindrical cutter
    case ballNose       // Hemispherical end
    case vBit           // V-shaped engraving bit
    case bullNose       // End mill with rounded corners
    case engraving      // Fine-point engraving tool
    case drill          // Twist drill for holes
    case taperedBallNose // Tapered shaft with ball nose tip
    case chamfer        // Chamfer/countersink bit
    case threadMill     // Thread cutting tool
    case custom         // User-defined profile
}

/// Units for tool measurements and feed rates.
public enum MeasurementUnit: String, Codable, Sendable {
    case inches
    case millimeters

    public var abbreviation: String {
        switch self {
        case .inches: return "in"
        case .millimeters: return "mm"
        }
    }

    public var perMinuteAbbreviation: String {
        switch self {
        case .inches: return "in/min"
        case .millimeters: return "mm/min"
        }
    }

    /// Conversion factor to millimeters.
    public var toMM: Double {
        switch self {
        case .inches: return 25.4
        case .millimeters: return 1.0
        }
    }
}

/// A CNC cutting tool with all parameters needed for toolpath generation.
public struct Tool: Identifiable, Hashable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var type: ToolType
    public var unit: MeasurementUnit

    // Geometry
    public var diameter: Double          // Tool diameter
    public var fluteLength: Double       // Cutting edge length
    public var shankDiameter: Double     // Shank diameter above cutting edge
    public var overallLength: Double     // Total tool length
    public var tipAngle: Double          // V-bit angle in degrees (for vBit, engraving, chamfer)
    public var cornerRadius: Double      // Corner radius (for bullNose)
    public var numberOfFlutes: Int       // Number of cutting flutes

    // Cutting parameters
    public var spindleSpeed: Double      // RPM
    public var feedRate: Double          // XY feed rate (units/min)
    public var plungeRate: Double        // Z plunge rate (units/min)
    public var depthPerPass: Double      // Maximum depth per cutting pass
    public var stepover: Double          // Stepover as fraction of diameter (0.0–1.0)

    public init(
        id: UUID = UUID(),
        name: String = "End Mill",
        type: ToolType = .endMill,
        unit: MeasurementUnit = .millimeters,
        diameter: Double = 6.0,
        fluteLength: Double = 25.0,
        shankDiameter: Double = 6.0,
        overallLength: Double = 50.0,
        tipAngle: Double = 0,
        cornerRadius: Double = 0,
        numberOfFlutes: Int = 2,
        spindleSpeed: Double = 18000,
        feedRate: Double = 2000,
        plungeRate: Double = 500,
        depthPerPass: Double = 3.0,
        stepover: Double = 0.4
    ) {
        self.id = id
        self.name = name
        self.type = type
        self.unit = unit
        self.diameter = diameter
        self.fluteLength = fluteLength
        self.shankDiameter = shankDiameter
        self.overallLength = overallLength
        self.tipAngle = tipAngle
        self.cornerRadius = cornerRadius
        self.numberOfFlutes = numberOfFlutes
        self.spindleSpeed = spindleSpeed
        self.feedRate = feedRate
        self.plungeRate = plungeRate
        self.depthPerPass = depthPerPass
        self.stepover = stepover
    }

    /// Radius of the tool.
    public var radius: Double { diameter / 2.0 }

    /// Stepover distance (not fraction).
    public var stepoverDistance: Double { diameter * stepover }

    /// Half angle in radians for V-bits.
    public var halfAngleRadians: Double { (tipAngle / 2.0) * .pi / 180.0 }

    /// For a V-bit, calculate the depth needed to achieve a given width of cut.
    public func vBitDepthForWidth(_ width: Double) -> Double {
        guard tipAngle > 0 else { return 0 }
        return width / (2.0 * tan(halfAngleRadians))
    }

    /// For a V-bit, calculate the width of cut at a given depth.
    public func vBitWidthAtDepth(_ depth: Double) -> Double {
        guard tipAngle > 0 else { return diameter }
        return 2.0 * depth * tan(halfAngleRadians)
    }

    // MARK: - Common tool presets

    public static func vBit60(unit: MeasurementUnit = .millimeters) -> Tool {
        Tool(
            name: "60° V-Bit",
            type: .vBit,
            unit: unit,
            diameter: unit == .millimeters ? 12.0 : 0.5,
            fluteLength: unit == .millimeters ? 15.0 : 0.6,
            shankDiameter: unit == .millimeters ? 6.35 : 0.25,
            overallLength: unit == .millimeters ? 50.0 : 2.0,
            tipAngle: 60,
            spindleSpeed: 18000,
            feedRate: unit == .millimeters ? 1500 : 60,
            plungeRate: unit == .millimeters ? 500 : 20,
            depthPerPass: unit == .millimeters ? 3.0 : 0.125
        )
    }

    public static func vBit90(unit: MeasurementUnit = .millimeters) -> Tool {
        var tool = vBit60(unit: unit)
        tool.name = "90° V-Bit"
        tool.tipAngle = 90
        return tool
    }

    public static func endMill6mm() -> Tool {
        Tool(name: "6mm End Mill", type: .endMill, diameter: 6.0)
    }

    public static func endMill3mm() -> Tool {
        Tool(name: "3mm End Mill", type: .endMill, diameter: 3.0)
    }

    public static func ballNose6mm() -> Tool {
        Tool(name: "6mm Ball Nose", type: .ballNose, diameter: 6.0, cornerRadius: 3.0)
    }
}
