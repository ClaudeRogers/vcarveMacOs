import Foundation
import ClaudeCarveCore

// MARK: - Plugin/Gadget System

/// A plugin (gadget) that extends ClaudeCarve functionality.
public struct Plugin: Identifiable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var version: String
    public var author: String
    public var description: String
    public var category: PluginCategory
    public var isEnabled: Bool
    public var scriptPath: String?       // Path to script file
    public var parameters: [PluginParameter]

    public init(
        id: UUID = UUID(),
        name: String,
        version: String = "1.0",
        author: String = "",
        description: String = "",
        category: PluginCategory = .utility,
        isEnabled: Bool = true,
        scriptPath: String? = nil,
        parameters: [PluginParameter] = []
    ) {
        self.id = id
        self.name = name
        self.version = version
        self.author = author
        self.description = description
        self.category = category
        self.isEnabled = isEnabled
        self.scriptPath = scriptPath
        self.parameters = parameters
    }
}

public enum PluginCategory: String, Codable, Sendable, CaseIterable {
    case toolpath       // Toolpath generation plugins
    case design         // Design/drawing plugins
    case utility        // General utility plugins
    case postProcessor  // Post-processor plugins
    case import_export  // Import/export plugins
}

/// A parameter that can be configured for a plugin.
public struct PluginParameter: Identifiable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var label: String
    public var type: ParameterType
    public var defaultValue: String
    public var currentValue: String
    public var minValue: Double?
    public var maxValue: Double?
    public var options: [String]?       // For dropdown type

    public init(
        id: UUID = UUID(),
        name: String,
        label: String = "",
        type: ParameterType = .text,
        defaultValue: String = "",
        currentValue: String = "",
        minValue: Double? = nil,
        maxValue: Double? = nil,
        options: [String]? = nil
    ) {
        self.id = id
        self.name = name
        self.label = label.isEmpty ? name : label
        self.type = type
        self.defaultValue = defaultValue
        self.currentValue = currentValue.isEmpty ? defaultValue : currentValue
        self.minValue = minValue
        self.maxValue = maxValue
        self.options = options
    }
}

public enum ParameterType: String, Codable, Sendable {
    case text
    case number
    case integer
    case boolean
    case dropdown
    case filePath
}

/// Result of running a plugin.
public struct PluginResult: Sendable {
    public let success: Bool
    public let message: String
    public let outputPaths: [VectorPath]?
    public let outputGCode: String?

    public init(success: Bool, message: String, outputPaths: [VectorPath]? = nil, outputGCode: String? = nil) {
        self.success = success
        self.message = message
        self.outputPaths = outputPaths
        self.outputGCode = outputGCode
    }
}

/// The plugin manager handles loading, managing, and executing plugins.
public struct PluginManager {

    /// Default plugin directory.
    public static var pluginDirectory: URL {
        let appSupport = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        return appSupport.appendingPathComponent("ClaudeCarve/Plugins")
    }

    /// List all installed plugins.
    public static func installedPlugins() -> [Plugin] {
        let dir = pluginDirectory
        guard FileManager.default.fileExists(atPath: dir.path) else { return [] }

        var plugins: [Plugin] = []
        let decoder = JSONDecoder()

        if let contents = try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: nil) {
            for url in contents where url.pathExtension == "ccplugin" {
                if let data = try? Data(contentsOf: url),
                   let plugin = try? decoder.decode(Plugin.self, from: data) {
                    plugins.append(plugin)
                }
            }
        }

        return plugins
    }

    /// Save a plugin to the plugin directory.
    public static func savePlugin(_ plugin: Plugin) throws {
        let dir = pluginDirectory
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)

        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        let data = try encoder.encode(plugin)
        let url = dir.appendingPathComponent("\(plugin.name).ccplugin")
        try data.write(to: url)
    }

    /// Remove a plugin.
    public static func removePlugin(_ plugin: Plugin) throws {
        let url = pluginDirectory.appendingPathComponent("\(plugin.name).ccplugin")
        try FileManager.default.removeItem(at: url)
    }

    /// Built-in example plugins.
    public static func builtInPlugins() -> [Plugin] {
        [
            Plugin(
                name: "Box Joint Generator",
                version: "1.0",
                author: "ClaudeCarve",
                description: "Generate box/finger joint profiles for box construction.",
                category: .toolpath,
                parameters: [
                    PluginParameter(name: "fingerWidth", label: "Finger Width (mm)", type: .number, defaultValue: "10", minValue: 2, maxValue: 50),
                    PluginParameter(name: "materialThickness", label: "Material Thickness (mm)", type: .number, defaultValue: "12", minValue: 1, maxValue: 50),
                    PluginParameter(name: "boardLength", label: "Board Length (mm)", type: .number, defaultValue: "200", minValue: 10, maxValue: 2000),
                ]
            ),
            Plugin(
                name: "Gear Generator",
                version: "1.0",
                author: "ClaudeCarve",
                description: "Generate involute spur gear profiles.",
                category: .design,
                parameters: [
                    PluginParameter(name: "teeth", label: "Number of Teeth", type: .integer, defaultValue: "20", minValue: 6, maxValue: 200),
                    PluginParameter(name: "module", label: "Module", type: .number, defaultValue: "2.0", minValue: 0.5, maxValue: 20),
                    PluginParameter(name: "pressureAngle", label: "Pressure Angle (deg)", type: .number, defaultValue: "20", minValue: 14.5, maxValue: 25),
                ]
            ),
            Plugin(
                name: "Celtic Knot Generator",
                version: "1.0",
                author: "ClaudeCarve",
                description: "Generate Celtic knot patterns for decorative carving.",
                category: .design,
                parameters: [
                    PluginParameter(name: "gridSize", label: "Grid Size", type: .integer, defaultValue: "4", minValue: 2, maxValue: 10),
                    PluginParameter(name: "width", label: "Pattern Width (mm)", type: .number, defaultValue: "100", minValue: 20, maxValue: 500),
                    PluginParameter(name: "strandWidth", label: "Strand Width (mm)", type: .number, defaultValue: "5", minValue: 1, maxValue: 20),
                ]
            ),
            Plugin(
                name: "Keyhole Slot",
                version: "1.0",
                author: "ClaudeCarve",
                description: "Generate keyhole slot toolpaths for wall-mounting.",
                category: .toolpath,
                parameters: [
                    PluginParameter(name: "screwDiameter", label: "Screw Head Diameter (mm)", type: .number, defaultValue: "8", minValue: 3, maxValue: 20),
                    PluginParameter(name: "shankDiameter", label: "Screw Shank Diameter (mm)", type: .number, defaultValue: "4", minValue: 2, maxValue: 10),
                    PluginParameter(name: "slotLength", label: "Slot Length (mm)", type: .number, defaultValue: "20", minValue: 5, maxValue: 50),
                    PluginParameter(name: "depth", label: "Depth (mm)", type: .number, defaultValue: "5", minValue: 2, maxValue: 15),
                ]
            ),
        ]
    }
}

// MARK: - Custom Post-Processor Editor

/// A custom post-processor definition that users can edit.
public struct CustomPostProcessor: Identifiable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var description: String
    public var fileExtension: String     // e.g., ".nc", ".gcode", ".tap"

    // G-code formatting
    public var lineNumbers: Bool
    public var lineNumberIncrement: Int
    public var useSpaces: Bool           // Spaces between commands

    // Header/footer
    public var headerLines: [String]
    public var footerLines: [String]

    // Commands
    public var rapidCommand: String      // G0 or G00
    public var linearCommand: String     // G1 or G01
    public var cwArcCommand: String      // G2 or G02
    public var ccwArcCommand: String     // G3 or G03
    public var spindleOnCW: String       // M3 or M03
    public var spindleOnCCW: String      // M4 or M04
    public var spindleOff: String        // M5 or M05
    public var coolantOn: String         // M8 or M08
    public var coolantOff: String        // M9 or M09
    public var programEnd: String        // M2, M30, M02

    // Coordinate format
    public var decimalPlaces: Int
    public var absoluteMode: String      // G90
    public var incrementalMode: String   // G91
    public var unitsMetric: String       // G21
    public var unitsImperial: String     // G20

    // Modal behavior
    public var modalGCodes: Bool         // Only output G-code when it changes
    public var modalFeedRates: Bool      // Only output feed when it changes

    // Tool change
    public var toolChangeCommand: String // e.g., "M6 T{tool}"
    public var toolChangeIncludesPause: Bool

    // Variables available in templates:
    // {tool} - tool number
    // {speed} - spindle speed
    // {feed} - feed rate
    // {x} {y} {z} - coordinates
    // {date} - current date
    // {filename} - file name

    public init(
        id: UUID = UUID(),
        name: String = "Custom Post",
        description: String = "",
        fileExtension: String = ".nc",
        lineNumbers: Bool = false,
        lineNumberIncrement: Int = 10,
        useSpaces: Bool = true,
        headerLines: [String] = ["(Header)", "G90 G21", "G17"],
        footerLines: [String] = ["M5", "G0 Z25", "M30"],
        rapidCommand: String = "G0",
        linearCommand: String = "G1",
        cwArcCommand: String = "G2",
        ccwArcCommand: String = "G3",
        spindleOnCW: String = "M3",
        spindleOnCCW: String = "M4",
        spindleOff: String = "M5",
        coolantOn: String = "M8",
        coolantOff: String = "M9",
        programEnd: String = "M30",
        decimalPlaces: Int = 3,
        absoluteMode: String = "G90",
        incrementalMode: String = "G91",
        unitsMetric: String = "G21",
        unitsImperial: String = "G20",
        modalGCodes: Bool = true,
        modalFeedRates: Bool = true,
        toolChangeCommand: String = "M6 T{tool}",
        toolChangeIncludesPause: Bool = true
    ) {
        self.id = id
        self.name = name
        self.description = description
        self.fileExtension = fileExtension
        self.lineNumbers = lineNumbers
        self.lineNumberIncrement = lineNumberIncrement
        self.useSpaces = useSpaces
        self.headerLines = headerLines
        self.footerLines = footerLines
        self.rapidCommand = rapidCommand
        self.linearCommand = linearCommand
        self.cwArcCommand = cwArcCommand
        self.ccwArcCommand = ccwArcCommand
        self.spindleOnCW = spindleOnCW
        self.spindleOnCCW = spindleOnCCW
        self.spindleOff = spindleOff
        self.coolantOn = coolantOn
        self.coolantOff = coolantOff
        self.programEnd = programEnd
        self.decimalPlaces = decimalPlaces
        self.absoluteMode = absoluteMode
        self.incrementalMode = incrementalMode
        self.unitsMetric = unitsMetric
        self.unitsImperial = unitsImperial
        self.modalGCodes = modalGCodes
        self.modalFeedRates = modalFeedRates
        self.toolChangeCommand = toolChangeCommand
        self.toolChangeIncludesPause = toolChangeIncludesPause
    }
}

/// Manager for custom post-processors.
public struct PostProcessorEditor {

    /// Default directory for custom post-processors.
    public static var postProcessorDirectory: URL {
        let appSupport = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        return appSupport.appendingPathComponent("ClaudeCarve/PostProcessors")
    }

    /// Load all custom post-processors.
    public static func loadCustomPostProcessors() -> [CustomPostProcessor] {
        let dir = postProcessorDirectory
        guard FileManager.default.fileExists(atPath: dir.path) else { return [] }

        var processors: [CustomPostProcessor] = []
        let decoder = JSONDecoder()

        if let contents = try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: nil) {
            for url in contents where url.pathExtension == "ccpost" {
                if let data = try? Data(contentsOf: url),
                   let pp = try? decoder.decode(CustomPostProcessor.self, from: data) {
                    processors.append(pp)
                }
            }
        }

        return processors
    }

    /// Save a custom post-processor.
    public static func save(_ postProcessor: CustomPostProcessor) throws {
        let dir = postProcessorDirectory
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)

        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        let data = try encoder.encode(postProcessor)
        let url = dir.appendingPathComponent("\(postProcessor.name).ccpost")
        try data.write(to: url)
    }

    /// Delete a custom post-processor.
    public static func delete(_ postProcessor: CustomPostProcessor) throws {
        let url = postProcessorDirectory.appendingPathComponent("\(postProcessor.name).ccpost")
        try FileManager.default.removeItem(at: url)
    }

    /// Generate G-code using a custom post-processor.
    public static func generateGCode(
        toolpath: ComputedToolpath,
        postProcessor: CustomPostProcessor,
        toolNumber: Int = 1,
        spindleSpeed: Double = 18000
    ) -> String {
        var lines: [String] = []
        var lineNum = postProcessor.lineNumberIncrement
        let sep = postProcessor.useSpaces ? " " : ""
        let fmt = "%.\(postProcessor.decimalPlaces)f"

        func addLine(_ content: String) {
            if postProcessor.lineNumbers {
                lines.append("N\(lineNum) \(content)")
                lineNum += postProcessor.lineNumberIncrement
            } else {
                lines.append(content)
            }
        }

        // Header
        for headerLine in postProcessor.headerLines {
            let expanded = expandVariables(headerLine, toolNumber: toolNumber, spindleSpeed: spindleSpeed)
            addLine(expanded)
        }

        // Tool change
        let toolChangeExpanded = expandVariables(postProcessor.toolChangeCommand, toolNumber: toolNumber, spindleSpeed: spindleSpeed)
        addLine(toolChangeExpanded)

        if postProcessor.toolChangeIncludesPause {
            addLine("M0 (Tool change pause)")
        }

        // Spindle on
        addLine("\(postProcessor.spindleOnCW)\(sep)S\(Int(spindleSpeed))")

        // Toolpath moves
        var lastMotion = ""
        var lastFeed = 0.0

        for move in toolpath.moves {
            let xStr = String(format: fmt, move.position.x)
            let yStr = String(format: fmt, move.position.y)
            let zStr = String(format: fmt, move.position.z)

            let motionCmd: String
            switch move.type {
            case .rapid:
                motionCmd = postProcessor.rapidCommand
            case .linear:
                motionCmd = postProcessor.linearCommand
            case .cwArc:
                motionCmd = postProcessor.cwArcCommand
            case .ccwArc:
                motionCmd = postProcessor.ccwArcCommand
            }

            var parts: [String] = []

            // Modal motion code
            if !postProcessor.modalGCodes || motionCmd != lastMotion {
                parts.append(motionCmd)
                lastMotion = motionCmd
            }

            parts.append("X\(xStr)")
            parts.append("Y\(yStr)")
            parts.append("Z\(zStr)")

            // Feed rate
            if move.type != .rapid {
                let feed = move.feedRate ?? 1000
                if !postProcessor.modalFeedRates || feed != lastFeed {
                    parts.append("F\(Int(feed))")
                    lastFeed = feed
                }
            }

            addLine(parts.joined(separator: sep))
        }

        // Footer
        for footerLine in postProcessor.footerLines {
            let expanded = expandVariables(footerLine, toolNumber: toolNumber, spindleSpeed: spindleSpeed)
            addLine(expanded)
        }

        return lines.joined(separator: "\n")
    }

    /// Expand template variables in a string.
    private static func expandVariables(
        _ template: String,
        toolNumber: Int,
        spindleSpeed: Double
    ) -> String {
        var result = template
        result = result.replacingOccurrences(of: "{tool}", with: "\(toolNumber)")
        result = result.replacingOccurrences(of: "{speed}", with: "\(Int(spindleSpeed))")
        let formatter = DateFormatter()
        formatter.dateStyle = .short
        result = result.replacingOccurrences(of: "{date}", with: formatter.string(from: Date()))
        return result
    }

    /// Built-in post-processor presets for common CNC controllers.
    public static func builtInPresets() -> [CustomPostProcessor] {
        [
            CustomPostProcessor(
                name: "GRBL",
                description: "GRBL CNC controller",
                fileExtension: ".nc",
                headerLines: ["(GRBL)", "G90 G21", "G17"],
                footerLines: ["M5", "G0 Z10", "G0 X0 Y0", "M30"]
            ),
            CustomPostProcessor(
                name: "Mach3/Mach4",
                description: "Mach3 and Mach4 CNC controller",
                fileExtension: ".tap",
                lineNumbers: true,
                headerLines: ["(Mach3/4)", "G90 G21 G17", "G0 Z25.0"],
                footerLines: ["M5", "G0 Z25.0", "G0 X0 Y0", "M30"]
            ),
            CustomPostProcessor(
                name: "LinuxCNC",
                description: "LinuxCNC (EMC2) controller",
                fileExtension: ".ngc",
                headerLines: ["(LinuxCNC)", "G90 G21 G17 G40 G49 G64 P0.01"],
                footerLines: ["M5", "G53 G0 Z0", "M2"]
            ),
            CustomPostProcessor(
                name: "Shopbot",
                description: "ShopBot CNC router",
                fileExtension: ".sbp",
                headerLines: ["'ShopBot File", "SA", "MS,{feed},{feed}"],
                footerLines: ["SO,1,0", "END"],
                rapidCommand: "J3",
                linearCommand: "M3",
                modalGCodes: false
            ),
        ]
    }
}
