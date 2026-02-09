import Foundation
import VCarveCore

/// G-code comment formatting style.
public enum CommentStyle: String, Codable, Sendable {
    case parentheses  // ( comment )
    case semicolon    // ; comment

    public func format(_ text: String) -> String {
        switch self {
        case .parentheses: return "(\(text))"
        case .semicolon: return "; \(text)"
        }
    }
}

/// Post-processor definitions for various CNC controllers.
/// Each post-processor defines the G-code dialect for a specific machine controller.
public enum PostProcessor: String, Codable, Sendable, CaseIterable {
    case grbl           // GRBL (Shapeoko, X-Carve, many hobby CNCs)
    case grblMM         // GRBL in millimeters
    case grblInch       // GRBL in inches
    case mach3          // Mach3/Mach4
    case linuxCNC       // LinuxCNC/EMC2
    case uccnc          // UCCNC
    case carbideMotion  // Carbide Motion (Shapeoko/Nomad)
    case generic        // Generic G-code
    case fanuc          // Fanuc-style
    case haas           // Haas CNC
    case shopbot        // ShopBot

    public var displayName: String {
        switch self {
        case .grbl: return "GRBL"
        case .grblMM: return "GRBL (mm)"
        case .grblInch: return "GRBL (inch)"
        case .mach3: return "Mach3 / Mach4"
        case .linuxCNC: return "LinuxCNC"
        case .uccnc: return "UCCNC"
        case .carbideMotion: return "Carbide Motion"
        case .generic: return "Generic G-Code"
        case .fanuc: return "Fanuc"
        case .haas: return "Haas"
        case .shopbot: return "ShopBot"
        }
    }

    public var fileExtension: String {
        switch self {
        case .grbl, .grblMM, .grblInch: return "gcode"
        case .mach3: return "tap"
        case .linuxCNC: return "ngc"
        case .uccnc: return "nc"
        case .carbideMotion: return "gcode"
        case .generic: return "gcode"
        case .fanuc: return "nc"
        case .haas: return "nc"
        case .shopbot: return "sbp"
        }
    }

    public var commentStyle: CommentStyle {
        switch self {
        case .grbl, .grblMM, .grblInch: return .semicolon
        case .shopbot: return .parentheses
        default: return .parentheses
        }
    }

    /// Header G-codes to initialize the machine.
    public func headerCommands(unit: MeasurementUnit) -> [String] {
        var cmds: [String] = []

        switch self {
        case .grbl, .grblMM, .grblInch:
            cmds.append(unit == .millimeters ? "G21" : "G20") // Units
            cmds.append("G90") // Absolute positioning
            cmds.append("G17") // XY plane selection

        case .mach3:
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")
            cmds.append("G17")
            cmds.append("G40") // Cancel cutter compensation
            cmds.append("G49") // Cancel tool length compensation

        case .linuxCNC:
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")
            cmds.append("G17")
            cmds.append("G40")
            cmds.append("G49")
            cmds.append("G54") // Work coordinate system

        case .uccnc:
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")
            cmds.append("G17")

        case .carbideMotion:
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")

        case .generic:
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")
            cmds.append("G17")

        case .fanuc, .haas:
            cmds.append("%")
            cmds.append("O0001")
            cmds.append(unit == .millimeters ? "G21" : "G20")
            cmds.append("G90")
            cmds.append("G17")
            cmds.append("G40 G49 G80")

        case .shopbot:
            cmds.append("SA") // Set absolute mode
            if unit == .millimeters {
                cmds.append("IF %(25)=1 THEN GOTO METRIC")
            }
        }

        return cmds
    }

    /// Footer G-codes to end the program.
    public func footerCommands() -> [String] {
        switch self {
        case .fanuc, .haas:
            return ["M30", "%"]
        case .shopbot:
            return ["END"]
        default:
            return ["M2"] // Program end
        }
    }
}
