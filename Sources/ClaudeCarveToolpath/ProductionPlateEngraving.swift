import Foundation
import ClaudeCarveCore

/// A data field that maps a column name to a position and style for plate engraving.
public struct PlateField: Codable, Sendable {
    public let columnName: String
    public var position: Vector2D      // Position on the plate template
    public var fontName: String
    public var fontSize: Double
    public var alignment: PlateTextAlignment
    public var maxWidth: Double        // Max width before text wraps/shrinks

    public init(
        columnName: String,
        position: Vector2D,
        fontName: String = "Helvetica",
        fontSize: Double = 5.0,
        alignment: PlateTextAlignment = .left,
        maxWidth: Double = 50.0
    ) {
        self.columnName = columnName
        self.position = position
        self.fontName = fontName
        self.fontSize = fontSize
        self.alignment = alignment
        self.maxWidth = maxWidth
    }
}

public enum PlateTextAlignment: String, Codable, Sendable {
    case left, center, right
}

/// Configuration for production plate engraving from CSV data.
public struct PlateEngravingConfig: Codable, Sendable {
    public var fields: [PlateField]
    public var plateWidth: Double       // Width of each plate
    public var plateHeight: Double      // Height of each plate
    public var plateSpacingX: Double    // Horizontal spacing between plates
    public var plateSpacingY: Double    // Vertical spacing between plates
    public var columns: Int             // Plates per row
    public var borderPaths: [VectorPath] // Plate border/frame paths (optional)
    public var cutDepth: Double
    public var feedRate: Double
    public var plungeRate: Double
    public var safeZ: Double

    public init(
        fields: [PlateField] = [],
        plateWidth: Double = 80.0,
        plateHeight: Double = 30.0,
        plateSpacingX: Double = 5.0,
        plateSpacingY: Double = 5.0,
        columns: Int = 4,
        borderPaths: [VectorPath] = [],
        cutDepth: Double = 0.3,
        feedRate: Double = 1000,
        plungeRate: Double = 500,
        safeZ: Double = 5.0
    ) {
        self.fields = fields
        self.plateWidth = plateWidth
        self.plateHeight = plateHeight
        self.plateSpacingX = plateSpacingX
        self.plateSpacingY = plateSpacingY
        self.columns = columns
        self.borderPaths = borderPaths
        self.cutDepth = cutDepth
        self.feedRate = feedRate
        self.plungeRate = plungeRate
        self.safeZ = safeZ
    }
}

/// A single row of CSV data parsed into key-value pairs.
public struct PlateRecord: Sendable {
    public let values: [String: String]

    public init(values: [String: String]) {
        self.values = values
    }

    public func value(for columnName: String) -> String {
        values[columnName] ?? ""
    }
}

/// Production plate engraving — merge CSV data with a plate template.
public struct ProductionPlateEngraving {

    // MARK: - CSV Parsing

    /// Parse CSV data into records.
    public static func parseCSV(_ csvString: String) -> [PlateRecord] {
        let lines = csvString.components(separatedBy: .newlines)
            .map { $0.trimmingCharacters(in: .whitespaces) }
            .filter { !$0.isEmpty }

        guard lines.count >= 2 else { return [] }

        let headers = parseCSVLine(lines[0])
        var records: [PlateRecord] = []

        for i in 1..<lines.count {
            let values = parseCSVLine(lines[i])
            var dict: [String: String] = [:]
            for (j, header) in headers.enumerated() {
                if j < values.count {
                    dict[header] = values[j]
                }
            }
            records.append(PlateRecord(values: dict))
        }

        return records
    }

    /// Parse a single CSV line handling quoted fields.
    private static func parseCSVLine(_ line: String) -> [String] {
        var fields: [String] = []
        var current = ""
        var inQuotes = false

        for char in line {
            if char == "\"" {
                inQuotes.toggle()
            } else if char == "," && !inQuotes {
                fields.append(current.trimmingCharacters(in: .whitespaces))
                current = ""
            } else {
                current.append(char)
            }
        }

        fields.append(current.trimmingCharacters(in: .whitespaces))
        return fields
    }

    // MARK: - Toolpath Generation

    /// Generate engraving toolpath for all plates from CSV records.
    public static func generate(
        records: [PlateRecord],
        config: PlateEngravingConfig,
        tool: Tool
    ) -> ComputedToolpath {
        var allMoves: [ToolpathMove] = []

        // Initial safe position
        allMoves.append(.rapid(to: Vector3D(0, 0, config.safeZ)))

        for (index, record) in records.enumerated() {
            let row = index / config.columns
            let col = index % config.columns

            let originX = Double(col) * (config.plateWidth + config.plateSpacingX)
            let originY = Double(row) * (config.plateHeight + config.plateSpacingY)
            let plateOrigin = Vector2D(originX, originY)

            // Engrave border paths for this plate
            let borderMoves = engraveBorderPaths(
                config: config,
                plateOrigin: plateOrigin
            )
            allMoves.append(contentsOf: borderMoves)

            // Engrave each field
            for field in config.fields {
                let text = record.value(for: field.columnName)
                guard !text.isEmpty else { continue }

                let fieldMoves = engraveTextField(
                    text: text,
                    field: field,
                    plateOrigin: plateOrigin,
                    config: config
                )
                allMoves.append(contentsOf: fieldMoves)
            }
        }

        // Return to safe position
        allMoves.append(.rapid(to: Vector3D(0, 0, config.safeZ)))

        return ComputedToolpath(configID: UUID(), toolID: tool.id, moves: allMoves)
    }

    // MARK: - Border Engraving

    private static func engraveBorderPaths(
        config: PlateEngravingConfig,
        plateOrigin: Vector2D
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        for borderPath in config.borderPaths {
            let points = borderPath.flattenedPoints(tolerance: 0.05)
            guard points.count >= 2 else { continue }

            // Translate border points to plate position
            let translatedPoints = points.map { $0 + plateOrigin }

            // Rapid to start
            let start = translatedPoints[0]
            moves.append(.rapid(to: Vector3D(start, z: config.safeZ)))
            moves.append(.plunge(to: config.cutDepth, at: start, feed: config.plungeRate))

            // Cut border
            for i in 1..<translatedPoints.count {
                let p = translatedPoints[i]
                moves.append(.linear(to: Vector3D(p, z: config.cutDepth), feed: config.feedRate))
            }

            // Close if needed
            if borderPath.isClosed {
                moves.append(.linear(to: Vector3D(start, z: config.cutDepth), feed: config.feedRate))
            }

            // Retract
            let last = translatedPoints.last!
            moves.append(.rapid(to: Vector3D(last, z: config.safeZ)))
        }

        return moves
    }

    // MARK: - Text Field Engraving

    private static func engraveTextField(
        text: String,
        field: PlateField,
        plateOrigin: Vector2D,
        config: PlateEngravingConfig
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []

        // Generate simple single-stroke text paths
        let textPaths = generateSingleStrokeText(
            text: text,
            fontSize: field.fontSize,
            alignment: field.alignment,
            maxWidth: field.maxWidth
        )

        let fieldOrigin = plateOrigin + field.position

        for path in textPaths {
            let points = path.flattenedPoints(tolerance: 0.05)
            guard points.count >= 2 else { continue }

            // Translate to field position
            let translatedPoints = points.map { $0 + fieldOrigin }

            // Rapid to start
            let start = translatedPoints[0]
            moves.append(.rapid(to: Vector3D(start, z: config.safeZ)))
            moves.append(.plunge(to: config.cutDepth, at: start, feed: config.plungeRate))

            // Engrave
            for i in 1..<translatedPoints.count {
                let p = translatedPoints[i]
                moves.append(.linear(to: Vector3D(p, z: config.cutDepth), feed: config.feedRate))
            }

            // Retract
            let last = translatedPoints.last!
            moves.append(.rapid(to: Vector3D(last, z: config.safeZ)))
        }

        return moves
    }

    // MARK: - Single-Stroke Font

    /// Generate simple single-stroke (Hershey-style) text paths.
    /// This is a simplified vector font — each character is a set of line strokes.
    private static func generateSingleStrokeText(
        text: String,
        fontSize: Double,
        alignment: PlateTextAlignment,
        maxWidth: Double
    ) -> [VectorPath] {
        var paths: [VectorPath] = []
        let scale = fontSize / 10.0 // Normalized character height is ~10 units
        var cursorX = 0.0

        // Calculate total text width for alignment
        let charWidth = 7.0 * scale
        let totalWidth = min(Double(text.count) * charWidth, maxWidth)

        let offsetX: Double
        switch alignment {
        case .left: offsetX = 0
        case .center: offsetX = -totalWidth / 2
        case .right: offsetX = -totalWidth
        }

        // Scale to fit if text is wider than maxWidth
        let effectiveScale: Double
        let naturalWidth = Double(text.count) * charWidth
        if naturalWidth > maxWidth && naturalWidth > 0 {
            effectiveScale = scale * (maxWidth / naturalWidth)
        } else {
            effectiveScale = scale
        }

        let effectiveCharWidth = 7.0 * effectiveScale

        for char in text {
            let charPaths = singleStrokeCharacter(char, scale: effectiveScale)
            for charPath in charPaths {
                let translated = charPath.translated(by: Vector2D(cursorX + offsetX, 0))
                paths.append(translated)
            }
            cursorX += effectiveCharWidth
        }

        return paths
    }

    /// Generate single-stroke paths for a single character.
    /// Uses a simplified Hershey-inspired vector font.
    private static func singleStrokeCharacter(_ char: Character, scale: Double) -> [VectorPath] {
        let s = scale
        let strokes = characterStrokes(char)

        return strokes.map { stroke in
            guard stroke.count >= 2 else {
                return VectorPath(startPoint: stroke[0] * s)
            }
            var path = VectorPath(startPoint: stroke[0] * s)
            for i in 1..<stroke.count {
                path.segments.append(.lineTo(stroke[i] * s))
            }
            return path
        }
    }

    /// Stroke data for basic ASCII characters (simplified Hershey font).
    private static func characterStrokes(_ char: Character) -> [[Vector2D]] {
        switch char {
        case "A":
            return [
                [Vector2D(0, 0), Vector2D(3, 10), Vector2D(6, 0)],
                [Vector2D(1.5, 5), Vector2D(4.5, 5)]
            ]
        case "B":
            return [
                [Vector2D(0, 0), Vector2D(0, 10), Vector2D(4, 10), Vector2D(5, 9), Vector2D(5, 6), Vector2D(4, 5), Vector2D(0, 5)],
                [Vector2D(4, 5), Vector2D(5, 4), Vector2D(5, 1), Vector2D(4, 0), Vector2D(0, 0)]
            ]
        case "C":
            return [[Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0), Vector2D(0, 2), Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8)]]
        case "D":
            return [[Vector2D(0, 0), Vector2D(0, 10), Vector2D(4, 10), Vector2D(6, 8), Vector2D(6, 2), Vector2D(4, 0), Vector2D(0, 0)]]
        case "E":
            return [
                [Vector2D(6, 0), Vector2D(0, 0), Vector2D(0, 10), Vector2D(6, 10)],
                [Vector2D(0, 5), Vector2D(4, 5)]
            ]
        case "F":
            return [
                [Vector2D(0, 0), Vector2D(0, 10), Vector2D(6, 10)],
                [Vector2D(0, 5), Vector2D(4, 5)]
            ]
        case "G":
            return [[Vector2D(6, 8), Vector2D(5, 10), Vector2D(1, 10), Vector2D(0, 8), Vector2D(0, 2), Vector2D(1, 0), Vector2D(5, 0), Vector2D(6, 2), Vector2D(6, 5), Vector2D(3, 5)]]
        case "H":
            return [
                [Vector2D(0, 0), Vector2D(0, 10)],
                [Vector2D(6, 0), Vector2D(6, 10)],
                [Vector2D(0, 5), Vector2D(6, 5)]
            ]
        case "I":
            return [
                [Vector2D(1, 0), Vector2D(5, 0)],
                [Vector2D(3, 0), Vector2D(3, 10)],
                [Vector2D(1, 10), Vector2D(5, 10)]
            ]
        case "J":
            return [[Vector2D(6, 10), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0), Vector2D(0, 2)]]
        case "K":
            return [
                [Vector2D(0, 0), Vector2D(0, 10)],
                [Vector2D(6, 10), Vector2D(0, 5), Vector2D(6, 0)]
            ]
        case "L":
            return [[Vector2D(0, 10), Vector2D(0, 0), Vector2D(6, 0)]]
        case "M":
            return [[Vector2D(0, 0), Vector2D(0, 10), Vector2D(3, 5), Vector2D(6, 10), Vector2D(6, 0)]]
        case "N":
            return [[Vector2D(0, 0), Vector2D(0, 10), Vector2D(6, 0), Vector2D(6, 10)]]
        case "O":
            return [[Vector2D(1, 0), Vector2D(0, 2), Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0)]]
        case "P":
            return [[Vector2D(0, 0), Vector2D(0, 10), Vector2D(5, 10), Vector2D(6, 9), Vector2D(6, 6), Vector2D(5, 5), Vector2D(0, 5)]]
        case "Q":
            return [
                [Vector2D(1, 0), Vector2D(0, 2), Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0)],
                [Vector2D(4, 2), Vector2D(7, -1)]
            ]
        case "R":
            return [
                [Vector2D(0, 0), Vector2D(0, 10), Vector2D(5, 10), Vector2D(6, 9), Vector2D(6, 6), Vector2D(5, 5), Vector2D(0, 5)],
                [Vector2D(3, 5), Vector2D(6, 0)]
            ]
        case "S":
            return [[Vector2D(6, 8), Vector2D(5, 10), Vector2D(1, 10), Vector2D(0, 9), Vector2D(0, 6), Vector2D(1, 5), Vector2D(5, 5), Vector2D(6, 4), Vector2D(6, 1), Vector2D(5, 0), Vector2D(1, 0), Vector2D(0, 2)]]
        case "T":
            return [
                [Vector2D(0, 10), Vector2D(6, 10)],
                [Vector2D(3, 0), Vector2D(3, 10)]
            ]
        case "U":
            return [[Vector2D(0, 10), Vector2D(0, 2), Vector2D(1, 0), Vector2D(5, 0), Vector2D(6, 2), Vector2D(6, 10)]]
        case "V":
            return [[Vector2D(0, 10), Vector2D(3, 0), Vector2D(6, 10)]]
        case "W":
            return [[Vector2D(0, 10), Vector2D(1.5, 0), Vector2D(3, 6), Vector2D(4.5, 0), Vector2D(6, 10)]]
        case "X":
            return [
                [Vector2D(0, 0), Vector2D(6, 10)],
                [Vector2D(0, 10), Vector2D(6, 0)]
            ]
        case "Y":
            return [
                [Vector2D(0, 10), Vector2D(3, 5), Vector2D(6, 10)],
                [Vector2D(3, 5), Vector2D(3, 0)]
            ]
        case "Z":
            return [[Vector2D(0, 10), Vector2D(6, 10), Vector2D(0, 0), Vector2D(6, 0)]]
        case "0":
            return [
                [Vector2D(1, 0), Vector2D(0, 2), Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0)],
                [Vector2D(1, 1), Vector2D(5, 9)]
            ]
        case "1":
            return [
                [Vector2D(1, 8), Vector2D(3, 10), Vector2D(3, 0)],
                [Vector2D(1, 0), Vector2D(5, 0)]
            ]
        case "2":
            return [[Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 6), Vector2D(0, 0), Vector2D(6, 0)]]
        case "3":
            return [
                [Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 6), Vector2D(5, 5), Vector2D(3, 5)],
                [Vector2D(5, 5), Vector2D(6, 4), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0), Vector2D(0, 2)]
            ]
        case "4":
            return [
                [Vector2D(5, 0), Vector2D(5, 10), Vector2D(0, 3), Vector2D(6, 3)]
            ]
        case "5":
            return [[Vector2D(6, 10), Vector2D(0, 10), Vector2D(0, 5), Vector2D(5, 5), Vector2D(6, 4), Vector2D(6, 1), Vector2D(5, 0), Vector2D(1, 0), Vector2D(0, 1)]]
        case "6":
            return [[Vector2D(5, 10), Vector2D(1, 10), Vector2D(0, 8), Vector2D(0, 2), Vector2D(1, 0), Vector2D(5, 0), Vector2D(6, 2), Vector2D(6, 4), Vector2D(5, 5), Vector2D(0, 5)]]
        case "7":
            return [[Vector2D(0, 10), Vector2D(6, 10), Vector2D(3, 0)]]
        case "8":
            return [
                [Vector2D(1, 5), Vector2D(0, 6), Vector2D(0, 9), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 9), Vector2D(6, 6), Vector2D(5, 5), Vector2D(1, 5)],
                [Vector2D(1, 5), Vector2D(0, 4), Vector2D(0, 1), Vector2D(1, 0), Vector2D(5, 0), Vector2D(6, 1), Vector2D(6, 4), Vector2D(5, 5)]
            ]
        case "9":
            return [[Vector2D(6, 5), Vector2D(1, 5), Vector2D(0, 6), Vector2D(0, 8), Vector2D(1, 10), Vector2D(5, 10), Vector2D(6, 8), Vector2D(6, 2), Vector2D(5, 0), Vector2D(1, 0)]]
        case " ":
            return []
        case "-":
            return [[Vector2D(1, 5), Vector2D(5, 5)]]
        case ".":
            return [[Vector2D(3, 0), Vector2D(3, 0.5)]]
        case ",":
            return [[Vector2D(3, 0), Vector2D(2, -1)]]
        case "/":
            return [[Vector2D(0, 0), Vector2D(6, 10)]]
        case ":":
            return [
                [Vector2D(3, 3), Vector2D(3, 3.5)],
                [Vector2D(3, 7), Vector2D(3, 7.5)]
            ]
        case "#":
            return [
                [Vector2D(2, 0), Vector2D(2, 10)],
                [Vector2D(4, 0), Vector2D(4, 10)],
                [Vector2D(0, 3), Vector2D(6, 3)],
                [Vector2D(0, 7), Vector2D(6, 7)]
            ]
        default:
            // For lowercase, use uppercase strokes at slightly smaller scale
            if char.isLetter, let upper = char.uppercased().first {
                return characterStrokes(upper)
            }
            // Unknown character: draw a small box
            return [[Vector2D(0, 0), Vector2D(0, 8), Vector2D(5, 8), Vector2D(5, 0), Vector2D(0, 0)]]
        }
    }

    // MARK: - Column Discovery

    /// Get available column names from CSV data.
    public static func columnNames(from csvString: String) -> [String] {
        let lines = csvString.components(separatedBy: .newlines)
            .map { $0.trimmingCharacters(in: .whitespaces) }
            .filter { !$0.isEmpty }
        guard let firstLine = lines.first else { return [] }
        return parseCSVLine(firstLine)
    }
}
