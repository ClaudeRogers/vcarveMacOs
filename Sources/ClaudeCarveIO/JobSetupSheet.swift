import Foundation
import AppKit
import ClaudeCarveCore

/// Configuration for a job setup sheet PDF.
public struct SetupSheetConfig: Codable, Sendable {
    public var includeToolList: Bool
    public var includeToolpathSummary: Bool
    public var includeMaterialInfo: Bool
    public var includeEstimatedTime: Bool
    public var includeDesignPreview: Bool
    public var includeNotes: Bool
    public var notes: String
    public var companyName: String
    public var projectName: String
    public var operatorName: String
    public var pageSize: PageSize

    public init(
        includeToolList: Bool = true,
        includeToolpathSummary: Bool = true,
        includeMaterialInfo: Bool = true,
        includeEstimatedTime: Bool = true,
        includeDesignPreview: Bool = true,
        includeNotes: Bool = true,
        notes: String = "",
        companyName: String = "",
        projectName: String = "",
        operatorName: String = "",
        pageSize: PageSize = .letter
    ) {
        self.includeToolList = includeToolList
        self.includeToolpathSummary = includeToolpathSummary
        self.includeMaterialInfo = includeMaterialInfo
        self.includeEstimatedTime = includeEstimatedTime
        self.includeDesignPreview = includeDesignPreview
        self.includeNotes = includeNotes
        self.notes = notes
        self.companyName = companyName
        self.projectName = projectName
        self.operatorName = operatorName
        self.pageSize = pageSize
    }
}

public enum PageSize: String, Codable, Sendable {
    case letter  // 8.5 x 11 inches
    case a4      // 210 x 297 mm
    case legal   // 8.5 x 14 inches

    public var widthPoints: Double {
        switch self {
        case .letter: return 612
        case .a4: return 595.28
        case .legal: return 612
        }
    }

    public var heightPoints: Double {
        switch self {
        case .letter: return 792
        case .a4: return 841.89
        case .legal: return 1008
        }
    }
}

/// Generates a PDF job setup sheet for CNC operators.
public struct JobSetupSheet {

    /// Generate a PDF setup sheet from a document.
    public static func generatePDF(
        document: ClaudeCarveDocument,
        config: SetupSheetConfig
    ) -> Data? {
        let pageWidth = config.pageSize.widthPoints
        let pageHeight = config.pageSize.heightPoints
        let margin = 50.0

        let mutableData = NSMutableData()
        guard let consumer = CGDataConsumer(data: mutableData),
              let context = CGContext(consumer: consumer, mediaBox: nil, nil) else {
            return nil
        }

        var pageRect = CGRect(x: 0, y: 0, width: pageWidth, height: pageHeight)
        var cursorY = pageHeight - margin

        // Start page
        context.beginPage(mediaBox: &pageRect)

        // Title
        cursorY = drawTitle(
            context: context,
            config: config,
            pageWidth: pageWidth,
            margin: margin,
            y: cursorY
        )

        cursorY -= 20

        // Material Info
        if config.includeMaterialInfo {
            cursorY = drawMaterialInfo(
                context: context,
                material: document.materialSetup,
                margin: margin,
                width: pageWidth - 2 * margin,
                y: cursorY
            )
            cursorY -= 15
        }

        // Tool List
        if config.includeToolList {
            cursorY = drawToolList(
                context: context,
                tools: document.toolDatabase,
                margin: margin,
                width: pageWidth - 2 * margin,
                y: cursorY
            )
            cursorY -= 15
        }

        // Toolpath Summary
        if config.includeToolpathSummary {
            cursorY = drawToolpathSummary(
                context: context,
                toolpaths: document.toolpathConfigs,
                margin: margin,
                width: pageWidth - 2 * margin,
                y: cursorY
            )
            cursorY -= 15
        }

        // Notes
        if config.includeNotes && !config.notes.isEmpty {
            cursorY = drawNotes(
                context: context,
                notes: config.notes,
                margin: margin,
                width: pageWidth - 2 * margin,
                y: cursorY
            )
        }

        // Footer
        drawFooter(
            context: context,
            pageWidth: pageWidth,
            margin: margin
        )

        context.endPage()
        context.closePDF()

        return mutableData as Data
    }

    // MARK: - Drawing Helpers

    private static func drawTitle(
        context: CGContext,
        config: SetupSheetConfig,
        pageWidth: Double,
        margin: Double,
        y: Double
    ) -> Double {
        var cursorY = y

        // Company name
        if !config.companyName.isEmpty {
            drawText(context: context, text: config.companyName, x: margin, y: cursorY, fontSize: 18, bold: true)
            cursorY -= 24
        }

        // Title
        drawText(context: context, text: "Job Setup Sheet", x: margin, y: cursorY, fontSize: 16, bold: true)

        // Date - right aligned
        let dateStr = formattedDate()
        drawText(context: context, text: "Date: \(dateStr)", x: pageWidth - margin - 150, y: cursorY, fontSize: 10, bold: false)
        cursorY -= 20

        // Project name
        if !config.projectName.isEmpty {
            drawText(context: context, text: "Project: \(config.projectName)", x: margin, y: cursorY, fontSize: 12, bold: false)
            cursorY -= 16
        }

        // Operator
        if !config.operatorName.isEmpty {
            drawText(context: context, text: "Operator: \(config.operatorName)", x: margin, y: cursorY, fontSize: 12, bold: false)
            cursorY -= 16
        }

        // Separator line
        context.setStrokeColor(CGColor(gray: 0.3, alpha: 1.0))
        context.setLineWidth(1)
        context.move(to: CGPoint(x: margin, y: cursorY))
        context.addLine(to: CGPoint(x: pageWidth - margin, y: cursorY))
        context.strokePath()
        cursorY -= 5

        return cursorY
    }

    private static func drawMaterialInfo(
        context: CGContext,
        material: MaterialSetup,
        margin: Double,
        width: Double,
        y: Double
    ) -> Double {
        var cursorY = y

        drawText(context: context, text: "Material Setup", x: margin, y: cursorY, fontSize: 14, bold: true)
        cursorY -= 18

        let unitStr = material.unit.abbreviation
        let rows = [
            ("Material Size:", String(format: "%.1f x %.1f %@", material.width, material.height, unitStr)),
            ("Thickness:", String(format: "%.2f %@", material.thickness, unitStr)),
            ("Z-Zero:", material.zZeroPosition.rawValue),
            ("XY Datum:", material.xyDatumPosition.rawValue),
        ]

        for (label, value) in rows {
            drawText(context: context, text: label, x: margin + 10, y: cursorY, fontSize: 10, bold: false)
            drawText(context: context, text: value, x: margin + 130, y: cursorY, fontSize: 10, bold: false)
            cursorY -= 14
        }

        return cursorY
    }

    private static func drawToolList(
        context: CGContext,
        tools: [Tool],
        margin: Double,
        width: Double,
        y: Double
    ) -> Double {
        var cursorY = y

        drawText(context: context, text: "Tools", x: margin, y: cursorY, fontSize: 14, bold: true)
        cursorY -= 18

        // Table header
        let colX: [Double] = [margin + 10, margin + 40, margin + 180, margin + 280, margin + 380]
        let headers = ["#", "Tool Name", "Diameter", "Type", "RPM"]

        for (i, header) in headers.enumerated() {
            drawText(context: context, text: header, x: colX[i], y: cursorY, fontSize: 9, bold: true)
        }
        cursorY -= 13

        // Separator
        context.setStrokeColor(CGColor(gray: 0.6, alpha: 1.0))
        context.setLineWidth(0.5)
        context.move(to: CGPoint(x: margin + 10, y: cursorY + 4))
        context.addLine(to: CGPoint(x: margin + width - 10, y: cursorY + 4))
        context.strokePath()

        for (index, tool) in tools.enumerated() {
            let typeStr: String
            switch tool.type {
            case .endMill: typeStr = "End Mill"
            case .ballNose: typeStr = "Ball Nose"
            case .vBit: typeStr = "V-Bit"
            case .bullNose: typeStr = "Bull Nose"
            case .drill: typeStr = "Drill"
            case .engraving: typeStr = "Engraving"
            case .taperedBallNose: typeStr = "Tapered Ball Nose"
            case .custom: typeStr = "Custom"
            case .chamfer: typeStr = "Chamfer"
            case .threadMill: typeStr = "Thread Mill"
            }

            let values = [
                "\(index + 1)",
                tool.name,
                String(format: "%.2f mm", tool.diameter),
                typeStr,
                String(format: "%.0f", tool.spindleSpeed)
            ]

            for (i, val) in values.enumerated() {
                drawText(context: context, text: val, x: colX[i], y: cursorY, fontSize: 9, bold: false)
            }
            cursorY -= 13
        }

        return cursorY
    }

    private static func drawToolpathSummary(
        context: CGContext,
        toolpaths: [ToolpathConfig],
        margin: Double,
        width: Double,
        y: Double
    ) -> Double {
        var cursorY = y

        drawText(context: context, text: "Toolpath Operations", x: margin, y: cursorY, fontSize: 14, bold: true)
        cursorY -= 18

        let colX: [Double] = [margin + 10, margin + 40, margin + 200, margin + 300, margin + 400]
        let headers = ["#", "Name", "Type", "Depth", "Vectors"]

        for (i, header) in headers.enumerated() {
            drawText(context: context, text: header, x: colX[i], y: cursorY, fontSize: 9, bold: true)
        }
        cursorY -= 13

        context.setStrokeColor(CGColor(gray: 0.6, alpha: 1.0))
        context.setLineWidth(0.5)
        context.move(to: CGPoint(x: margin + 10, y: cursorY + 4))
        context.addLine(to: CGPoint(x: margin + width - 10, y: cursorY + 4))
        context.strokePath()

        for (index, tp) in toolpaths.enumerated() {
            let values = [
                "\(index + 1)",
                tp.name,
                tp.type.rawValue.capitalized,
                String(format: "%.2f mm", tp.cutDepth),
                "\(tp.selectedPathIDs.count)"
            ]

            for (i, val) in values.enumerated() {
                drawText(context: context, text: val, x: colX[i], y: cursorY, fontSize: 9, bold: false)
            }
            cursorY -= 13
        }

        if toolpaths.isEmpty {
            drawText(context: context, text: "(No toolpaths configured)", x: margin + 10, y: cursorY, fontSize: 9, bold: false)
            cursorY -= 13
        }

        return cursorY
    }

    private static func drawNotes(
        context: CGContext,
        notes: String,
        margin: Double,
        width: Double,
        y: Double
    ) -> Double {
        var cursorY = y

        drawText(context: context, text: "Notes", x: margin, y: cursorY, fontSize: 14, bold: true)
        cursorY -= 18

        // Draw notes box
        let boxHeight = 80.0
        context.setStrokeColor(CGColor(gray: 0.5, alpha: 1.0))
        context.setLineWidth(0.5)
        context.stroke(CGRect(x: margin + 10, y: cursorY - boxHeight, width: width - 20, height: boxHeight))

        // Draw notes text
        let lines = notes.components(separatedBy: .newlines)
        var textY = cursorY - 5
        for line in lines.prefix(6) {
            drawText(context: context, text: line, x: margin + 15, y: textY, fontSize: 9, bold: false)
            textY -= 12
        }

        cursorY -= boxHeight + 5

        return cursorY
    }

    private static func drawFooter(
        context: CGContext,
        pageWidth: Double,
        margin: Double
    ) {
        let y = 30.0
        context.setStrokeColor(CGColor(gray: 0.3, alpha: 1.0))
        context.setLineWidth(0.5)
        context.move(to: CGPoint(x: margin, y: y + 10))
        context.addLine(to: CGPoint(x: pageWidth - margin, y: y + 10))
        context.strokePath()

        drawText(context: context, text: "Generated by ClaudeCarve for macOS", x: margin, y: y, fontSize: 8, bold: false)
        drawText(context: context, text: formattedDate(), x: pageWidth - margin - 80, y: y, fontSize: 8, bold: false)
    }

    // MARK: - Primitive Drawing

    private static func drawText(
        context: CGContext,
        text: String,
        x: Double,
        y: Double,
        fontSize: Double,
        bold: Bool
    ) {
        let fontName = bold ? "Helvetica-Bold" : "Helvetica"
        guard let font = CGFont(fontName as CFString) else { return }

        context.saveGState()
        context.setFont(font)
        context.setFontSize(fontSize)
        context.setFillColor(CGColor(gray: 0, alpha: 1.0))
        context.textPosition = CGPoint(x: x, y: y)

        // Use Core Text for proper text rendering
        let attributes: [NSAttributedString.Key: Any] = [
            .font: NSFont(name: fontName, size: fontSize) ?? NSFont.systemFont(ofSize: fontSize),
            .foregroundColor: NSColor.black
        ]
        let attrString = NSAttributedString(string: text, attributes: attributes)
        let line = CTLineCreateWithAttributedString(attrString)
        CTLineDraw(line, context)

        context.restoreGState()
    }

    private static func formattedDate() -> String {
        let formatter = DateFormatter()
        formatter.dateStyle = .medium
        formatter.timeStyle = .short
        return formatter.string(from: Date())
    }
}
