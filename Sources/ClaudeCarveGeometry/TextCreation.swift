import Foundation
import CoreText
import ClaudeCarveCore

// MARK: - TextAlignment

/// Text alignment for multi-line text layout.
public enum TextAlignment: String, Codable, Sendable {
    case left
    case center
    case right
}

// MARK: - TextConfig

/// Configuration for text-to-vector conversion.
public struct TextConfig: Codable, Sendable {
    public var text: String
    public var fontName: String
    public var fontSize: Double
    public var alignment: TextAlignment
    public var letterSpacing: Double
    public var lineSpacing: Double
    public var bold: Bool
    public var italic: Bool

    public init(
        text: String,
        fontName: String = "Helvetica",
        fontSize: Double = 12.0,
        alignment: TextAlignment = .left,
        letterSpacing: Double = 0.0,
        lineSpacing: Double = 1.2,
        bold: Bool = false,
        italic: Bool = false
    ) {
        self.text = text
        self.fontName = fontName
        self.fontSize = fontSize
        self.alignment = alignment
        self.letterSpacing = letterSpacing
        self.lineSpacing = lineSpacing
        self.bold = bold
        self.italic = italic
    }
}

// MARK: - TextCreator

/// Converts text strings to vector path outlines using CoreText glyph extraction.
public struct TextCreator: Sendable {

    // MARK: - Public API

    /// Convert text to vector paths using CoreText glyph outlines.
    ///
    /// Each contour of each glyph becomes a separate `VectorPath`. Multi-line text
    /// is split on `"\n"` and offset vertically. The coordinate system places the
    /// baseline of the first line at Y = 0, with positive Y going upward.
    public static func createText(config: TextConfig) -> [VectorPath] {
        let font = resolveFont(config: config)
        let lines = config.text.split(separator: "\n", omittingEmptySubsequences: false)
            .map { String($0) }

        // Compute per-line advance widths for alignment.
        let lineWidths = lines.map { lineWidth($0, font: font, letterSpacing: config.letterSpacing) }
        let maxWidth = lineWidths.max() ?? 0.0

        let lineHeight = config.fontSize * config.lineSpacing
        var allPaths: [VectorPath] = []

        for (lineIndex, line) in lines.enumerated() {
            let yOffset = -Double(lineIndex) * lineHeight

            // Horizontal offset for alignment.
            let lineW = lineWidths[lineIndex]
            let xOffset: Double
            switch config.alignment {
            case .left:
                xOffset = 0
            case .center:
                xOffset = (maxWidth - lineW) / 2.0
            case .right:
                xOffset = maxWidth - lineW
            }

            let glyphPaths = extractGlyphPaths(
                line: line,
                font: font,
                letterSpacing: config.letterSpacing,
                xOffset: xOffset,
                yOffset: yOffset,
                italic: config.italic
            )
            allPaths.append(contentsOf: glyphPaths)
        }

        return allPaths
    }

    /// Create text following a curve path.
    ///
    /// Each glyph is placed along the curve at its cumulative advance distance,
    /// rotated to match the curve tangent at that position. The `offset` parameter
    /// shifts glyphs perpendicular to the curve (positive = above, negative = below).
    public static func createTextOnCurve(
        config: TextConfig,
        curve: VectorPath,
        offset: Double = 0
    ) -> [VectorPath] {
        let font = resolveFont(config: config)
        let flatPoints = curve.flattenedPoints(tolerance: 0.1)
        guard flatPoints.count >= 2 else { return [] }

        // Pre-compute cumulative arc lengths along the flattened curve.
        let (cumulativeLengths, totalLength) = computeCumulativeLengths(points: flatPoints)

        // Extract individual glyph path sets with their advances.
        let glyphData = extractPerGlyphData(
            text: config.text,
            font: font,
            letterSpacing: config.letterSpacing,
            italic: config.italic
        )

        var allPaths: [VectorPath] = []
        var cursorDistance = 0.0

        for (glyphPaths, advance) in glyphData {
            // Place glyph center at the midpoint of its advance span.
            let glyphCenter = cursorDistance + advance / 2.0

            // Clamp to curve length.
            guard glyphCenter <= totalLength else { break }

            // Find position and tangent on curve at glyphCenter distance.
            let (position, tangent) = pointAndTangentOnCurve(
                at: glyphCenter,
                points: flatPoints,
                cumulativeLengths: cumulativeLengths
            )

            let angle = tangent.angle

            // Apply perpendicular offset.
            let normal = tangent.perpendicular.normalized
            let placementPos = position + normal * offset

            // Transform each glyph contour: rotate by tangent angle, translate to position.
            for path in glyphPaths {
                let transformed = transformPath(
                    path,
                    rotation: angle,
                    translation: placementPos
                )
                allPaths.append(transformed)
            }

            cursorDistance += advance
        }

        return allPaths
    }

    /// List available font family names on the system.
    public static func availableFonts() -> [String] {
        let collection = CTFontCollectionCreateFromAvailableFonts(nil)
        guard let descriptors = CTFontCollectionCreateMatchingFontDescriptors(collection) as? [CTFontDescriptor] else {
            return []
        }
        var names: [String] = []
        for descriptor in descriptors {
            if let nameRef = CTFontDescriptorCopyAttribute(descriptor, kCTFontFamilyNameAttribute) {
                names.append(nameRef as! String)
            }
        }
        // Deduplicate and sort.
        return Array(Set(names)).sorted()
    }

    // MARK: - Font Resolution

    /// Resolve a CTFont from the config, applying bold/italic traits.
    private static func resolveFont(config: TextConfig) -> CTFont {
        let baseFontName = config.fontName as CFString
        let baseFont = CTFontCreateWithName(baseFontName, config.fontSize, nil)

        var traits: CTFontSymbolicTraits = []
        if config.bold {
            traits.insert(.boldTrait)
        }
        if config.italic {
            traits.insert(.italicTrait)
        }

        if !traits.isEmpty {
            if let traitedFont = CTFontCreateCopyWithSymbolicTraits(
                baseFont, config.fontSize, nil, traits, traits
            ) {
                return traitedFont
            }
        }

        return baseFont
    }

    // MARK: - Glyph Path Extraction

    /// Compute the total advance width of a line of text.
    private static func lineWidth(_ line: String, font: CTFont, letterSpacing: Double) -> Double {
        let scalars = Array(line.unicodeScalars)
        guard !scalars.isEmpty else { return 0 }

        var totalWidth = 0.0
        var glyphs = [CGGlyph](repeating: 0, count: scalars.count)
        var characters = scalars.map { UniChar($0.value) }
        CTFontGetGlyphsForCharacters(font, &characters, &glyphs, scalars.count)

        var advances = [CGSize](repeating: .zero, count: scalars.count)
        CTFontGetAdvancesForGlyphs(font, .horizontal, glyphs, &advances, scalars.count)

        for i in 0..<scalars.count {
            totalWidth += advances[i].width + letterSpacing
        }
        // Remove trailing letter spacing.
        if !scalars.isEmpty {
            totalWidth -= letterSpacing
        }
        return totalWidth
    }

    /// Extract vector paths for all glyphs in a line, positioned at the given offset.
    private static func extractGlyphPaths(
        line: String,
        font: CTFont,
        letterSpacing: Double,
        xOffset: Double,
        yOffset: Double,
        italic: Bool
    ) -> [VectorPath] {
        let scalars = Array(line.unicodeScalars)
        guard !scalars.isEmpty else { return [] }

        var glyphs = [CGGlyph](repeating: 0, count: scalars.count)
        var characters = scalars.map { UniChar($0.value) }
        CTFontGetGlyphsForCharacters(font, &characters, &glyphs, scalars.count)

        var advances = [CGSize](repeating: .zero, count: scalars.count)
        CTFontGetAdvancesForGlyphs(font, .horizontal, glyphs, &advances, scalars.count)

        var allPaths: [VectorPath] = []
        var cursor = xOffset

        // If the font did not resolve an italic variant, apply a synthetic shear.
        let needsSyntheticItalic = italic && !fontHasItalicTrait(font)
        let shearFactor = needsSyntheticItalic ? 0.2 : 0.0

        for i in 0..<scalars.count {
            let glyph = glyphs[i]
            guard glyph != 0 else {
                cursor += advances[i].width + letterSpacing
                continue
            }

            if let cgPath = CTFontCreatePathForGlyph(font, glyph, nil) {
                let contours = extractContours(from: cgPath)
                for contour in contours {
                    let translated = translateContour(
                        contour,
                        dx: cursor,
                        dy: yOffset,
                        shearX: shearFactor
                    )
                    allPaths.append(translated)
                }
            }

            cursor += advances[i].width + letterSpacing
        }

        return allPaths
    }

    /// Check whether a resolved CTFont already has the italic trait.
    private static func fontHasItalicTrait(_ font: CTFont) -> Bool {
        let traits = CTFontGetSymbolicTraits(font)
        return traits.contains(.italicTrait)
    }

    /// Extract per-glyph data: each glyph's contour paths (centered at origin) and advance width.
    private static func extractPerGlyphData(
        text: String,
        font: CTFont,
        letterSpacing: Double,
        italic: Bool
    ) -> [([VectorPath], Double)] {
        let scalars = Array(text.unicodeScalars)
        guard !scalars.isEmpty else { return [] }

        var glyphs = [CGGlyph](repeating: 0, count: scalars.count)
        var characters = scalars.map { UniChar($0.value) }
        CTFontGetGlyphsForCharacters(font, &characters, &glyphs, scalars.count)

        var advances = [CGSize](repeating: .zero, count: scalars.count)
        CTFontGetAdvancesForGlyphs(font, .horizontal, glyphs, &advances, scalars.count)

        let needsSyntheticItalic = italic && !fontHasItalicTrait(font)
        let shearFactor = needsSyntheticItalic ? 0.2 : 0.0

        var result: [([VectorPath], Double)] = []

        for i in 0..<scalars.count {
            let glyph = glyphs[i]
            let advance = advances[i].width + letterSpacing
            var paths: [VectorPath] = []

            if glyph != 0, let cgPath = CTFontCreatePathForGlyph(font, glyph, nil) {
                let contours = extractContours(from: cgPath)
                for contour in contours {
                    // Apply shear but keep glyphs at origin; translation happens during placement.
                    let shearedContour = translateContour(contour, dx: 0, dy: 0, shearX: shearFactor)
                    paths.append(shearedContour)
                }
            }

            result.append((paths, advance))
        }

        return result
    }

    // MARK: - CGPath to VectorPath Conversion

    /// A raw contour extracted from a CGPath: a start point and a sequence of segments.
    private struct RawContour {
        var startPoint: Vector2D
        var segments: [PathSegment]
        var isClosed: Bool
    }

    /// Walk a CGPath and extract contours as VectorPath objects.
    private static func extractContours(from cgPath: CGPath) -> [RawContour] {
        // Accumulate elements using CGPath.applyWithBlock.
        nonisolated(unsafe) var contours: [RawContour] = []
        nonisolated(unsafe) var currentStart: Vector2D = .zero
        nonisolated(unsafe) var currentSegments: [PathSegment] = []
        nonisolated(unsafe) var hasOpenContour = false

        cgPath.applyWithBlock { elementPtr in
            let element = elementPtr.pointee
            switch element.type {
            case .moveToPoint:
                // Close previous contour if open.
                if hasOpenContour {
                    contours.append(RawContour(
                        startPoint: currentStart,
                        segments: currentSegments,
                        isClosed: false
                    ))
                }
                let p = element.points[0]
                currentStart = Vector2D(p.x, p.y)
                currentSegments = []
                hasOpenContour = true

            case .addLineToPoint:
                let p = element.points[0]
                currentSegments.append(.lineTo(Vector2D(p.x, p.y)))

            case .addQuadCurveToPoint:
                let cp = element.points[0]
                let end = element.points[1]
                currentSegments.append(.quadTo(
                    control: Vector2D(cp.x, cp.y),
                    end: Vector2D(end.x, end.y)
                ))

            case .addCurveToPoint:
                let cp1 = element.points[0]
                let cp2 = element.points[1]
                let end = element.points[2]
                currentSegments.append(.cubicTo(
                    control1: Vector2D(cp1.x, cp1.y),
                    control2: Vector2D(cp2.x, cp2.y),
                    end: Vector2D(end.x, end.y)
                ))

            case .closeSubpath:
                contours.append(RawContour(
                    startPoint: currentStart,
                    segments: currentSegments,
                    isClosed: true
                ))
                currentSegments = []
                hasOpenContour = false

            @unknown default:
                break
            }
        }

        // Flush any remaining open contour.
        if hasOpenContour && !currentSegments.isEmpty {
            contours.append(RawContour(
                startPoint: currentStart,
                segments: currentSegments,
                isClosed: false
            ))
        }

        return contours
    }

    /// Translate a raw contour by (dx, dy) with optional horizontal shear for synthetic italic.
    /// Shear formula: x' = x + shearX * y
    private static func translateContour(
        _ contour: RawContour,
        dx: Double,
        dy: Double,
        shearX: Double
    ) -> VectorPath {
        func transform(_ v: Vector2D) -> Vector2D {
            Vector2D(v.x + shearX * v.y + dx, v.y + dy)
        }

        let start = transform(contour.startPoint)
        let segments = contour.segments.map { segment -> PathSegment in
            switch segment {
            case .lineTo(let p):
                return .lineTo(transform(p))
            case .quadTo(let control, let end):
                return .quadTo(control: transform(control), end: transform(end))
            case .cubicTo(let c1, let c2, let end):
                return .cubicTo(control1: transform(c1), control2: transform(c2), end: transform(end))
            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                return .arcTo(
                    center: transform(center),
                    radius: radius,
                    startAngle: startAngle,
                    endAngle: endAngle,
                    clockwise: clockwise
                )
            }
        }

        return VectorPath(startPoint: start, segments: segments, isClosed: contour.isClosed)
    }

    // MARK: - Curve Utilities

    /// Compute cumulative arc lengths for a polyline.
    private static func computeCumulativeLengths(
        points: [Vector2D]
    ) -> ([Double], Double) {
        var lengths = [0.0]
        var total = 0.0
        for i in 1..<points.count {
            let d = points[i - 1].distance(to: points[i])
            total += d
            lengths.append(total)
        }
        return (lengths, total)
    }

    /// Find the position and tangent on a polyline at a given arc-length distance.
    private static func pointAndTangentOnCurve(
        at distance: Double,
        points: [Vector2D],
        cumulativeLengths: [Double]
    ) -> (Vector2D, Vector2D) {
        // Binary search for the segment containing `distance`.
        var lo = 0
        var hi = cumulativeLengths.count - 1

        while lo < hi - 1 {
            let mid = (lo + hi) / 2
            if cumulativeLengths[mid] <= distance {
                lo = mid
            } else {
                hi = mid
            }
        }

        let segStart = points[lo]
        let segEnd = points[hi]
        let segLength = cumulativeLengths[hi] - cumulativeLengths[lo]

        let t: Double
        if segLength > 1e-12 {
            t = (distance - cumulativeLengths[lo]) / segLength
        } else {
            t = 0
        }

        let position = segStart.lerp(to: segEnd, t: t)
        let tangent = (segEnd - segStart).normalized

        return (position, tangent)
    }

    /// Rotate and translate all points in a VectorPath.
    private static func transformPath(
        _ path: VectorPath,
        rotation: Double,
        translation: Vector2D
    ) -> VectorPath {
        func xform(_ v: Vector2D) -> Vector2D {
            v.rotated(by: rotation) + translation
        }

        let newStart = xform(path.startPoint)
        let newSegments = path.segments.map { segment -> PathSegment in
            switch segment {
            case .lineTo(let p):
                return .lineTo(xform(p))
            case .quadTo(let control, let end):
                return .quadTo(control: xform(control), end: xform(end))
            case .cubicTo(let c1, let c2, let end):
                return .cubicTo(control1: xform(c1), control2: xform(c2), end: xform(end))
            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                return .arcTo(
                    center: xform(center),
                    radius: radius,
                    startAngle: startAngle + rotation,
                    endAngle: endAngle + rotation,
                    clockwise: clockwise
                )
            }
        }

        return VectorPath(startPoint: newStart, segments: newSegments, isClosed: path.isClosed)
    }
}
