import Foundation
import ClaudeCarveCore

/// Imports SVG (Scalable Vector Graphics) files into ClaudeCarve vector paths.
public struct SVGImporter {

    public init() {}

    /// Import an SVG file from the given URL.
    public func importFile(at url: URL) throws -> [VectorPath] {
        let data = try Data(contentsOf: url)
        return try importData(data)
    }

    /// Import SVG from raw data.
    public func importData(_ data: Data) throws -> [VectorPath] {
        guard let xmlString = String(data: data, encoding: .utf8) else {
            throw SVGImportError.invalidData
        }

        let parser = SVGPathParser()
        return parser.parse(svgContent: xmlString)
    }
}

public enum SVGImportError: Error, LocalizedError {
    case invalidData
    case parseError(String)
    case unsupportedElement(String)

    public var errorDescription: String? {
        switch self {
        case .invalidData: return "Invalid SVG data"
        case .parseError(let msg): return "SVG parse error: \(msg)"
        case .unsupportedElement(let elem): return "Unsupported SVG element: \(elem)"
        }
    }
}

/// Parses SVG path data ("d" attribute) into VectorPath objects.
struct SVGPathParser {

    func parse(svgContent: String) -> [VectorPath] {
        var paths: [VectorPath] = []

        // Extract path elements using regex
        let pathPattern = #"<path[^>]*d="([^"]*)"[^>]*/?>|<path[^>]*d='([^']*)'[^>]*/?>"#
        guard let regex = try? NSRegularExpression(pattern: pathPattern, options: [.dotMatchesLineSeparators]) else {
            return paths
        }

        let nsString = svgContent as NSString
        let matches = regex.matches(in: svgContent, range: NSRange(location: 0, length: nsString.length))

        for match in matches {
            var pathData: String?
            if match.range(at: 1).location != NSNotFound {
                pathData = nsString.substring(with: match.range(at: 1))
            } else if match.range(at: 2).location != NSNotFound {
                pathData = nsString.substring(with: match.range(at: 2))
            }

            if let d = pathData, let path = parseSVGPathData(d) {
                paths.append(path)
            }
        }

        // Also parse basic shapes
        paths.append(contentsOf: parseRectangles(svgContent))
        paths.append(contentsOf: parseCircles(svgContent))
        paths.append(contentsOf: parseEllipses(svgContent))
        paths.append(contentsOf: parseLines(svgContent))
        paths.append(contentsOf: parsePolylines(svgContent))
        paths.append(contentsOf: parsePolygons(svgContent))

        return paths
    }

    /// Parse SVG path data (the "d" attribute).
    func parseSVGPathData(_ d: String) -> VectorPath? {
        let tokens = tokenizeSVGPath(d)
        guard !tokens.isEmpty else { return nil }

        var currentPoint = Vector2D.zero
        var startPoint = Vector2D.zero
        var segments: [PathSegment] = []
        var pathStarted = false
        var i = 0

        while i < tokens.count {
            let token = tokens[i]
            i += 1

            switch token {
            case "M": // Move to (absolute)
                guard i + 1 < tokens.count else { break }
                let x = Double(tokens[i]) ?? 0
                let y = Double(tokens[i + 1]) ?? 0
                i += 2
                currentPoint = Vector2D(x, y)
                startPoint = currentPoint
                pathStarted = true

                // Subsequent coordinate pairs are treated as L
                while i + 1 < tokens.count, let x2 = Double(tokens[i]), let y2 = Double(tokens[i + 1]) {
                    let to = Vector2D(x2, y2)
                    segments.append(.lineTo(to))
                    currentPoint = to
                    i += 2
                }

            case "m": // Move to (relative)
                guard i + 1 < tokens.count else { break }
                let dx = Double(tokens[i]) ?? 0
                let dy = Double(tokens[i + 1]) ?? 0
                i += 2
                currentPoint = currentPoint + Vector2D(dx, dy)
                startPoint = currentPoint
                pathStarted = true

                while i + 1 < tokens.count, let dx2 = Double(tokens[i]), let dy2 = Double(tokens[i + 1]) {
                    let to = currentPoint + Vector2D(dx2, dy2)
                    segments.append(.lineTo(to))
                    currentPoint = to
                    i += 2
                }

            case "L": // Line to (absolute)
                while i + 1 < tokens.count, let x = Double(tokens[i]), let y = Double(tokens[i + 1]) {
                    let to = Vector2D(x, y)
                    segments.append(.lineTo(to))
                    currentPoint = to
                    i += 2
                }

            case "l": // Line to (relative)
                while i + 1 < tokens.count, let dx = Double(tokens[i]), let dy = Double(tokens[i + 1]) {
                    let to = currentPoint + Vector2D(dx, dy)
                    segments.append(.lineTo(to))
                    currentPoint = to
                    i += 2
                }

            case "H": // Horizontal line (absolute)
                guard let x = Double(tokens[i]) else { break }
                i += 1
                let to = Vector2D(x, currentPoint.y)
                segments.append(.lineTo(to))
                currentPoint = to

            case "h": // Horizontal line (relative)
                guard let dx = Double(tokens[i]) else { break }
                i += 1
                let to = currentPoint + Vector2D(dx, 0)
                segments.append(.lineTo(to))
                currentPoint = to

            case "V": // Vertical line (absolute)
                guard let y = Double(tokens[i]) else { break }
                i += 1
                let to = Vector2D(currentPoint.x, y)
                segments.append(.lineTo(to))
                currentPoint = to

            case "v": // Vertical line (relative)
                guard let dy = Double(tokens[i]) else { break }
                i += 1
                let to = currentPoint + Vector2D(0, dy)
                segments.append(.lineTo(to))
                currentPoint = to

            case "C": // Cubic Bézier (absolute)
                while i + 5 < tokens.count,
                      let x1 = Double(tokens[i]), let y1 = Double(tokens[i + 1]),
                      let x2 = Double(tokens[i + 2]), let y2 = Double(tokens[i + 3]),
                      let x = Double(tokens[i + 4]), let y = Double(tokens[i + 5]) {
                    let c1 = Vector2D(x1, y1)
                    let c2 = Vector2D(x2, y2)
                    let end = Vector2D(x, y)
                    segments.append(.cubicTo(control1: c1, control2: c2, end: end))
                    currentPoint = end
                    i += 6
                }

            case "c": // Cubic Bézier (relative)
                while i + 5 < tokens.count,
                      let dx1 = Double(tokens[i]), let dy1 = Double(tokens[i + 1]),
                      let dx2 = Double(tokens[i + 2]), let dy2 = Double(tokens[i + 3]),
                      let dx = Double(tokens[i + 4]), let dy = Double(tokens[i + 5]) {
                    let c1 = currentPoint + Vector2D(dx1, dy1)
                    let c2 = currentPoint + Vector2D(dx2, dy2)
                    let end = currentPoint + Vector2D(dx, dy)
                    segments.append(.cubicTo(control1: c1, control2: c2, end: end))
                    currentPoint = end
                    i += 6
                }

            case "Q": // Quadratic Bézier (absolute)
                while i + 3 < tokens.count,
                      let x1 = Double(tokens[i]), let y1 = Double(tokens[i + 1]),
                      let x = Double(tokens[i + 2]), let y = Double(tokens[i + 3]) {
                    let ctrl = Vector2D(x1, y1)
                    let end = Vector2D(x, y)
                    segments.append(.quadTo(control: ctrl, end: end))
                    currentPoint = end
                    i += 4
                }

            case "q": // Quadratic Bézier (relative)
                while i + 3 < tokens.count,
                      let dx1 = Double(tokens[i]), let dy1 = Double(tokens[i + 1]),
                      let dx = Double(tokens[i + 2]), let dy = Double(tokens[i + 3]) {
                    let ctrl = currentPoint + Vector2D(dx1, dy1)
                    let end = currentPoint + Vector2D(dx, dy)
                    segments.append(.quadTo(control: ctrl, end: end))
                    currentPoint = end
                    i += 4
                }

            case "Z", "z": // Close path
                if currentPoint.distance(to: startPoint) > 1e-6 {
                    segments.append(.lineTo(startPoint))
                }
                currentPoint = startPoint

            default:
                // Skip unknown commands
                break
            }
        }

        guard pathStarted else { return nil }

        let isClosed = currentPoint.distance(to: startPoint) < 1e-6
        return VectorPath(startPoint: startPoint, segments: segments, isClosed: isClosed)
    }

    /// Tokenize SVG path data into commands and numbers.
    private func tokenizeSVGPath(_ d: String) -> [String] {
        var tokens: [String] = []
        var current = ""

        for char in d {
            if char.isLetter {
                if !current.isEmpty {
                    tokens.append(current)
                    current = ""
                }
                tokens.append(String(char))
            } else if char == "," || char == " " || char == "\n" || char == "\r" || char == "\t" {
                if !current.isEmpty {
                    tokens.append(current)
                    current = ""
                }
            } else if char == "-" && !current.isEmpty && !current.hasSuffix("e") && !current.hasSuffix("E") {
                tokens.append(current)
                current = String(char)
            } else {
                current.append(char)
            }
        }

        if !current.isEmpty {
            tokens.append(current)
        }

        return tokens
    }

    // MARK: - Basic SVG shapes

    private func parseRectangles(_ svg: String) -> [VectorPath] {
        var paths: [VectorPath] = []
        let pattern = #"<rect[^>]*>"#
        guard let regex = try? NSRegularExpression(pattern: pattern) else { return paths }

        let nsString = svg as NSString
        let matches = regex.matches(in: svg, range: NSRange(location: 0, length: nsString.length))

        for match in matches {
            let element = nsString.substring(with: match.range)
            let x = extractAttribute(element, "x").flatMap(Double.init) ?? 0
            let y = extractAttribute(element, "y").flatMap(Double.init) ?? 0
            let w = extractAttribute(element, "width").flatMap(Double.init) ?? 0
            let h = extractAttribute(element, "height").flatMap(Double.init) ?? 0

            guard w > 0 && h > 0 else { continue }

            let start = Vector2D(x, y)
            let path = VectorPath(
                startPoint: start,
                segments: [
                    .lineTo(Vector2D(x + w, y)),
                    .lineTo(Vector2D(x + w, y + h)),
                    .lineTo(Vector2D(x, y + h)),
                    .lineTo(start),
                ],
                isClosed: true
            )
            paths.append(path)
        }

        return paths
    }

    private func parseCircles(_ svg: String) -> [VectorPath] {
        var paths: [VectorPath] = []
        let pattern = #"<circle[^>]*>"#
        guard let regex = try? NSRegularExpression(pattern: pattern) else { return paths }

        let nsString = svg as NSString
        let matches = regex.matches(in: svg, range: NSRange(location: 0, length: nsString.length))

        for match in matches {
            let element = nsString.substring(with: match.range)
            let cx = extractAttribute(element, "cx").flatMap(Double.init) ?? 0
            let cy = extractAttribute(element, "cy").flatMap(Double.init) ?? 0
            let r = extractAttribute(element, "r").flatMap(Double.init) ?? 0

            guard r > 0 else { continue }

            // Create circle as 4 cubic Bézier curves (standard approximation)
            let k = 0.5522847498 * r // Magic number for circular arc approximation
            let center = Vector2D(cx, cy)

            let top = center + Vector2D(0, -r)
            let right = center + Vector2D(r, 0)
            let bottom = center + Vector2D(0, r)
            let left = center + Vector2D(-r, 0)

            let path = VectorPath(
                startPoint: top,
                segments: [
                    .cubicTo(control1: top + Vector2D(k, 0), control2: right + Vector2D(0, -k), end: right),
                    .cubicTo(control1: right + Vector2D(0, k), control2: bottom + Vector2D(k, 0), end: bottom),
                    .cubicTo(control1: bottom + Vector2D(-k, 0), control2: left + Vector2D(0, k), end: left),
                    .cubicTo(control1: left + Vector2D(0, -k), control2: top + Vector2D(-k, 0), end: top),
                ],
                isClosed: true
            )
            paths.append(path)
        }

        return paths
    }

    private func parseEllipses(_ svg: String) -> [VectorPath] {
        // Similar to circles but with rx/ry
        return []
    }

    private func parseLines(_ svg: String) -> [VectorPath] {
        var paths: [VectorPath] = []
        let pattern = #"<line[^>]*>"#
        guard let regex = try? NSRegularExpression(pattern: pattern) else { return paths }

        let nsString = svg as NSString
        let matches = regex.matches(in: svg, range: NSRange(location: 0, length: nsString.length))

        for match in matches {
            let element = nsString.substring(with: match.range)
            let x1 = extractAttribute(element, "x1").flatMap(Double.init) ?? 0
            let y1 = extractAttribute(element, "y1").flatMap(Double.init) ?? 0
            let x2 = extractAttribute(element, "x2").flatMap(Double.init) ?? 0
            let y2 = extractAttribute(element, "y2").flatMap(Double.init) ?? 0

            let path = VectorPath(
                startPoint: Vector2D(x1, y1),
                segments: [.lineTo(Vector2D(x2, y2))],
                isClosed: false
            )
            paths.append(path)
        }

        return paths
    }

    private func parsePolylines(_ svg: String) -> [VectorPath] {
        return parsePointList(svg, tag: "polyline", closed: false)
    }

    private func parsePolygons(_ svg: String) -> [VectorPath] {
        return parsePointList(svg, tag: "polygon", closed: true)
    }

    private func parsePointList(_ svg: String, tag: String, closed: Bool) -> [VectorPath] {
        var paths: [VectorPath] = []
        let pattern = "<\(tag)[^>]*>"
        guard let regex = try? NSRegularExpression(pattern: pattern) else { return paths }

        let nsString = svg as NSString
        let matches = regex.matches(in: svg, range: NSRange(location: 0, length: nsString.length))

        for match in matches {
            let element = nsString.substring(with: match.range)
            guard let pointsStr = extractAttribute(element, "points") else { continue }

            let numbers = pointsStr.split(whereSeparator: { $0 == "," || $0 == " " || $0 == "\n" })
                .compactMap { Double($0) }

            guard numbers.count >= 4 else { continue }

            var points: [Vector2D] = []
            var idx = 0
            while idx + 1 < numbers.count {
                points.append(Vector2D(numbers[idx], numbers[idx + 1]))
                idx += 2
            }

            guard let first = points.first else { continue }
            var path = VectorPath(startPoint: first, isClosed: closed)
            for i in 1..<points.count {
                path.segments.append(.lineTo(points[i]))
            }
            if closed {
                path.segments.append(.lineTo(first))
            }
            paths.append(path)
        }

        return paths
    }

    /// Extract an attribute value from an XML element string.
    private func extractAttribute(_ element: String, _ name: String) -> String? {
        let pattern = "\(name)=[\"']([^\"']*)[\"']"
        guard let regex = try? NSRegularExpression(pattern: pattern) else { return nil }
        let nsString = element as NSString
        if let match = regex.firstMatch(in: element, range: NSRange(location: 0, length: nsString.length)) {
            return nsString.substring(with: match.range(at: 1))
        }
        return nil
    }
}
