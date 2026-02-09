import Foundation
import VCarveCore

/// Imports DXF (Drawing Exchange Format) files into VCarve vector paths.
/// DXF is the most common format for CNC vector data exchange.
public struct DXFImporter {

    public init() {}

    /// Import a DXF file from the given URL.
    public func importFile(at url: URL) throws -> [VectorPath] {
        let data = try Data(contentsOf: url)
        guard let content = String(data: data, encoding: .utf8) ?? String(data: data, encoding: .ascii) else {
            throw DXFImportError.invalidData
        }
        return try parse(content)
    }

    /// Parse DXF content string.
    public func parse(_ content: String) throws -> [VectorPath] {
        let lines = content.components(separatedBy: .newlines).map { $0.trimmingCharacters(in: .whitespaces) }
        var paths: [VectorPath] = []
        var i = 0

        // Find ENTITIES section
        while i < lines.count {
            if lines[i] == "ENTITIES" { break }
            i += 1
        }

        // Parse entities
        while i < lines.count {
            if lines[i] == "ENDSEC" { break }

            if i + 1 < lines.count && lines[i] == "  0" || lines[i] == "0" {
                let entityType = i + 1 < lines.count ? lines[i + 1].uppercased() : ""

                switch entityType {
                case "LINE":
                    if let path = parseLine(lines: lines, from: &i) {
                        paths.append(path)
                    }
                case "CIRCLE":
                    if let path = parseCircle(lines: lines, from: &i) {
                        paths.append(path)
                    }
                case "ARC":
                    if let path = parseArc(lines: lines, from: &i) {
                        paths.append(path)
                    }
                case "LWPOLYLINE":
                    if let path = parseLWPolyline(lines: lines, from: &i) {
                        paths.append(path)
                    }
                case "POLYLINE":
                    if let path = parsePolyline(lines: lines, from: &i) {
                        paths.append(path)
                    }
                case "SPLINE":
                    if let path = parseSpline(lines: lines, from: &i) {
                        paths.append(path)
                    }
                default:
                    i += 1
                }
            } else {
                i += 1
            }
        }

        return paths
    }

    // MARK: - Entity parsers

    private func parseLine(lines: [String], from i: inout Int) -> VectorPath? {
        var x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0
        i += 2 // Skip "0" and "LINE"

        while i + 1 < lines.count {
            let code = Int(lines[i].trimmingCharacters(in: .whitespaces)) ?? -1
            let value = lines[i + 1].trimmingCharacters(in: .whitespaces)

            switch code {
            case 0: // Next entity
                let start = Vector2D(x1, y1)
                let end = Vector2D(x2, y2)
                return VectorPath(startPoint: start, segments: [.lineTo(end)], isClosed: false)
            case 10: x1 = Double(value) ?? 0
            case 20: y1 = Double(value) ?? 0
            case 11: x2 = Double(value) ?? 0
            case 21: y2 = Double(value) ?? 0
            default: break
            }
            i += 2
        }

        let start = Vector2D(x1, y1)
        let end = Vector2D(x2, y2)
        return VectorPath(startPoint: start, segments: [.lineTo(end)], isClosed: false)
    }

    private func parseCircle(lines: [String], from i: inout Int) -> VectorPath? {
        var cx = 0.0, cy = 0.0, r = 0.0
        i += 2

        while i + 1 < lines.count {
            let code = Int(lines[i].trimmingCharacters(in: .whitespaces)) ?? -1
            let value = lines[i + 1].trimmingCharacters(in: .whitespaces)

            switch code {
            case 0:
                return createCirclePath(cx: cx, cy: cy, r: r)
            case 10: cx = Double(value) ?? 0
            case 20: cy = Double(value) ?? 0
            case 40: r = Double(value) ?? 0
            default: break
            }
            i += 2
        }

        return createCirclePath(cx: cx, cy: cy, r: r)
    }

    private func parseArc(lines: [String], from i: inout Int) -> VectorPath? {
        var cx = 0.0, cy = 0.0, r = 0.0
        var startAngle = 0.0, endAngle = 360.0
        i += 2

        while i + 1 < lines.count {
            let code = Int(lines[i].trimmingCharacters(in: .whitespaces)) ?? -1
            let value = lines[i + 1].trimmingCharacters(in: .whitespaces)

            switch code {
            case 0:
                return createArcPath(cx: cx, cy: cy, r: r, startAngle: startAngle, endAngle: endAngle)
            case 10: cx = Double(value) ?? 0
            case 20: cy = Double(value) ?? 0
            case 40: r = Double(value) ?? 0
            case 50: startAngle = Double(value) ?? 0
            case 51: endAngle = Double(value) ?? 360
            default: break
            }
            i += 2
        }

        return createArcPath(cx: cx, cy: cy, r: r, startAngle: startAngle, endAngle: endAngle)
    }

    private func parseLWPolyline(lines: [String], from i: inout Int) -> VectorPath? {
        var points: [Vector2D] = []
        var isClosed = false
        var currentX = 0.0
        i += 2

        while i + 1 < lines.count {
            let code = Int(lines[i].trimmingCharacters(in: .whitespaces)) ?? -1
            let value = lines[i + 1].trimmingCharacters(in: .whitespaces)

            switch code {
            case 0:
                return createPolylinePath(points: points, isClosed: isClosed)
            case 70:
                isClosed = (Int(value) ?? 0) & 1 == 1
            case 10:
                currentX = Double(value) ?? 0
            case 20:
                let y = Double(value) ?? 0
                points.append(Vector2D(currentX, y))
            default: break
            }
            i += 2
        }

        return createPolylinePath(points: points, isClosed: isClosed)
    }

    private func parsePolyline(lines: [String], from i: inout Int) -> VectorPath? {
        // Simplified - skip for now, LWPOLYLINE is more common
        i += 2
        while i + 1 < lines.count {
            if lines[i].trimmingCharacters(in: .whitespaces) == "0" &&
               lines[i + 1].trimmingCharacters(in: .whitespaces) == "SEQEND" {
                i += 2
                break
            }
            i += 2
        }
        return nil
    }

    private func parseSpline(lines: [String], from i: inout Int) -> VectorPath? {
        // Spline parsing is complex — skip for initial version
        i += 2
        while i + 1 < lines.count {
            let code = Int(lines[i].trimmingCharacters(in: .whitespaces)) ?? -1
            if code == 0 { break }
            i += 2
        }
        return nil
    }

    // MARK: - Shape creation helpers

    private func createCirclePath(cx: Double, cy: Double, r: Double) -> VectorPath? {
        guard r > 0 else { return nil }
        let center = Vector2D(cx, cy)
        let startAngleRad = 0.0

        let start = center + Vector2D.fromAngle(startAngleRad) * r
        return VectorPath(
            startPoint: start,
            segments: [
                .arcTo(center: center, radius: r, startAngle: 0, endAngle: .pi, clockwise: false),
                .arcTo(center: center, radius: r, startAngle: .pi, endAngle: 2 * .pi, clockwise: false),
            ],
            isClosed: true
        )
    }

    private func createArcPath(cx: Double, cy: Double, r: Double, startAngle: Double, endAngle: Double) -> VectorPath? {
        guard r > 0 else { return nil }
        let center = Vector2D(cx, cy)
        let startRad = startAngle * .pi / 180
        let endRad = endAngle * .pi / 180
        let start = center + Vector2D.fromAngle(startRad) * r

        return VectorPath(
            startPoint: start,
            segments: [
                .arcTo(center: center, radius: r, startAngle: startRad, endAngle: endRad, clockwise: false),
            ],
            isClosed: false
        )
    }

    private func createPolylinePath(points: [Vector2D], isClosed: Bool) -> VectorPath? {
        guard let first = points.first, points.count >= 2 else { return nil }
        var path = VectorPath(startPoint: first, isClosed: isClosed)
        for i in 1..<points.count {
            path.segments.append(.lineTo(points[i]))
        }
        if isClosed {
            path.segments.append(.lineTo(first))
        }
        return path
    }
}

public enum DXFImportError: Error, LocalizedError {
    case invalidData
    case parseError(String)

    public var errorDescription: String? {
        switch self {
        case .invalidData: return "Invalid DXF data"
        case .parseError(let msg): return "DXF parse error: \(msg)"
        }
    }
}
