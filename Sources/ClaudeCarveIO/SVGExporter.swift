import Foundation
import ClaudeCarveCore

/// Exports vector paths to SVG format.
public struct SVGExporter {

    public init() {}

    /// Export paths to an SVG string.
    public func export(
        paths: [VectorPath],
        materialBounds: BoundingBox2D,
        strokeWidth: Double = 0.5,
        showMaterialBounds: Bool = true
    ) -> String {
        let width = materialBounds.width
        let height = materialBounds.height
        let offsetX = -materialBounds.min.x
        let offsetY = -materialBounds.min.y

        var svg = """
        <?xml version="1.0" encoding="UTF-8"?>
        <svg xmlns="http://www.w3.org/2000/svg"
             width="\(fmt(width))mm" height="\(fmt(height))mm"
             viewBox="0 0 \(fmt(width)) \(fmt(height))"
             xmlns:claudecarve="http://claudecarve.app/ns">

        """

        // Material bounds rectangle
        if showMaterialBounds {
            svg += """
              <rect x="0" y="0" width="\(fmt(width))" height="\(fmt(height))"
                    fill="none" stroke="#0000FF" stroke-width="\(fmt(strokeWidth * 2))"
                    stroke-dasharray="5,5"/>

            """
        }

        // Export each path
        for path in paths {
            let d = pathToSVGData(path, offsetX: offsetX, offsetY: offsetY, flipY: height)
            let fillRule = path.isClosed ? "evenodd" : "none"
            let fill = path.isClosed ? "none" : "none"

            svg += """
              <path d="\(d)"
                    fill="\(fill)" stroke="#000000" stroke-width="\(fmt(strokeWidth))"
                    fill-rule="\(fillRule)"
                    claudecarve:id="\(path.id.uuidString)"/>

            """
        }

        svg += "</svg>\n"
        return svg
    }

    /// Export paths to an SVG file.
    public func exportToFile(
        paths: [VectorPath],
        materialBounds: BoundingBox2D,
        url: URL
    ) throws {
        let svg = export(paths: paths, materialBounds: materialBounds)
        try svg.write(to: url, atomically: true, encoding: .utf8)
    }

    /// Convert a VectorPath to SVG path data string.
    private func pathToSVGData(_ path: VectorPath, offsetX: Double, offsetY: Double, flipY: Double) -> String {
        var d = "M \(fmt(path.startPoint.x + offsetX)) \(fmt(flipY - (path.startPoint.y + offsetY)))"

        for segment in path.segments {
            switch segment {
            case .lineTo(let p):
                d += " L \(fmt(p.x + offsetX)) \(fmt(flipY - (p.y + offsetY)))"

            case .quadTo(let ctrl, let end):
                d += " Q \(fmt(ctrl.x + offsetX)) \(fmt(flipY - (ctrl.y + offsetY)))"
                d += " \(fmt(end.x + offsetX)) \(fmt(flipY - (end.y + offsetY)))"

            case .cubicTo(let c1, let c2, let end):
                d += " C \(fmt(c1.x + offsetX)) \(fmt(flipY - (c1.y + offsetY)))"
                d += " \(fmt(c2.x + offsetX)) \(fmt(flipY - (c2.y + offsetY)))"
                d += " \(fmt(end.x + offsetX)) \(fmt(flipY - (end.y + offsetY)))"

            case .arcTo(let center, let radius, let startAngle, let endAngle, let clockwise):
                // Convert arc to SVG arc command
                let endPoint = center + Vector2D.fromAngle(endAngle) * radius
                var sweep = endAngle - startAngle
                if clockwise && sweep > 0 { sweep -= 2 * .pi }
                if !clockwise && sweep < 0 { sweep += 2 * .pi }
                let largeArc = abs(sweep) > .pi ? 1 : 0
                let sweepFlag = clockwise ? 0 : 1

                d += " A \(fmt(radius)) \(fmt(radius)) 0 \(largeArc) \(sweepFlag)"
                d += " \(fmt(endPoint.x + offsetX)) \(fmt(flipY - (endPoint.y + offsetY)))"
            }
        }

        if path.isClosed {
            d += " Z"
        }

        return d
    }

    private func fmt(_ v: Double) -> String {
        String(format: "%.4f", v)
    }
}
