import SwiftUI
import AppKit
import ClaudeCarveCore

/// Interactive 2D design canvas using NSView for precise mouse handling.
/// Supports drawing, selecting, and manipulating vector paths.
public struct DesignCanvasView: NSViewRepresentable {
    @Binding var document: ClaudeCarveDocument
    @Binding var selectedTool: DesignTool
    @Binding var selectedPathIDs: Set<UUID>
    @Binding var zoomLevel: Double
    @Binding var panOffset: CGSize

    public init(
        document: Binding<ClaudeCarveDocument>,
        selectedTool: Binding<DesignTool>,
        selectedPathIDs: Binding<Set<UUID>>,
        zoomLevel: Binding<Double>,
        panOffset: Binding<CGSize>
    ) {
        self._document = document
        self._selectedTool = selectedTool
        self._selectedPathIDs = selectedPathIDs
        self._zoomLevel = zoomLevel
        self._panOffset = panOffset
    }

    public func makeNSView(context: Context) -> CanvasNSView {
        let view = CanvasNSView()
        view.delegate = context.coordinator
        return view
    }

    public func updateNSView(_ nsView: CanvasNSView, context: Context) {
        nsView.document = document
        nsView.selectedTool = selectedTool
        nsView.selectedPathIDs = selectedPathIDs
        nsView.zoomLevel = zoomLevel
        nsView.panOffset = panOffset
        nsView.needsDisplay = true
    }

    public func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }

    @MainActor
    public class Coordinator: NSObject, CanvasDelegate {
        var parent: DesignCanvasView

        init(_ parent: DesignCanvasView) {
            self.parent = parent
        }

        public func canvasDidSelectPaths(_ pathIDs: Set<UUID>) {
            parent.selectedPathIDs = pathIDs
        }

        public func canvasDidAddPath(_ path: VectorPath) {
            parent.document.addPath(path)
        }

        public func canvasDidUpdateZoom(_ zoom: Double) {
            parent.zoomLevel = zoom
        }

        public func canvasDidUpdatePan(_ offset: CGSize) {
            parent.panOffset = offset
        }
    }
}

@MainActor
public protocol CanvasDelegate: AnyObject {
    func canvasDidSelectPaths(_ pathIDs: Set<UUID>)
    func canvasDidAddPath(_ path: VectorPath)
    func canvasDidUpdateZoom(_ zoom: Double)
    func canvasDidUpdatePan(_ offset: CGSize)
}

/// The NSView that handles drawing and mouse interaction for the design canvas.
public class CanvasNSView: NSView {
    weak var delegate: CanvasDelegate?

    var document = ClaudeCarveDocument()
    var selectedTool: DesignTool = .select
    var selectedPathIDs: Set<UUID> = []
    var zoomLevel: Double = 1.0
    var panOffset: CGSize = .zero

    // Drawing state
    private var drawingPoints: [Vector2D] = []
    private var isDrawing = false
    private var dragStart: NSPoint?
    private var currentMouse: NSPoint?

    // Colors
    private let gridColor = NSColor.gray.withAlphaComponent(0.15)
    private let materialBorderColor = NSColor.systemBlue
    private let materialFillColor = NSColor.white
    private let pathColor = NSColor.black
    private let selectedPathColor = NSColor.systemBlue
    private let drawingColor = NSColor.systemRed
    private let cursorCrosshairColor = NSColor.gray.withAlphaComponent(0.5)

    public override var isFlipped: Bool { true }
    public override var acceptsFirstResponder: Bool { true }

    public override func draw(_ dirtyRect: NSRect) {
        super.draw(dirtyRect)

        guard let context = NSGraphicsContext.current?.cgContext else { return }
        let size = bounds.size

        // Background
        NSColor.controlBackgroundColor.setFill()
        context.fill(bounds)

        // Grid
        drawGrid(context: context, size: size)

        // Material bounds
        drawMaterialBounds(context: context, size: size)

        // Paths
        drawPaths(context: context, size: size)

        // Drawing preview
        if isDrawing {
            drawCurrentShape(context: context, size: size)
        }

        // Crosshair cursor
        if let mouse = currentMouse {
            drawCrosshair(context: context, at: mouse, size: size)
        }
    }

    // MARK: - Mouse handling

    public override func mouseDown(with event: NSEvent) {
        let point = convert(event.locationInWindow, from: nil)
        let worldPoint = screenToWorld(point)
        dragStart = point

        switch selectedTool {
        case .select:
            handleSelect(at: worldPoint, extend: event.modifierFlags.contains(.shift))

        case .pan:
            // Pan starts on drag
            break

        case .drawLine, .drawRectangle, .drawCircle, .drawEllipse, .drawPolygon:
            isDrawing = true
            drawingPoints = [worldPoint]

        case .drawArc:
            isDrawing = true
            drawingPoints.append(worldPoint)

        default:
            break
        }

        needsDisplay = true
    }

    public override func mouseDragged(with event: NSEvent) {
        let point = convert(event.locationInWindow, from: nil)
        currentMouse = point

        if selectedTool == .pan, let start = dragStart {
            let dx = point.x - start.x
            let dy = point.y - start.y
            panOffset = CGSize(
                width: panOffset.width + dx,
                height: panOffset.height - dy
            )
            delegate?.canvasDidUpdatePan(panOffset)
            dragStart = point
        }

        needsDisplay = true
    }

    public override func mouseUp(with event: NSEvent) {
        let point = convert(event.locationInWindow, from: nil)
        let worldPoint = screenToWorld(point)

        if isDrawing && drawingPoints.count >= 1 {
            finishDrawing(endPoint: worldPoint)
        }

        dragStart = nil
        needsDisplay = true
    }

    public override func mouseMoved(with event: NSEvent) {
        currentMouse = convert(event.locationInWindow, from: nil)
        needsDisplay = true
    }

    public override func scrollWheel(with event: NSEvent) {
        let zoomDelta = event.deltaY * 0.05
        zoomLevel = max(0.05, min(20, zoomLevel + zoomDelta))
        delegate?.canvasDidUpdateZoom(zoomLevel)
        needsDisplay = true
    }

    public override func updateTrackingAreas() {
        super.updateTrackingAreas()
        for area in trackingAreas {
            removeTrackingArea(area)
        }
        let area = NSTrackingArea(
            rect: bounds,
            options: [.activeInActiveApp, .mouseMoved, .mouseEnteredAndExited],
            owner: self,
            userInfo: nil
        )
        addTrackingArea(area)
    }

    // MARK: - Selection

    private func handleSelect(at point: Vector2D, extend: Bool) {
        var hitID: UUID?

        // Find the path closest to the click point
        for (id, path) in document.paths {
            let points = path.flattenedPoints(tolerance: 0.5)
            for p in points {
                if p.distance(to: point) < 5.0 / zoomLevel {
                    hitID = id
                    break
                }
            }
            if hitID != nil { break }
        }

        if let id = hitID {
            if extend {
                if selectedPathIDs.contains(id) {
                    selectedPathIDs.remove(id)
                } else {
                    selectedPathIDs.insert(id)
                }
            } else {
                selectedPathIDs = [id]
            }
        } else if !extend {
            selectedPathIDs = []
        }

        delegate?.canvasDidSelectPaths(selectedPathIDs)
    }

    // MARK: - Drawing completion

    private func finishDrawing(endPoint: Vector2D) {
        guard let startPoint = drawingPoints.first else { return }
        let path: VectorPath?

        switch selectedTool {
        case .drawLine:
            path = VectorPath(
                startPoint: startPoint,
                segments: [.lineTo(endPoint)],
                isClosed: false
            )

        case .drawRectangle:
            let minX = min(startPoint.x, endPoint.x)
            let minY = min(startPoint.y, endPoint.y)
            let maxX = max(startPoint.x, endPoint.x)
            let maxY = max(startPoint.y, endPoint.y)
            let tl = Vector2D(minX, minY)
            let tr = Vector2D(maxX, minY)
            let br = Vector2D(maxX, maxY)
            let bl = Vector2D(minX, maxY)
            path = VectorPath(
                startPoint: tl,
                segments: [.lineTo(tr), .lineTo(br), .lineTo(bl), .lineTo(tl)],
                isClosed: true
            )

        case .drawCircle:
            let center = startPoint
            let radius = startPoint.distance(to: endPoint)
            guard radius > 0.5 else {
                isDrawing = false
                drawingPoints = []
                return
            }
            let k = 0.5522847498 * radius
            let top = center + Vector2D(0, -radius)
            let right = center + Vector2D(radius, 0)
            let bottom = center + Vector2D(0, radius)
            let left = center + Vector2D(-radius, 0)
            path = VectorPath(
                startPoint: top,
                segments: [
                    .cubicTo(control1: top + Vector2D(k, 0), control2: right + Vector2D(0, -k), end: right),
                    .cubicTo(control1: right + Vector2D(0, k), control2: bottom + Vector2D(k, 0), end: bottom),
                    .cubicTo(control1: bottom + Vector2D(-k, 0), control2: left + Vector2D(0, k), end: left),
                    .cubicTo(control1: left + Vector2D(0, -k), control2: top + Vector2D(-k, 0), end: top),
                ],
                isClosed: true
            )

        case .drawEllipse:
            let cx = (startPoint.x + endPoint.x) / 2
            let cy = (startPoint.y + endPoint.y) / 2
            let rx = abs(endPoint.x - startPoint.x) / 2
            let ry = abs(endPoint.y - startPoint.y) / 2
            guard rx > 0.5 && ry > 0.5 else {
                isDrawing = false
                drawingPoints = []
                return
            }
            let kx = 0.5522847498 * rx
            let ky = 0.5522847498 * ry
            let center = Vector2D(cx, cy)
            let top = center + Vector2D(0, -ry)
            let right = center + Vector2D(rx, 0)
            let bottom = center + Vector2D(0, ry)
            let left = center + Vector2D(-rx, 0)
            path = VectorPath(
                startPoint: top,
                segments: [
                    .cubicTo(control1: top + Vector2D(kx, 0), control2: right + Vector2D(0, -ky), end: right),
                    .cubicTo(control1: right + Vector2D(0, ky), control2: bottom + Vector2D(kx, 0), end: bottom),
                    .cubicTo(control1: bottom + Vector2D(-kx, 0), control2: left + Vector2D(0, ky), end: left),
                    .cubicTo(control1: left + Vector2D(0, -ky), control2: top + Vector2D(-kx, 0), end: top),
                ],
                isClosed: true
            )

        default:
            path = nil
        }

        if let newPath = path {
            delegate?.canvasDidAddPath(newPath)
        }

        isDrawing = false
        drawingPoints = []
    }

    // MARK: - Coordinate transforms

    private func worldToScreen(_ point: Vector2D) -> CGPoint {
        let x = bounds.width / 2 + point.x * zoomLevel + panOffset.width
        let y = bounds.height / 2 - point.y * zoomLevel + panOffset.height
        return CGPoint(x: x, y: y)
    }

    private func screenToWorld(_ point: CGPoint) -> Vector2D {
        let x = (point.x - bounds.width / 2 - panOffset.width) / zoomLevel
        let y = -(point.y - bounds.height / 2 - panOffset.height) / zoomLevel
        return Vector2D(x, y)
    }

    // MARK: - Drawing helpers

    private func drawGrid(context: CGContext, size: CGSize) {
        let gridSpacing = 10.0 * zoomLevel
        guard gridSpacing > 3 else { return } // Don't draw grid if too small

        context.setStrokeColor(gridColor.cgColor)
        context.setLineWidth(0.5)

        let centerX = size.width / 2 + panOffset.width
        let centerY = size.height / 2 + panOffset.height

        var x = centerX.truncatingRemainder(dividingBy: gridSpacing)
        while x < size.width {
            context.move(to: CGPoint(x: x, y: 0))
            context.addLine(to: CGPoint(x: x, y: size.height))
            x += gridSpacing
        }

        var y = centerY.truncatingRemainder(dividingBy: gridSpacing)
        while y < size.height {
            context.move(to: CGPoint(x: 0, y: y))
            context.addLine(to: CGPoint(x: size.width, y: y))
            y += gridSpacing
        }

        context.strokePath()
    }

    private func drawMaterialBounds(context: CGContext, size: CGSize) {
        let bounds = document.materialSetup.bounds
        let topLeft = worldToScreen(Vector2D(bounds.min.x, bounds.max.y))
        let bottomRight = worldToScreen(Vector2D(bounds.max.x, bounds.min.y))

        let rect = CGRect(
            x: topLeft.x, y: topLeft.y,
            width: bottomRight.x - topLeft.x,
            height: bottomRight.y - topLeft.y
        )

        context.setFillColor(materialFillColor.cgColor)
        context.fill(rect)

        context.setStrokeColor(materialBorderColor.cgColor)
        context.setLineWidth(2)
        context.stroke(rect)
    }

    private func drawPaths(context: CGContext, size: CGSize) {
        for layer in document.layers where layer.isVisible {
            for pathID in layer.pathIDs {
                guard let vectorPath = document.paths[pathID] else { continue }
                let points = vectorPath.flattenedPoints(tolerance: 0.1)
                guard points.count >= 2 else { continue }

                let isSelected = selectedPathIDs.contains(pathID)
                context.setStrokeColor(isSelected ? selectedPathColor.cgColor : pathColor.cgColor)
                context.setLineWidth(isSelected ? 2.0 : 1.0)

                let first = worldToScreen(points[0])
                context.move(to: first)

                for i in 1..<points.count {
                    context.addLine(to: worldToScreen(points[i]))
                }

                if vectorPath.isClosed {
                    context.closePath()
                }

                context.strokePath()

                // Draw selection handles
                if isSelected {
                    for point in points {
                        let screen = worldToScreen(point)
                        let handleRect = CGRect(x: screen.x - 3, y: screen.y - 3, width: 6, height: 6)
                        context.setFillColor(selectedPathColor.cgColor)
                        context.fill(handleRect)
                    }
                }
            }
        }
    }

    private func drawCurrentShape(context: CGContext, size: CGSize) {
        guard let start = drawingPoints.first, let mouse = currentMouse else { return }

        let worldMouse = screenToWorld(mouse)
        context.setStrokeColor(drawingColor.cgColor)
        context.setLineWidth(1)
        context.setLineDash(phase: 0, lengths: [5, 5])

        let screenStart = worldToScreen(start)
        let screenEnd = worldToScreen(worldMouse)

        switch selectedTool {
        case .drawLine:
            context.move(to: screenStart)
            context.addLine(to: screenEnd)

        case .drawRectangle:
            let rect = CGRect(
                x: min(screenStart.x, screenEnd.x),
                y: min(screenStart.y, screenEnd.y),
                width: abs(screenEnd.x - screenStart.x),
                height: abs(screenEnd.y - screenStart.y)
            )
            context.addRect(rect)

        case .drawCircle:
            let radius = start.distance(to: worldMouse) * zoomLevel
            let circle = CGRect(
                x: screenStart.x - radius,
                y: screenStart.y - radius,
                width: radius * 2,
                height: radius * 2
            )
            context.addEllipse(in: circle)

        case .drawEllipse:
            let rect = CGRect(
                x: min(screenStart.x, screenEnd.x),
                y: min(screenStart.y, screenEnd.y),
                width: abs(screenEnd.x - screenStart.x),
                height: abs(screenEnd.y - screenStart.y)
            )
            context.addEllipse(in: rect)

        default:
            break
        }

        context.strokePath()
        context.setLineDash(phase: 0, lengths: [])
    }

    private func drawCrosshair(context: CGContext, at point: NSPoint, size: CGSize) {
        context.setStrokeColor(cursorCrosshairColor.cgColor)
        context.setLineWidth(0.5)

        context.move(to: CGPoint(x: point.x, y: 0))
        context.addLine(to: CGPoint(x: point.x, y: size.height))
        context.move(to: CGPoint(x: 0, y: point.y))
        context.addLine(to: CGPoint(x: size.width, y: point.y))

        context.strokePath()

        // Coordinate display
        let worldPoint = screenToWorld(point)
        let coordString = String(format: "X: %.2f  Y: %.2f", worldPoint.x, worldPoint.y)
        let attrs: [NSAttributedString.Key: Any] = [
            .font: NSFont.monospacedSystemFont(ofSize: 10, weight: .regular),
            .foregroundColor: NSColor.secondaryLabelColor,
        ]
        let nsString = coordString as NSString
        let textSize = nsString.size(withAttributes: attrs)
        let textRect = CGRect(
            x: point.x + 15,
            y: point.y + 5,
            width: textSize.width + 4,
            height: textSize.height + 2
        )

        NSColor.windowBackgroundColor.withAlphaComponent(0.8).setFill()
        NSBezierPath(roundedRect: textRect, xRadius: 3, yRadius: 3).fill()

        nsString.draw(
            at: CGPoint(x: textRect.origin.x + 2, y: textRect.origin.y + 1),
            withAttributes: attrs
        )
    }
}
