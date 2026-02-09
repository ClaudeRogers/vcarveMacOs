import SwiftUI
import ClaudeCarveCore

/// The main document view — the primary workspace showing the 2D design canvas
/// with inspector panels, toolbar, and toolpath list.
public struct ClaudeCarveDocumentView: View {
    @Binding var document: ClaudeCarveDocument
    @State private var selectedTool: DesignTool = .select
    @State private var zoomLevel: Double = 1.0
    @State private var panOffset: CGSize = .zero
    @State private var selectedPathIDs: Set<UUID> = []
    @State private var showToolpathPanel: Bool = true
    @State private var showToolDatabase: Bool = false
    @State private var showMaterialSetup: Bool = false
    @State private var show3DPreview: Bool = false

    public init(document: Binding<ClaudeCarveDocument>) {
        self._document = document
    }

    public var body: some View {
        HSplitView {
            // Left panel: Layers & Toolpath list
            VStack(spacing: 0) {
                layerPanel
                Divider()
                toolpathListPanel
            }
            .frame(minWidth: 220, idealWidth: 250, maxWidth: 300)

            // Center: Design canvas
            VStack(spacing: 0) {
                designToolbar
                Divider()
                designCanvas
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                Divider()
                statusBar
            }

            // Right panel: Properties inspector
            if showToolpathPanel {
                propertiesPanel
                    .frame(minWidth: 280, idealWidth: 320, maxWidth: 400)
            }
        }
        .toolbar {
            mainToolbar
        }
        .sheet(isPresented: $showMaterialSetup) {
            MaterialSetupView(setup: $document.materialSetup)
        }
        .sheet(isPresented: $showToolDatabase) {
            ToolDatabaseView(tools: $document.toolDatabase)
        }
        .sheet(isPresented: $show3DPreview) {
            ToolpathPreview3DView(document: document)
        }
    }

    // MARK: - Layer panel

    private var layerPanel: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text("Layers")
                    .font(.headline)
                Spacer()
                Button(action: addLayer) {
                    Image(systemName: "plus")
                }
                .buttonStyle(.borderless)
            }
            .padding(.horizontal, 8)
            .padding(.top, 8)

            List(document.layers.indices, id: \.self) { index in
                HStack {
                    Image(systemName: document.layers[index].isVisible ? "eye" : "eye.slash")
                        .foregroundColor(.secondary)
                        .onTapGesture {
                            document.layers[index].isVisible.toggle()
                        }

                    Image(systemName: document.layers[index].isLocked ? "lock" : "lock.open")
                        .foregroundColor(.secondary)
                        .onTapGesture {
                            document.layers[index].isLocked.toggle()
                        }

                    Circle()
                        .fill(colorForLayer(document.layers[index].color))
                        .frame(width: 10, height: 10)

                    Text(document.layers[index].name)
                        .font(.system(size: 12))

                    Spacer()

                    Text("\(document.layers[index].pathIDs.count)")
                        .font(.system(size: 10))
                        .foregroundColor(.secondary)
                }
                .padding(.vertical, 2)
            }
            .listStyle(.sidebar)
        }
        .frame(minHeight: 150)
    }

    // MARK: - Toolpath list

    private var toolpathListPanel: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text("Toolpaths")
                    .font(.headline)
                Spacer()
            }
            .padding(.horizontal, 8)
            .padding(.top, 8)

            if document.toolpathConfigs.isEmpty {
                Text("No toolpaths created")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding()
            } else {
                List(document.toolpathConfigs) { config in
                    HStack {
                        Image(systemName: config.isVisible ? "eye" : "eye.slash")
                            .foregroundColor(.secondary)

                        Image(systemName: iconForToolpathType(config.type))
                            .foregroundColor(.accentColor)

                        VStack(alignment: .leading) {
                            Text(config.name)
                                .font(.system(size: 12))
                            Text(config.type.rawValue.capitalized)
                                .font(.system(size: 10))
                                .foregroundColor(.secondary)
                        }
                    }
                }
                .listStyle(.sidebar)
            }

            // Toolpath creation buttons
            VStack(spacing: 4) {
                toolpathButton("Profile", icon: "square.dashed", type: .profile)
                toolpathButton("Pocket", icon: "square.fill", type: .pocket)
                toolpathButton("V-Carve", icon: "v.square", type: .vCarve)
                toolpathButton("Drilling", icon: "circle.circle", type: .drilling)
            }
            .padding(8)
        }
    }

    private func toolpathButton(_ title: String, icon: String, type: ToolpathType) -> some View {
        Button(action: { createToolpath(type: type) }) {
            HStack {
                Image(systemName: icon)
                Text(title)
                    .font(.system(size: 11))
            }
            .frame(maxWidth: .infinity, alignment: .leading)
        }
        .buttonStyle(.bordered)
        .controlSize(.small)
    }

    // MARK: - Design toolbar

    private var designToolbar: some View {
        HStack(spacing: 12) {
            // Selection tools
            ForEach(DesignTool.allCases, id: \.self) { tool in
                Button(action: { selectedTool = tool }) {
                    Image(systemName: tool.iconName)
                }
                .buttonStyle(.bordered)
                .tint(selectedTool == tool ? .accentColor : .secondary)
                .help(tool.displayName)
            }

            Divider()
                .frame(height: 20)

            // Zoom controls
            Button(action: { zoomLevel = max(0.1, zoomLevel - 0.25) }) {
                Image(systemName: "minus.magnifyingglass")
            }
            .buttonStyle(.borderless)

            Text("\(Int(zoomLevel * 100))%")
                .font(.system(size: 11, design: .monospaced))
                .frame(width: 45)

            Button(action: { zoomLevel = min(10, zoomLevel + 0.25) }) {
                Image(systemName: "plus.magnifyingglass")
            }
            .buttonStyle(.borderless)

            Button(action: { zoomLevel = 1.0; panOffset = .zero }) {
                Image(systemName: "arrow.up.left.and.arrow.down.right")
            }
            .buttonStyle(.borderless)
            .help("Fit to window")

            Spacer()
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
    }

    // MARK: - Design canvas

    private var designCanvas: some View {
        GeometryReader { geometry in
            ZStack {
                // Background
                Color(nsColor: .controlBackgroundColor)

                // Grid
                Canvas { context, size in
                    drawGrid(context: context, size: size)
                    drawMaterialBounds(context: context, size: size)
                    drawPaths(context: context, size: size)
                }
            }
            .gesture(
                MagnifyGesture()
                    .onChanged { value in
                        zoomLevel *= value.magnification
                    }
            )
            .gesture(
                DragGesture()
                    .onChanged { value in
                        if selectedTool == .pan {
                            panOffset = CGSize(
                                width: panOffset.width + value.translation.width,
                                height: panOffset.height + value.translation.height
                            )
                        }
                    }
            )
        }
    }

    // MARK: - Properties panel

    private var propertiesPanel: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 12) {
                Text("Properties")
                    .font(.headline)
                    .padding(.horizontal)
                    .padding(.top, 8)

                if selectedPathIDs.isEmpty {
                    Text("Select vectors to view properties")
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .padding(.horizontal)
                } else {
                    Text("\(selectedPathIDs.count) vector(s) selected")
                        .font(.caption)
                        .padding(.horizontal)
                }

                Divider()

                // Material info
                GroupBox("Material") {
                    VStack(alignment: .leading, spacing: 4) {
                        propertyRow("Size", "\(fmt(document.materialSetup.width)) x \(fmt(document.materialSetup.height)) \(document.materialSetup.unit.abbreviation)")
                        propertyRow("Thickness", "\(fmt(document.materialSetup.thickness)) \(document.materialSetup.unit.abbreviation)")
                        propertyRow("Z-Zero", document.materialSetup.zZeroPosition.rawValue)

                        Button("Edit Material Setup...") {
                            showMaterialSetup = true
                        }
                        .buttonStyle(.link)
                        .font(.caption)
                    }
                }
                .padding(.horizontal)

                Spacer()
            }
        }
    }

    // MARK: - Status bar

    private var statusBar: some View {
        HStack {
            Text("Tool: \(selectedTool.displayName)")
                .font(.system(size: 11))

            Spacer()

            Text("Vectors: \(document.paths.count)")
                .font(.system(size: 11))

            Text("Toolpaths: \(document.toolpathConfigs.count)")
                .font(.system(size: 11))

            Text("Units: \(document.materialSetup.unit.abbreviation)")
                .font(.system(size: 11))
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(Color(nsColor: .windowBackgroundColor))
    }

    // MARK: - Main toolbar

    @ToolbarContentBuilder
    private var mainToolbar: some ToolbarContent {
        ToolbarItemGroup(placement: .primaryAction) {
            Button(action: { showMaterialSetup = true }) {
                Label("Material Setup", systemImage: "square.3.layers.3d")
            }
            .help("Material Setup")

            Button(action: { showToolDatabase = true }) {
                Label("Tool Database", systemImage: "wrench.and.screwdriver")
            }
            .help("Tool Database")

            Button(action: { show3DPreview = true }) {
                Label("3D Preview", systemImage: "cube")
            }
            .help("3D Preview")

            Button(action: { showToolpathPanel.toggle() }) {
                Label("Toggle Panel", systemImage: "sidebar.right")
            }
            .help("Toggle Properties Panel")
        }
    }

    // MARK: - Canvas drawing

    private func drawGrid(context: GraphicsContext, size: CGSize) {
        let gridSpacing = 10.0 * zoomLevel
        let centerX = size.width / 2 + panOffset.width
        let centerY = size.height / 2 + panOffset.height

        // Draw grid lines
        var x = centerX.truncatingRemainder(dividingBy: gridSpacing)
        while x < size.width {
            var path = SwiftUI.Path()
            path.move(to: CGPoint(x: x, y: 0))
            path.addLine(to: CGPoint(x: x, y: size.height))
            context.stroke(path, with: .color(.gray.opacity(0.15)), lineWidth: 0.5)
            x += gridSpacing
        }

        var y = centerY.truncatingRemainder(dividingBy: gridSpacing)
        while y < size.height {
            var path = SwiftUI.Path()
            path.move(to: CGPoint(x: 0, y: y))
            path.addLine(to: CGPoint(x: size.width, y: y))
            context.stroke(path, with: .color(.gray.opacity(0.15)), lineWidth: 0.5)
            y += gridSpacing
        }
    }

    private func drawMaterialBounds(context: GraphicsContext, size: CGSize) {
        let bounds = document.materialSetup.bounds
        let origin = worldToScreen(Vector2D(bounds.min.x, bounds.min.y), size: size)
        let corner = worldToScreen(Vector2D(bounds.max.x, bounds.max.y), size: size)

        let rect = CGRect(
            x: origin.x,
            y: origin.y,
            width: corner.x - origin.x,
            height: corner.y - origin.y
        )

        // Material background
        context.fill(SwiftUI.Path(rect), with: .color(.white))

        // Material border
        context.stroke(SwiftUI.Path(rect), with: .color(.blue), lineWidth: 2)
    }

    private func drawPaths(context: GraphicsContext, size: CGSize) {
        for layer in document.layers where layer.isVisible {
            for pathID in layer.pathIDs {
                guard let vectorPath = document.paths[pathID] else { continue }
                let points = vectorPath.flattenedPoints(tolerance: 0.1)
                guard points.count >= 2 else { continue }

                var path = SwiftUI.Path()
                let firstScreen = worldToScreen(points[0], size: size)
                path.move(to: CGPoint(x: firstScreen.x, y: firstScreen.y))

                for i in 1..<points.count {
                    let screen = worldToScreen(points[i], size: size)
                    path.addLine(to: CGPoint(x: screen.x, y: screen.y))
                }

                if vectorPath.isClosed {
                    path.closeSubpath()
                }

                let isSelected = selectedPathIDs.contains(pathID)
                let color: Color = isSelected ? .accentColor : colorForLayer(layer.color)
                let lineWidth: CGFloat = isSelected ? 2.0 : 1.0

                context.stroke(path, with: .color(color), lineWidth: lineWidth)
            }
        }
    }

    // MARK: - Coordinate transforms

    private func worldToScreen(_ point: Vector2D, size: CGSize) -> CGPoint {
        let x = size.width / 2 + point.x * zoomLevel + panOffset.width
        let y = size.height / 2 - point.y * zoomLevel + panOffset.height // Flip Y
        return CGPoint(x: x, y: y)
    }

    // MARK: - Actions

    private func addLayer() {
        let index = document.layers.count + 1
        document.layers.append(DesignLayer(name: "Layer \(index)"))
    }

    private func createToolpath(type: ToolpathType) {
        let config = ToolpathConfig(
            name: "\(type.rawValue.capitalized) Toolpath",
            type: type,
            selectedPathIDs: Array(selectedPathIDs),
            toolID: document.toolDatabase.first?.id ?? UUID()
        )
        document.toolpathConfigs.append(config)
    }

    // MARK: - Helpers

    private func colorForLayer(_ color: LayerColor) -> Color {
        switch color {
        case .red: return .red
        case .orange: return .orange
        case .yellow: return .yellow
        case .green: return .green
        case .blue: return .blue
        case .purple: return .purple
        case .black: return .primary
        case .gray: return .gray
        case .white: return .white
        case .cyan: return .cyan
        case .magenta: return .pink
        }
    }

    private func iconForToolpathType(_ type: ToolpathType) -> String {
        switch type {
        case .profile: return "square.dashed"
        case .pocket: return "square.fill"
        case .vCarve: return "v.square"
        case .drilling: return "circle.circle"
        case .fluting: return "line.diagonal"
        case .texture: return "square.grid.3x3"
        case .prism: return "triangle"
        case .photoVCarve: return "photo"
        case .engraving: return "pencil.tip"
        case .threadMill: return "screw"
        case .moulding: return "cylinder"
        case .roughing3D: return "cube"
        case .finishing3D: return "cube.fill"
        }
    }

    private func propertyRow(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label)
                .font(.system(size: 11))
                .foregroundColor(.secondary)
            Spacer()
            Text(value)
                .font(.system(size: 11, design: .monospaced))
        }
    }

    private func fmt(_ v: Double) -> String {
        String(format: "%.1f", v)
    }
}

/// Design tools available in the canvas toolbar.
public enum DesignTool: String, CaseIterable, Sendable {
    case select
    case pan
    case drawLine
    case drawRectangle
    case drawCircle
    case drawEllipse
    case drawPolygon
    case drawArc
    case drawText
    case node

    public var displayName: String {
        switch self {
        case .select: return "Select"
        case .pan: return "Pan"
        case .drawLine: return "Draw Line"
        case .drawRectangle: return "Draw Rectangle"
        case .drawCircle: return "Draw Circle"
        case .drawEllipse: return "Draw Ellipse"
        case .drawPolygon: return "Draw Polygon"
        case .drawArc: return "Draw Arc"
        case .drawText: return "Draw Text"
        case .node: return "Node Edit"
        }
    }

    public var iconName: String {
        switch self {
        case .select: return "arrow.up.left"
        case .pan: return "hand.raised"
        case .drawLine: return "line.diagonal"
        case .drawRectangle: return "rectangle"
        case .drawCircle: return "circle"
        case .drawEllipse: return "oval"
        case .drawPolygon: return "pentagon"
        case .drawArc: return "circle.bottomhalf.filled"
        case .drawText: return "textformat"
        case .node: return "point.topleft.down.to.point.bottomright.curvepath"
        }
    }
}
