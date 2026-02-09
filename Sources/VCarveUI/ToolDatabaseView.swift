import SwiftUI
import VCarveCore

/// Tool Database management view.
public struct ToolDatabaseView: View {
    @Binding var tools: [Tool]
    @Environment(\.dismiss) private var dismiss
    @State private var selectedToolID: UUID?

    public init(tools: Binding<[Tool]>) {
        self._tools = tools
    }

    public var body: some View {
        HSplitView {
            // Tool list
            VStack(alignment: .leading) {
                Text("Tools")
                    .font(.headline)
                    .padding(.horizontal)
                    .padding(.top, 8)

                List(tools, selection: $selectedToolID) { tool in
                    HStack {
                        Image(systemName: iconForToolType(tool.type))
                            .foregroundColor(.accentColor)
                        VStack(alignment: .leading) {
                            Text(tool.name)
                                .font(.system(size: 12))
                            Text("\(String(format: "%.1f", tool.diameter))\(tool.unit.abbreviation) \(tool.type.rawValue)")
                                .font(.system(size: 10))
                                .foregroundColor(.secondary)
                        }
                    }
                    .tag(tool.id)
                }
                .listStyle(.sidebar)

                HStack {
                    Button(action: addTool) {
                        Image(systemName: "plus")
                    }
                    Button(action: duplicateTool) {
                        Image(systemName: "doc.on.doc")
                    }
                    Button(action: deleteTool) {
                        Image(systemName: "minus")
                    }
                    .disabled(selectedToolID == nil)
                }
                .padding(8)
            }
            .frame(minWidth: 200, idealWidth: 250)

            // Tool editor
            if let toolID = selectedToolID,
               let toolIndex = tools.firstIndex(where: { $0.id == toolID }) {
                ToolEditorView(tool: $tools[toolIndex])
            } else {
                VStack {
                    Spacer()
                    Text("Select a tool to edit")
                        .foregroundColor(.secondary)
                    Spacer()
                }
            }
        }
        .frame(width: 700, height: 500)
        .toolbar {
            ToolbarItem(placement: .confirmationAction) {
                Button("Done") { dismiss() }
            }
        }
    }

    private func addTool() {
        let tool = Tool(name: "New Tool")
        tools.append(tool)
        selectedToolID = tool.id
    }

    private func duplicateTool() {
        guard let id = selectedToolID,
              let tool = tools.first(where: { $0.id == id }) else { return }
        var copy = tool
        copy = Tool(
            name: tool.name + " (Copy)",
            type: tool.type,
            unit: tool.unit,
            diameter: tool.diameter,
            fluteLength: tool.fluteLength,
            shankDiameter: tool.shankDiameter,
            overallLength: tool.overallLength,
            tipAngle: tool.tipAngle,
            cornerRadius: tool.cornerRadius,
            numberOfFlutes: tool.numberOfFlutes,
            spindleSpeed: tool.spindleSpeed,
            feedRate: tool.feedRate,
            plungeRate: tool.plungeRate,
            depthPerPass: tool.depthPerPass,
            stepover: tool.stepover
        )
        tools.append(copy)
        selectedToolID = copy.id
    }

    private func deleteTool() {
        guard let id = selectedToolID else { return }
        tools.removeAll { $0.id == id }
        selectedToolID = tools.first?.id
    }

    private func iconForToolType(_ type: ToolType) -> String {
        switch type {
        case .endMill: return "cylinder"
        case .ballNose: return "capsule"
        case .vBit: return "arrowtriangle.down"
        case .bullNose: return "cylinder"
        case .engraving: return "pencil.tip"
        case .drill: return "arrow.down.circle"
        case .taperedBallNose: return "capsule"
        case .chamfer: return "triangle"
        case .threadMill: return "screw"
        case .custom: return "gearshape"
        }
    }
}

/// Tool parameter editor.
struct ToolEditorView: View {
    @Binding var tool: Tool

    var body: some View {
        Form {
            Section("General") {
                TextField("Name", text: $tool.name)
                Picker("Type", selection: $tool.type) {
                    ForEach(ToolType.allCases, id: \.self) { type in
                        Text(type.rawValue.capitalized).tag(type)
                    }
                }
                Picker("Units", selection: $tool.unit) {
                    Text("Millimeters").tag(MeasurementUnit.millimeters)
                    Text("Inches").tag(MeasurementUnit.inches)
                }
            }

            Section("Geometry") {
                numberField("Diameter", value: $tool.diameter, unit: tool.unit.abbreviation)
                numberField("Flute Length", value: $tool.fluteLength, unit: tool.unit.abbreviation)
                numberField("Shank Diameter", value: $tool.shankDiameter, unit: tool.unit.abbreviation)
                numberField("Overall Length", value: $tool.overallLength, unit: tool.unit.abbreviation)

                if tool.type == .vBit || tool.type == .chamfer || tool.type == .engraving {
                    numberField("Tip Angle", value: $tool.tipAngle, unit: "°")
                }

                if tool.type == .bullNose {
                    numberField("Corner Radius", value: $tool.cornerRadius, unit: tool.unit.abbreviation)
                }

                Stepper("Flutes: \(tool.numberOfFlutes)", value: $tool.numberOfFlutes, in: 1...8)
            }

            Section("Cutting Parameters") {
                numberField("Spindle Speed", value: $tool.spindleSpeed, unit: "RPM")
                numberField("Feed Rate", value: $tool.feedRate, unit: tool.unit.perMinuteAbbreviation)
                numberField("Plunge Rate", value: $tool.plungeRate, unit: tool.unit.perMinuteAbbreviation)
                numberField("Depth per Pass", value: $tool.depthPerPass, unit: tool.unit.abbreviation)
                numberField("Stepover", value: $tool.stepover, unit: "× dia")
            }
        }
        .formStyle(.grouped)
        .padding()
    }

    private func numberField(_ label: String, value: Binding<Double>, unit: String) -> some View {
        HStack {
            Text(label)
            Spacer()
            TextField(label, value: value, format: .number)
                .textFieldStyle(.roundedBorder)
                .frame(width: 100)
            Text(unit)
                .font(.caption)
                .foregroundColor(.secondary)
                .frame(width: 50, alignment: .leading)
        }
    }
}
