import SwiftUI
import ClaudeCarveCore
import ClaudeCarveGCode
import ClaudeCarveToolpath

/// G-Code export dialog — select toolpaths, post-processor, and save.
public struct GCodeExportView: View {
    let document: ClaudeCarveDocument
    @State private var selectedPostProcessor: PostProcessor = .grbl
    @State private var outputUnit: MeasurementUnit = .millimeters
    @State private var includeComments: Bool = true
    @State private var useLineNumbers: Bool = false
    @State private var selectedToolpathIndices: Set<Int> = []
    @State private var previewGCode: String = ""
    @State private var showPreview: Bool = false
    @Environment(\.dismiss) private var dismiss

    public init(document: ClaudeCarveDocument) {
        self.document = document
    }

    public var body: some View {
        VStack(spacing: 16) {
            Text("Save Toolpaths")
                .font(.title2)
                .fontWeight(.semibold)

            HSplitView {
                // Left: Toolpath selection
                VStack(alignment: .leading) {
                    Text("Toolpaths to Export")
                        .font(.headline)

                    List(document.toolpathConfigs.indices, id: \.self, selection: $selectedToolpathIndices) { index in
                        let config = document.toolpathConfigs[index]
                        HStack {
                            Image(systemName: config.isCalculated ? "checkmark.circle.fill" : "circle")
                                .foregroundColor(config.isCalculated ? .green : .gray)
                            VStack(alignment: .leading) {
                                Text(config.name)
                                    .font(.system(size: 12))
                                Text(config.type.rawValue.capitalized)
                                    .font(.system(size: 10))
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                    .listStyle(.bordered)
                }
                .frame(minWidth: 200)

                // Right: Export settings
                Form {
                    Section("Post Processor") {
                        Picker("Machine", selection: $selectedPostProcessor) {
                            ForEach(PostProcessor.allCases, id: \.self) { pp in
                                Text(pp.displayName).tag(pp)
                            }
                        }

                        HStack {
                            Text("Output file extension:")
                            Text(".\(selectedPostProcessor.fileExtension)")
                                .fontWeight(.medium)
                                .foregroundColor(.accentColor)
                        }
                    }

                    Section("Units") {
                        Picker("Output Units", selection: $outputUnit) {
                            Text("Millimeters (G21)").tag(MeasurementUnit.millimeters)
                            Text("Inches (G20)").tag(MeasurementUnit.inches)
                        }
                        .pickerStyle(.radioGroup)
                    }

                    Section("Options") {
                        Toggle("Include comments", isOn: $includeComments)
                        Toggle("Line numbers", isOn: $useLineNumbers)
                    }

                    Section("Preview") {
                        Button("Generate Preview") {
                            generatePreview()
                        }

                        if showPreview {
                            ScrollView {
                                Text(previewGCode)
                                    .font(.system(size: 10, design: .monospaced))
                                    .textSelection(.enabled)
                                    .frame(maxWidth: .infinity, alignment: .leading)
                            }
                            .frame(height: 200)
                            .background(Color(nsColor: .textBackgroundColor))
                            .border(Color.gray.opacity(0.3))
                        }
                    }
                }
                .formStyle(.grouped)
            }

            HStack {
                Button("Cancel") { dismiss() }
                    .keyboardShortcut(.cancelAction)
                Spacer()

                Button("Copy to Clipboard") {
                    generatePreview()
                    NSPasteboard.general.clearContents()
                    NSPasteboard.general.setString(previewGCode, forType: .string)
                }

                Button("Save G-Code...") {
                    saveGCode()
                }
                .keyboardShortcut(.defaultAction)
                .buttonStyle(.borderedProminent)
            }
            .padding(.horizontal)
            .padding(.bottom)
        }
        .frame(width: 700, height: 550)
        .onAppear {
            outputUnit = document.materialSetup.unit
            selectedToolpathIndices = Set(document.toolpathConfigs.indices)
        }
    }

    private func generatePreview() {
        let config = GCodeGenerator.OutputConfig(
            postProcessor: selectedPostProcessor,
            unit: outputUnit,
            lineNumbers: useLineNumbers,
            includeComments: includeComments,
            fileExtension: selectedPostProcessor.fileExtension
        )

        let generator = GCodeGenerator(config: config)

        // For preview, generate a sample toolpath
        if let firstConfig = document.toolpathConfigs.first,
           let tool = document.toolDatabase.first {
            let paths = firstConfig.selectedPathIDs.compactMap { document.paths[$0] }
            let computed: ComputedToolpath

            switch firstConfig.type {
            case .profile:
                computed = ProfileToolpathGenerator.generate(
                    paths: paths, config: firstConfig, tool: tool, material: document.materialSetup
                )
            case .pocket:
                computed = PocketToolpathGenerator.generate(
                    paths: paths, config: firstConfig, tool: tool, material: document.materialSetup
                )
            case .vCarve:
                computed = VCarveToolpathGenerator.generate(
                    paths: paths, config: firstConfig, tool: tool, material: document.materialSetup
                )
            case .drilling:
                computed = DrillToolpathGenerator.generate(
                    paths: paths, config: firstConfig, tool: tool, material: document.materialSetup
                )
            default:
                computed = ComputedToolpath(configID: firstConfig.id, toolID: tool.id)
            }

            previewGCode = generator.generate(
                toolpath: computed, tool: tool, material: document.materialSetup
            )
        } else {
            previewGCode = "; No toolpaths to export\n"
        }

        showPreview = true
    }

    private func saveGCode() {
        generatePreview()

        let panel = NSSavePanel()
        panel.nameFieldStringValue = "toolpath.\(selectedPostProcessor.fileExtension)"
        panel.allowedContentTypes = [.plainText]
        panel.canCreateDirectories = true

        panel.begin { response in
            if response == .OK, let url = panel.url {
                try? previewGCode.write(to: url, atomically: true, encoding: .utf8)
            }
        }
    }
}
