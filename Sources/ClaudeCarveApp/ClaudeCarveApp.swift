import SwiftUI
import ClaudeCarveUI
import ClaudeCarveCore

@main
struct ClaudeCarveApp: App {
    var body: some Scene {
        DocumentGroup(newDocument: ClaudeCarveAppDocument()) { file in
            ClaudeCarveDocumentView(document: file.$document.document)
        }
        .commands {
            // File menu additions
            CommandGroup(after: .importExport) {
                Button("Import SVG...") {
                    importFile(type: "svg")
                }
                .keyboardShortcut("i", modifiers: [.command, .shift])

                Button("Import DXF...") {
                    importFile(type: "dxf")
                }

                Divider()

                Button("Export G-Code...") {
                    exportGCode()
                }
                .keyboardShortcut("e", modifiers: [.command, .shift])
            }

            // View menu
            CommandGroup(after: .toolbar) {
                Button("Zoom In") {}
                    .keyboardShortcut("+", modifiers: .command)
                Button("Zoom Out") {}
                    .keyboardShortcut("-", modifiers: .command)
                Button("Fit to Window") {}
                    .keyboardShortcut("0", modifiers: .command)
                Divider()
                Button("3D Preview") {}
                    .keyboardShortcut("3", modifiers: .command)
            }

            // Toolpath menu
            CommandMenu("Toolpath") {
                Button("Create Profile Toolpath...") {}
                    .keyboardShortcut("p", modifiers: [.command, .shift])
                Button("Create Pocket Toolpath...") {}
                    .keyboardShortcut("k", modifiers: [.command, .shift])
                Button("Create V-Carve Toolpath...") {}
                    .keyboardShortcut("v", modifiers: [.command, .shift])
                Button("Create Drilling Toolpath...") {}
                    .keyboardShortcut("d", modifiers: [.command, .shift])
                Divider()
                Button("Calculate All Toolpaths") {}
                    .keyboardShortcut("a", modifiers: [.command, .shift])
                Button("Preview All Toolpaths") {}
            }

            // Tools menu
            CommandMenu("CNC Tools") {
                Button("Tool Database...") {}
                    .keyboardShortcut("t", modifiers: [.command, .shift])
                Button("Material Setup...") {}
                    .keyboardShortcut("j", modifiers: [.command, .shift])
                Divider()
                Button("Post Processor Settings...") {}
            }
        }

        // Settings window
        Settings {
            SettingsView()
        }
    }

    private func importFile(type: String) {
        // File import will be handled by the document view
    }

    private func exportGCode() {
        // G-code export will be handled by the document view
    }
}

/// Application settings view.
struct SettingsView: View {
    @AppStorage("defaultUnit") private var defaultUnit: String = "millimeters"
    @AppStorage("defaultPostProcessor") private var defaultPostProcessor: String = "grbl"
    @AppStorage("snapToGrid") private var snapToGrid: Bool = true
    @AppStorage("gridSpacing") private var gridSpacing: Double = 10.0

    var body: some View {
        TabView {
            // General settings
            Form {
                Picker("Default Units", selection: $defaultUnit) {
                    Text("Millimeters").tag("millimeters")
                    Text("Inches").tag("inches")
                }

                Picker("Default Post Processor", selection: $defaultPostProcessor) {
                    Text("GRBL").tag("grbl")
                    Text("Mach3").tag("mach3")
                    Text("LinuxCNC").tag("linuxCNC")
                    Text("Generic G-Code").tag("generic")
                }

                Toggle("Snap to Grid", isOn: $snapToGrid)
                TextField("Grid Spacing", value: $gridSpacing, format: .number)
            }
            .formStyle(.grouped)
            .tabItem {
                Label("General", systemImage: "gear")
            }

            // Machine settings
            Form {
                Text("Machine-specific settings will appear here.")
                    .foregroundColor(.secondary)
            }
            .formStyle(.grouped)
            .tabItem {
                Label("Machine", systemImage: "desktopcomputer")
            }
        }
        .frame(width: 450, height: 300)
    }
}
