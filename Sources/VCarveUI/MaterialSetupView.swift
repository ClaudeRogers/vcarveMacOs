import SwiftUI
import VCarveCore

/// Material/Job Setup dialog — equivalent to VCarve's Job Setup.
public struct MaterialSetupView: View {
    @Binding var setup: MaterialSetup
    @Environment(\.dismiss) private var dismiss

    public init(setup: Binding<MaterialSetup>) {
        self._setup = setup
    }

    public var body: some View {
        VStack(spacing: 16) {
            Text("Job Setup")
                .font(.title2)
                .fontWeight(.semibold)

            Form {
                // Job type
                Section("Job Type") {
                    Picker("Type", selection: $setup.jobType) {
                        Text("Single Sided").tag(JobType.singleSided)
                        Text("Double Sided").tag(JobType.doubleSided)
                        Text("Rotary").tag(JobType.rotary)
                    }
                    .pickerStyle(.segmented)
                }

                // Units
                Section("Units") {
                    Picker("Measurement", selection: $setup.unit) {
                        Text("Millimeters (mm)").tag(MeasurementUnit.millimeters)
                        Text("Inches (in)").tag(MeasurementUnit.inches)
                    }
                }

                // Material dimensions
                Section("Material Size") {
                    HStack {
                        VStack(alignment: .leading) {
                            Text("Width (X)")
                                .font(.caption)
                            TextField("Width", value: $setup.width, format: .number)
                                .textFieldStyle(.roundedBorder)
                        }
                        VStack(alignment: .leading) {
                            Text("Height (Y)")
                                .font(.caption)
                            TextField("Height", value: $setup.height, format: .number)
                                .textFieldStyle(.roundedBorder)
                        }
                    }

                    VStack(alignment: .leading) {
                        Text("Thickness (Z)")
                            .font(.caption)
                        TextField("Thickness", value: $setup.thickness, format: .number)
                            .textFieldStyle(.roundedBorder)
                    }
                }

                // Z-Zero position
                Section("Z Zero Position") {
                    Picker("Z Zero", selection: $setup.zZeroPosition) {
                        Text("Material Surface").tag(ZZeroPosition.materialSurface)
                        Text("Machine Bed").tag(ZZeroPosition.machineBed)
                    }
                    .pickerStyle(.radioGroup)
                }

                // XY Datum
                Section("XY Datum Position") {
                    Picker("XY Datum", selection: $setup.xyDatumPosition) {
                        Text("Center").tag(XYDatumPosition.center)
                        Text("Bottom Left").tag(XYDatumPosition.bottomLeft)
                        Text("Bottom Right").tag(XYDatumPosition.bottomRight)
                        Text("Top Left").tag(XYDatumPosition.topLeft)
                        Text("Top Right").tag(XYDatumPosition.topRight)
                    }
                }

                // Clearances
                Section("Clearances") {
                    VStack(alignment: .leading) {
                        Text("Safe Z Height")
                            .font(.caption)
                        TextField("Safe Z", value: $setup.safeZHeight, format: .number)
                            .textFieldStyle(.roundedBorder)
                    }
                }

                // Material appearance
                Section("Appearance") {
                    Picker("Material Color", selection: $setup.materialColor) {
                        ForEach(MaterialColor.allCases, id: \.self) { color in
                            Text(color.rawValue.capitalized).tag(color)
                        }
                    }
                }
            }
            .formStyle(.grouped)

            HStack {
                Button("Cancel") { dismiss() }
                    .keyboardShortcut(.cancelAction)
                Spacer()
                Button("OK") { dismiss() }
                    .keyboardShortcut(.defaultAction)
                    .buttonStyle(.borderedProminent)
            }
            .padding(.horizontal)
            .padding(.bottom)
        }
        .frame(width: 480, height: 600)
    }
}
