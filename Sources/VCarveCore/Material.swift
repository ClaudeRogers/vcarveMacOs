import Foundation

/// Position of the Z-zero reference for the job.
public enum ZZeroPosition: String, Codable, Sendable {
    case materialSurface   // Z=0 at top of material
    case machineBed        // Z=0 at the machine bed (bottom of material)
}

/// XY datum (origin) position for the job.
public enum XYDatumPosition: String, Codable, Sendable {
    case center
    case bottomLeft
    case bottomRight
    case topLeft
    case topRight
}

/// Job type - single-sided, double-sided, or rotary.
public enum JobType: String, Codable, Sendable {
    case singleSided
    case doubleSided
    case rotary
}

/// The material/job setup, equivalent to VCarve's Job Setup dialog.
public struct MaterialSetup: Codable, Sendable {
    public var jobType: JobType
    public var unit: MeasurementUnit

    // Material dimensions
    public var width: Double           // X dimension
    public var height: Double          // Y dimension
    public var thickness: Double       // Z dimension (material thickness)

    // Z reference
    public var zZeroPosition: ZZeroPosition
    public var xyDatumPosition: XYDatumPosition

    // Machine clearances
    public var safeZHeight: Double     // Rapid Z above material
    public var homePosition: Vector3D  // Machine home/park position

    // Material appearance (for preview)
    public var materialColor: MaterialColor

    // Rotary axis settings (only used for rotary jobs)
    public var rotaryDiameter: Double
    public var rotaryLength: Double
    public var wrappedAxis: WrappedAxis

    public init(
        jobType: JobType = .singleSided,
        unit: MeasurementUnit = .millimeters,
        width: Double = 300,
        height: Double = 200,
        thickness: Double = 18,
        zZeroPosition: ZZeroPosition = .materialSurface,
        xyDatumPosition: XYDatumPosition = .bottomLeft,
        safeZHeight: Double = 5.0,
        homePosition: Vector3D = Vector3D(0, 0, 20),
        materialColor: MaterialColor = .wood,
        rotaryDiameter: Double = 75,
        rotaryLength: Double = 300,
        wrappedAxis: WrappedAxis = .xAxis
    ) {
        self.jobType = jobType
        self.unit = unit
        self.width = width
        self.height = height
        self.thickness = thickness
        self.zZeroPosition = zZeroPosition
        self.xyDatumPosition = xyDatumPosition
        self.safeZHeight = safeZHeight
        self.homePosition = homePosition
        self.materialColor = materialColor
        self.rotaryDiameter = rotaryDiameter
        self.rotaryLength = rotaryLength
        self.wrappedAxis = wrappedAxis
    }

    /// The material bounds as a 2D bounding box.
    public var bounds: BoundingBox2D {
        let origin: Vector2D
        switch xyDatumPosition {
        case .bottomLeft: origin = .zero
        case .bottomRight: origin = Vector2D(-width, 0)
        case .topLeft: origin = Vector2D(0, -height)
        case .topRight: origin = Vector2D(-width, -height)
        case .center: origin = Vector2D(-width / 2, -height / 2)
        }
        return BoundingBox2D(min: origin, max: origin + Vector2D(width, height))
    }
}

/// Preset material colors for 3D preview.
public enum MaterialColor: String, Codable, Sendable, CaseIterable {
    case wood
    case mdf
    case acrylic
    case aluminum
    case brass
    case hdpe
    case custom

    public var rgb: (r: Double, g: Double, b: Double) {
        switch self {
        case .wood: return (0.82, 0.68, 0.47)
        case .mdf: return (0.76, 0.65, 0.50)
        case .acrylic: return (0.9, 0.9, 0.92)
        case .aluminum: return (0.77, 0.79, 0.82)
        case .brass: return (0.85, 0.75, 0.38)
        case .hdpe: return (0.95, 0.95, 0.95)
        case .custom: return (0.8, 0.8, 0.8)
        }
    }
}

/// Which axis is used for rotary wrapping.
public enum WrappedAxis: String, Codable, Sendable {
    case xAxis
    case yAxis
}
