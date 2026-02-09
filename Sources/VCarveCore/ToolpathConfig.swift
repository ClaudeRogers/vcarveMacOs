import Foundation

/// The type of toolpath operation.
public enum ToolpathType: String, Codable, Sendable, CaseIterable {
    case profile        // Cut along/around vectors
    case pocket         // Clear interior area
    case vCarve         // V-carving with variable depth
    case drilling       // Drill holes
    case fluting        // Tapered groove cutting
    case texture        // Textured surface pattern
    case prism          // Raised prism/chamfer
    case photoVCarve    // Photo to V-carved depth map
    case engraving      // Quick engrave / drag engraving
    case threadMill     // Thread milling
    case moulding       // 3D moulding along vectors
    case roughing3D     // 3D roughing pass
    case finishing3D    // 3D finishing pass
}

/// Profile cut side — inside, outside, or on the vector.
public enum ProfileSide: String, Codable, Sendable {
    case inside
    case outside
    case onLine
}

/// Profile machining direction.
public enum MachineDirection: String, Codable, Sendable {
    case climb      // Climb milling (preferred for CNC)
    case conventional
}

/// How the tool enters the cut.
public enum RampType: String, Codable, Sendable {
    case straight   // Straight plunge
    case ramp       // Linear ramp entry
    case helical    // Helical/spiral entry
    case profile    // Ramp along the profile path
}

/// Lead-in/lead-out arc configuration.
public struct LeadInOut: Codable, Sendable {
    public var enabled: Bool
    public var radius: Double
    public var angle: Double  // degrees
    public var overcut: Double

    public init(enabled: Bool = true, radius: Double = 2.0, angle: Double = 90, overcut: Double = 0) {
        self.enabled = enabled
        self.radius = radius
        self.angle = angle
        self.overcut = overcut
    }
}

/// Pocket clearing strategy.
public enum PocketStrategy: String, Codable, Sendable {
    case offset     // Concentric offset passes
    case raster     // Back and forth parallel lines
    case spiral     // Spiral from outside in
}

/// A toolpath configuration that fully describes a machining operation.
public struct ToolpathConfig: Identifiable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var type: ToolpathType
    public var isVisible: Bool
    public var isCalculated: Bool

    // Target vectors
    public var selectedPathIDs: [UUID]

    // Tool selection
    public var toolID: UUID
    public var secondaryToolID: UUID?  // For flat-bottom V-carve clearing

    // Depth settings
    public var startDepth: Double      // Depth from which cutting begins
    public var cutDepth: Double        // Total depth of cut
    public var useModelSurface: Bool   // Project onto 3D model surface

    // Profile-specific
    public var profileSide: ProfileSide
    public var machineDirection: MachineDirection
    public var allowanceOffset: Double // Finishing allowance
    public var addTabs: Bool
    public var tabWidth: Double
    public var tabHeight: Double
    public var tabSpacing: Double
    public var separateLastPass: Bool
    public var lastPassAllowance: Double

    // Entry/exit
    public var rampType: RampType
    public var rampDistance: Double
    public var rampAngle: Double       // degrees
    public var leadIn: LeadInOut
    public var leadOut: LeadInOut

    // Pocket-specific
    public var pocketStrategy: PocketStrategy
    public var pocketStepover: Double  // Fraction of tool diameter

    // V-Carve specific
    public var flatBottomDepth: Double // For flat-bottom V-carving (0 = standard)
    public var useFlatBottomTool: Bool

    // Drilling specific
    public var peckDrilling: Bool
    public var peckDepth: Double
    public var retractHeight: Double
    public var dwellTime: Double       // seconds

    // Fluting specific
    public var rampInLength: Double
    public var rampOutLength: Double

    public init(
        id: UUID = UUID(),
        name: String = "Toolpath",
        type: ToolpathType = .profile,
        isVisible: Bool = true,
        isCalculated: Bool = false,
        selectedPathIDs: [UUID] = [],
        toolID: UUID = UUID(),
        secondaryToolID: UUID? = nil,
        startDepth: Double = 0,
        cutDepth: Double = 5.0,
        useModelSurface: Bool = false,
        profileSide: ProfileSide = .outside,
        machineDirection: MachineDirection = .climb,
        allowanceOffset: Double = 0,
        addTabs: Bool = false,
        tabWidth: Double = 6.0,
        tabHeight: Double = 2.0,
        tabSpacing: Double = 50.0,
        separateLastPass: Bool = false,
        lastPassAllowance: Double = 0.5,
        rampType: RampType = .straight,
        rampDistance: Double = 10.0,
        rampAngle: Double = 10.0,
        leadIn: LeadInOut = LeadInOut(),
        leadOut: LeadInOut = LeadInOut(),
        pocketStrategy: PocketStrategy = .offset,
        pocketStepover: Double = 0.4,
        flatBottomDepth: Double = 0,
        useFlatBottomTool: Bool = false,
        peckDrilling: Bool = false,
        peckDepth: Double = 3.0,
        retractHeight: Double = 2.0,
        dwellTime: Double = 0,
        rampInLength: Double = 10.0,
        rampOutLength: Double = 10.0
    ) {
        self.id = id
        self.name = name
        self.type = type
        self.isVisible = isVisible
        self.isCalculated = isCalculated
        self.selectedPathIDs = selectedPathIDs
        self.toolID = toolID
        self.secondaryToolID = secondaryToolID
        self.startDepth = startDepth
        self.cutDepth = cutDepth
        self.useModelSurface = useModelSurface
        self.profileSide = profileSide
        self.machineDirection = machineDirection
        self.allowanceOffset = allowanceOffset
        self.addTabs = addTabs
        self.tabWidth = tabWidth
        self.tabHeight = tabHeight
        self.tabSpacing = tabSpacing
        self.separateLastPass = separateLastPass
        self.lastPassAllowance = lastPassAllowance
        self.rampType = rampType
        self.rampDistance = rampDistance
        self.rampAngle = rampAngle
        self.leadIn = leadIn
        self.leadOut = leadOut
        self.pocketStrategy = pocketStrategy
        self.pocketStepover = pocketStepover
        self.flatBottomDepth = flatBottomDepth
        self.useFlatBottomTool = useFlatBottomTool
        self.peckDrilling = peckDrilling
        self.peckDepth = peckDepth
        self.retractHeight = retractHeight
        self.dwellTime = dwellTime
        self.rampInLength = rampInLength
        self.rampOutLength = rampOutLength
    }
}
