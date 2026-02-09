import Foundation

/// A layer in the design, grouping related vectors.
public struct DesignLayer: Identifiable, Codable, Sendable {
    public let id: UUID
    public var name: String
    public var isVisible: Bool
    public var isLocked: Bool
    public var color: LayerColor
    public var pathIDs: [UUID]

    public init(
        id: UUID = UUID(),
        name: String = "Layer 1",
        isVisible: Bool = true,
        isLocked: Bool = false,
        color: LayerColor = .blue,
        pathIDs: [UUID] = []
    ) {
        self.id = id
        self.name = name
        self.isVisible = isVisible
        self.isLocked = isLocked
        self.color = color
        self.pathIDs = pathIDs
    }
}

/// Simple color presets for layers.
public enum LayerColor: String, Codable, Sendable, CaseIterable {
    case red, orange, yellow, green, blue, purple, black, gray, white, cyan, magenta
}

/// A VCarve document containing all design data, toolpaths, and settings.
public struct VCarveDocument: Codable, Sendable {
    public var materialSetup: MaterialSetup
    public var layers: [DesignLayer]
    public var paths: [UUID: VectorPath]
    public var toolDatabase: [Tool]
    public var toolpathConfigs: [ToolpathConfig]

    public init(
        materialSetup: MaterialSetup = MaterialSetup(),
        layers: [DesignLayer] = [DesignLayer()],
        paths: [UUID: VectorPath] = [:],
        toolDatabase: [Tool] = defaultTools(),
        toolpathConfigs: [ToolpathConfig] = []
    ) {
        self.materialSetup = materialSetup
        self.layers = layers
        self.paths = paths
        self.toolDatabase = toolDatabase
        self.toolpathConfigs = toolpathConfigs
    }

    /// Add a path to the document on the specified layer.
    public mutating func addPath(_ path: VectorPath, toLayer layerIndex: Int = 0) {
        paths[path.id] = path
        if layers.indices.contains(layerIndex) {
            layers[layerIndex].pathIDs.append(path.id)
        }
    }

    /// Get all paths on a given layer.
    public func pathsOnLayer(_ layerIndex: Int) -> [VectorPath] {
        guard layers.indices.contains(layerIndex) else { return [] }
        return layers[layerIndex].pathIDs.compactMap { paths[$0] }
    }

    /// All paths in the document.
    public var allPaths: [VectorPath] {
        Array(paths.values)
    }

    /// Default tool set for new documents.
    public static func defaultTools() -> [Tool] {
        [
            .endMill6mm(),
            .endMill3mm(),
            .ballNose6mm(),
            .vBit60(),
            .vBit90(),
        ]
    }
}
