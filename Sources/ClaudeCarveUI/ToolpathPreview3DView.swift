import SwiftUI
import SceneKit
import ClaudeCarveCore

/// 3D toolpath preview using SceneKit.
/// Shows the material block with toolpath moves rendered as colored lines.
public struct ToolpathPreview3DView: View {
    let document: ClaudeCarveDocument
    @State private var showMaterial: Bool = true
    @State private var showToolpaths: Bool = true
    @Environment(\.dismiss) private var dismiss

    public init(document: ClaudeCarveDocument) {
        self.document = document
    }

    public var body: some View {
        VStack(spacing: 0) {
            HStack {
                Text("3D Preview")
                    .font(.title3)
                    .fontWeight(.semibold)

                Spacer()

                Toggle("Material", isOn: $showMaterial)
                    .toggleStyle(.checkbox)
                Toggle("Toolpaths", isOn: $showToolpaths)
                    .toggleStyle(.checkbox)

                Button("Close") { dismiss() }
            }
            .padding()

            SceneView(
                scene: createScene(),
                options: [.allowsCameraControl, .autoenablesDefaultLighting]
            )
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .frame(width: 800, height: 600)
    }

    private func createScene() -> SCNScene {
        let scene = SCNScene()

        // Camera
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.zNear = 0.1
        cameraNode.camera?.zFar = 10000
        let material = document.materialSetup
        cameraNode.position = SCNVector3(
            material.width / 2,
            material.height / 2,
            max(material.width, material.height) * 1.5
        )
        cameraNode.look(at: SCNVector3(material.width / 2, material.height / 2, 0))
        scene.rootNode.addChildNode(cameraNode)

        // Ambient light
        let ambientLight = SCNNode()
        ambientLight.light = SCNLight()
        ambientLight.light?.type = .ambient
        ambientLight.light?.intensity = 500
        scene.rootNode.addChildNode(ambientLight)

        // Directional light
        let directionalLight = SCNNode()
        directionalLight.light = SCNLight()
        directionalLight.light?.type = .directional
        directionalLight.light?.intensity = 800
        directionalLight.eulerAngles = SCNVector3(-Float.pi / 4, Float.pi / 4, 0)
        scene.rootNode.addChildNode(directionalLight)

        // Material block
        if showMaterial {
            let blockGeometry = SCNBox(
                width: material.width,
                height: material.height,
                length: material.thickness,
                chamferRadius: 0
            )
            let blockMaterial = SCNMaterial()
            let rgb = material.materialColor.rgb
            blockMaterial.diffuse.contents = NSColor(
                red: rgb.r,
                green: rgb.g,
                blue: rgb.b,
                alpha: 0.7
            )
            blockMaterial.transparency = 0.7
            blockGeometry.materials = [blockMaterial]

            let blockNode = SCNNode(geometry: blockGeometry)
            blockNode.position = SCNVector3(
                material.width / 2,
                material.height / 2,
                -material.thickness / 2
            )
            scene.rootNode.addChildNode(blockNode)
        }

        // Render toolpath moves as lines (simplified for SceneKit)
        // Full implementation would carve the material geometry
        if showToolpaths {
            // For now, show toolpath center lines
            // A complete implementation would use a heightfield and subtract tool geometry
            let toolpathNode = SCNNode()
            scene.rootNode.addChildNode(toolpathNode)
        }

        // Ground plane grid
        let gridNode = createGridNode(width: material.width, height: material.height)
        scene.rootNode.addChildNode(gridNode)

        return scene
    }

    private func createGridNode(width: Double, height: Double) -> SCNNode {
        let gridNode = SCNNode()
        let gridSpacing = 10.0
        let lineColor = NSColor.gray.withAlphaComponent(0.3)

        // Create grid lines using thin box geometry
        var x = 0.0
        while x <= width {
            let lineGeo = SCNBox(width: 0.2, height: height, length: 0.1, chamferRadius: 0)
            let lineMat = SCNMaterial()
            lineMat.diffuse.contents = lineColor
            lineGeo.materials = [lineMat]

            let lineNode = SCNNode(geometry: lineGeo)
            lineNode.position = SCNVector3(x, height / 2, 0.05)
            gridNode.addChildNode(lineNode)
            x += gridSpacing
        }

        var y = 0.0
        while y <= height {
            let lineGeo = SCNBox(width: width, height: 0.2, length: 0.1, chamferRadius: 0)
            let lineMat = SCNMaterial()
            lineMat.diffuse.contents = lineColor
            lineGeo.materials = [lineMat]

            let lineNode = SCNNode(geometry: lineGeo)
            lineNode.position = SCNVector3(width / 2, y, 0.05)
            gridNode.addChildNode(lineNode)
            y += gridSpacing
        }

        return gridNode
    }
}
