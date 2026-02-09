import Foundation
import ClaudeCarveCore

// MARK: - Triangle3D

/// A triangle in 3D space.
public struct Triangle3D: Sendable {
    public let v0: Vector3D
    public let v1: Vector3D
    public let v2: Vector3D
    public let normal: Vector3D

    public init(v0: Vector3D, v1: Vector3D, v2: Vector3D, normal: Vector3D) {
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.normal = normal
    }

    /// Initialize with auto-computed normal via cross product.
    public init(v0: Vector3D, v1: Vector3D, v2: Vector3D) {
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        let edge1 = v1 - v0
        let edge2 = v2 - v0
        self.normal = edge1.cross(edge2).normalized
    }
}

// MARK: - Mesh3D

/// A 3D mesh imported from STL or OBJ.
public struct Mesh3D: Sendable {
    public let triangles: [Triangle3D]
    public let boundingBox: BoundingBox3D
    public let vertexCount: Int
    public let triangleCount: Int

    public init(triangles: [Triangle3D]) {
        self.triangles = triangles
        self.triangleCount = triangles.count
        self.vertexCount = triangles.count * 3

        var bb = BoundingBox3D()
        for tri in triangles {
            bb.expand(toInclude: tri.v0)
            bb.expand(toInclude: tri.v1)
            bb.expand(toInclude: tri.v2)
        }
        self.boundingBox = bb
    }

    /// Scale the mesh to fit within the specified dimensions, maintaining aspect ratio.
    public func scaledToFit(width: Double, height: Double, depth: Double) -> Mesh3D {
        let bbWidth = boundingBox.width
        let bbHeight = boundingBox.height
        let bbDepth = boundingBox.depth

        guard bbWidth > .ulpOfOne && bbHeight > .ulpOfOne && bbDepth > .ulpOfOne else {
            return self
        }

        let scaleX = width / bbWidth
        let scaleY = height / bbHeight
        let scaleZ = depth / bbDepth
        let uniformScale = Swift.min(scaleX, Swift.min(scaleY, scaleZ))

        let center = boundingBox.center
        let scaledTriangles = triangles.map { tri in
            Triangle3D(
                v0: (tri.v0 - center) * uniformScale,
                v1: (tri.v1 - center) * uniformScale,
                v2: (tri.v2 - center) * uniformScale,
                normal: tri.normal
            )
        }
        return Mesh3D(triangles: scaledTriangles)
    }

    /// Center the mesh at the origin.
    public func centered() -> Mesh3D {
        let center = boundingBox.center
        let centeredTriangles = triangles.map { tri in
            Triangle3D(
                v0: tri.v0 - center,
                v1: tri.v1 - center,
                v2: tri.v2 - center,
                normal: tri.normal
            )
        }
        return Mesh3D(triangles: centeredTriangles)
    }

    /// Generate a heightmap (2D depth image) by projecting rays downward along the Z axis.
    ///
    /// Each cell contains the maximum Z value of any triangle intersection at that XY position.
    /// Cells with no intersection contain `Double.nan`.
    public func toHeightmap(resolutionX: Int, resolutionY: Int) -> [[Double]] {
        guard resolutionX > 0 && resolutionY > 0 else { return [] }

        let bbMin = boundingBox.min
        let bbMax = boundingBox.max
        let cellWidth = (bbMax.x - bbMin.x) / Double(resolutionX)
        let cellHeight = (bbMax.y - bbMin.y) / Double(resolutionY)

        var heightmap = [[Double]](
            repeating: [Double](repeating: Double.nan, count: resolutionX),
            count: resolutionY
        )

        for row in 0..<resolutionY {
            let rayY = bbMin.y + (Double(row) + 0.5) * cellHeight
            for col in 0..<resolutionX {
                let rayX = bbMin.x + (Double(col) + 0.5) * cellWidth

                var maxZ = -Double.infinity
                var hit = false

                for tri in triangles {
                    if let z = ModelImporter.rayTriangleIntersectionZ(
                        rayX: rayX,
                        rayY: rayY,
                        v0: tri.v0,
                        v1: tri.v1,
                        v2: tri.v2
                    ) {
                        if z > maxZ {
                            maxZ = z
                            hit = true
                        }
                    }
                }

                if hit {
                    heightmap[row][col] = maxZ
                }
            }
        }

        return heightmap
    }
}

// MARK: - ModelFormat

/// Supported 3D model file formats.
public enum ModelFormat: Sendable {
    case stlBinary
    case stlASCII
    case obj
}

// MARK: - ModelImporter

/// Imports 3D models from STL (binary and ASCII) and OBJ file formats.
public struct ModelImporter {

    // MARK: - Public API

    /// Import a 3D model from file data with an explicit format.
    public static func importModel(data: Data, format: ModelFormat) -> Mesh3D? {
        switch format {
        case .stlBinary:
            return importSTLBinary(data: data)
        case .stlASCII:
            return importSTLASCII(data: data)
        case .obj:
            return importOBJ(data: data)
        }
    }

    /// Auto-detect format from the filename extension and import.
    public static func importModel(data: Data, filename: String) -> Mesh3D? {
        let ext = (filename as NSString).pathExtension.lowercased()
        switch ext {
        case "obj":
            return importOBJ(data: data)
        case "stl":
            return importSTL(data: data)
        default:
            return nil
        }
    }

    /// Import an STL file, auto-detecting whether it is binary or ASCII.
    public static func importSTL(data: Data) -> Mesh3D? {
        if isASCIISTL(data: data) {
            return importSTLASCII(data: data)
        } else {
            return importSTLBinary(data: data)
        }
    }

    /// Import a Wavefront OBJ file.
    public static func importOBJ(data: Data) -> Mesh3D? {
        guard let content = String(data: data, encoding: .utf8) ?? String(data: data, encoding: .ascii) else {
            return nil
        }
        return parseOBJ(content)
    }

    // MARK: - STL Binary

    private static func importSTLBinary(data: Data) -> Mesh3D? {
        // Minimum: 80 (header) + 4 (count) = 84 bytes
        guard data.count >= 84 else { return nil }

        return data.withUnsafeBytes { buffer -> Mesh3D? in
            guard let baseAddress = buffer.baseAddress else { return nil }

            // Read triangle count at offset 80
            let triangleCount = baseAddress.advanced(by: 80)
                .assumingMemoryBound(to: UInt32.self)
                .pointee
            let count = Int(UInt32(littleEndian: triangleCount))

            // Validate data size: 84 + count * 50
            let expectedSize = 84 + count * 50
            guard data.count >= expectedSize else { return nil }
            guard count > 0 else { return nil }

            var triangles: [Triangle3D] = []
            triangles.reserveCapacity(count)

            var offset = 84
            for _ in 0..<count {
                let floatPtr = baseAddress.advanced(by: offset)
                    .assumingMemoryBound(to: Float32.self)

                let nx = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[0].bitPattern)))
                let ny = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[1].bitPattern)))
                let nz = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[2].bitPattern)))

                let x0 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[3].bitPattern)))
                let y0 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[4].bitPattern)))
                let z0 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[5].bitPattern)))

                let x1 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[6].bitPattern)))
                let y1 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[7].bitPattern)))
                let z1 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[8].bitPattern)))

                let x2 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[9].bitPattern)))
                let y2 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[10].bitPattern)))
                let z2 = Double(Float32(bitPattern: UInt32(littleEndian: floatPtr[11].bitPattern)))

                let normal = Vector3D(nx, ny, nz)
                let v0 = Vector3D(x0, y0, z0)
                let v1 = Vector3D(x1, y1, z1)
                let v2 = Vector3D(x2, y2, z2)

                triangles.append(Triangle3D(v0: v0, v1: v1, v2: v2, normal: normal))

                // 12 floats (48 bytes) + 2 bytes attribute = 50 bytes
                offset += 50
            }

            return Mesh3D(triangles: triangles)
        }
    }

    // MARK: - STL ASCII

    private static func importSTLASCII(data: Data) -> Mesh3D? {
        guard let content = String(data: data, encoding: .utf8) ?? String(data: data, encoding: .ascii) else {
            return nil
        }

        var triangles: [Triangle3D] = []
        let lines = content.components(separatedBy: .newlines)

        var currentNormal: Vector3D?
        var currentVertices: [Vector3D] = []

        for rawLine in lines {
            let line = rawLine.trimmingCharacters(in: .whitespaces).lowercased()

            if line.hasPrefix("facet normal") {
                let parts = parseFloats(from: rawLine)
                if parts.count >= 3 {
                    currentNormal = Vector3D(parts[0], parts[1], parts[2])
                }
                currentVertices = []
            } else if line.hasPrefix("vertex") {
                let parts = parseFloats(from: rawLine)
                if parts.count >= 3 {
                    currentVertices.append(Vector3D(parts[0], parts[1], parts[2]))
                }
            } else if line.hasPrefix("endfacet") {
                if currentVertices.count == 3 {
                    if let normal = currentNormal {
                        triangles.append(Triangle3D(
                            v0: currentVertices[0],
                            v1: currentVertices[1],
                            v2: currentVertices[2],
                            normal: normal
                        ))
                    } else {
                        triangles.append(Triangle3D(
                            v0: currentVertices[0],
                            v1: currentVertices[1],
                            v2: currentVertices[2]
                        ))
                    }
                }
                currentNormal = nil
                currentVertices = []
            }
        }

        guard !triangles.isEmpty else { return nil }
        return Mesh3D(triangles: triangles)
    }

    // MARK: - OBJ

    private static func parseOBJ(_ content: String) -> Mesh3D? {
        var vertices: [Vector3D] = []
        var normals: [Vector3D] = []
        var triangles: [Triangle3D] = []

        let lines = content.components(separatedBy: .newlines)

        for rawLine in lines {
            let line = rawLine.trimmingCharacters(in: .whitespaces)
            guard !line.isEmpty && !line.hasPrefix("#") else { continue }

            let parts = line.split(separator: " ", omittingEmptySubsequences: true)
            guard let command = parts.first else { continue }

            switch command {
            case "v":
                // Vertex position: v x y z
                guard parts.count >= 4,
                      let x = Double(parts[1]),
                      let y = Double(parts[2]),
                      let z = Double(parts[3]) else { continue }
                vertices.append(Vector3D(x, y, z))

            case "vn":
                // Vertex normal: vn nx ny nz
                guard parts.count >= 4,
                      let nx = Double(parts[1]),
                      let ny = Double(parts[2]),
                      let nz = Double(parts[3]) else { continue }
                normals.append(Vector3D(nx, ny, nz))

            case "f":
                // Face: f v1 v2 v3 ... or f v1/vt1/vn1 v2/vt2/vn2 ...
                let faceIndices = parseFaceIndices(parts: Array(parts.dropFirst()))
                guard faceIndices.count >= 3 else { continue }

                // Triangulate: fan triangulation from first vertex
                for i in 1..<(faceIndices.count - 1) {
                    let (vi0, _, ni0) = faceIndices[0]
                    let (vi1, _, _) = faceIndices[i]
                    let (vi2, _, _) = faceIndices[i + 1]

                    // OBJ indices are 1-based; convert to 0-based
                    guard vi0 >= 1 && vi0 <= vertices.count,
                          vi1 >= 1 && vi1 <= vertices.count,
                          vi2 >= 1 && vi2 <= vertices.count else { continue }

                    let v0 = vertices[vi0 - 1]
                    let v1 = vertices[vi1 - 1]
                    let v2 = vertices[vi2 - 1]

                    // Use provided normal if available (from first vertex), otherwise compute
                    if let ni = ni0, ni >= 1 && ni <= normals.count {
                        triangles.append(Triangle3D(v0: v0, v1: v1, v2: v2, normal: normals[ni - 1]))
                    } else {
                        triangles.append(Triangle3D(v0: v0, v1: v1, v2: v2))
                    }
                }

            default:
                // Ignore vt, mtllib, usemtl, s, g, o, etc.
                break
            }
        }

        guard !triangles.isEmpty else { return nil }
        return Mesh3D(triangles: triangles)
    }

    /// Parse OBJ face vertex indices. Handles formats:
    ///   v
    ///   v/vt
    ///   v/vt/vn
    ///   v//vn
    /// Returns array of (vertexIndex, textureIndex?, normalIndex?).
    private static func parseFaceIndices(parts: [Substring]) -> [(Int, Int?, Int?)] {
        var indices: [(Int, Int?, Int?)] = []

        for part in parts {
            let components = part.split(separator: "/", omittingEmptySubsequences: false)

            guard let vi = Int(components[0]) else { continue }

            var vti: Int? = nil
            var vni: Int? = nil

            if components.count >= 2 && !components[1].isEmpty {
                vti = Int(components[1])
            }
            if components.count >= 3 && !components[2].isEmpty {
                vni = Int(components[2])
            }

            indices.append((vi, vti, vni))
        }

        return indices
    }

    // MARK: - Format Detection

    /// Detect whether STL data is ASCII format.
    /// ASCII STL starts with "solid" and contains "facet" somewhere in the file.
    private static func isASCIISTL(data: Data) -> Bool {
        // Check for "solid" prefix (ASCII STL starts with "solid name")
        guard data.count >= 5 else { return false }

        let headerBytes = data.prefix(5)
        guard let headerStr = String(data: headerBytes, encoding: .ascii),
              headerStr.lowercased() == "solid" else {
            return false
        }

        // Some binary STLs happen to start with "solid" in the header.
        // Check for "facet" keyword within the first ~1000 bytes to confirm ASCII.
        let checkLength = Swift.min(data.count, 1000)
        let checkData = data.prefix(checkLength)
        guard let checkStr = String(data: checkData, encoding: .ascii) else {
            return false
        }
        return checkStr.lowercased().contains("facet")
    }

    // MARK: - Ray-Triangle Intersection (Moller-Trumbore)

    /// Cast a ray downward at (rayX, rayY) and find the Z intersection with a triangle.
    /// Returns the Z coordinate of the intersection, or nil if the ray misses.
    ///
    /// Uses barycentric coordinates in the XY projection to determine if the point
    /// lies within the triangle, then interpolates the Z value.
    static func rayTriangleIntersectionZ(
        rayX: Double,
        rayY: Double,
        v0: Vector3D,
        v1: Vector3D,
        v2: Vector3D
    ) -> Double? {
        return barycentricIntersectionZ(
            px: rayX, py: rayY,
            v0: v0, v1: v1, v2: v2
        )
    }

    /// Compute the Z value at point (px, py) if it lies within the XY projection of the triangle.
    private static func barycentricIntersectionZ(
        px: Double, py: Double,
        v0: Vector3D, v1: Vector3D, v2: Vector3D
    ) -> Double? {
        // Compute barycentric coordinates of (px, py) with respect to the
        // XY projection of the triangle (v0, v1, v2).
        let dx0 = v1.x - v0.x
        let dy0 = v1.y - v0.y
        let dx1 = v2.x - v0.x
        let dy1 = v2.y - v0.y
        let dx2 = px - v0.x
        let dy2 = py - v0.y

        let d00 = dx0 * dx0 + dy0 * dy0
        let d01 = dx0 * dx1 + dy0 * dy1
        let d11 = dx1 * dx1 + dy1 * dy1
        let d20 = dx2 * dx0 + dy2 * dy0
        let d21 = dx2 * dx1 + dy2 * dy1

        let denom = d00 * d11 - d01 * d01
        guard abs(denom) > 1e-12 else { return nil }

        let invDenom = 1.0 / denom
        let u = (d11 * d20 - d01 * d21) * invDenom
        let v = (d00 * d21 - d01 * d20) * invDenom

        // Check if point is inside triangle
        guard u >= -1e-9 && v >= -1e-9 && (u + v) <= 1.0 + 1e-9 else {
            return nil
        }

        // Interpolate Z
        let z = v0.z + u * (v1.z - v0.z) + v * (v2.z - v0.z)
        return z
    }

    // MARK: - Parsing Helpers

    /// Extract floating point numbers from a line of text.
    private static func parseFloats(from line: String) -> [Double] {
        var results: [Double] = []
        let parts = line.split(whereSeparator: { $0 == " " || $0 == "\t" })
        for part in parts {
            if let value = Double(part) {
                results.append(value)
            }
        }
        return results
    }
}
