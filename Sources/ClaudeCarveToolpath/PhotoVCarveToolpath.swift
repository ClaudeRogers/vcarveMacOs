import Foundation
import AppKit
import ClaudeCarveCore

// MARK: - Strategy

/// The scan-line strategy used to convert image brightness into toolpath lines.
public enum PhotoVCarveStrategy: String, Codable, Sendable {
    case horizontal   // Horizontal scan lines (left-right zigzag)
    case vertical     // Vertical scan lines (top-bottom zigzag)
    case crossHatch   // Both horizontal and vertical combined
    case diagonal     // 45-degree lines
    case stipple      // Dot stippling pattern — depth proportional to darkness
    case spiral       // Archimedean spiral from center outward
}

// MARK: - Configuration

/// Configuration for photo V-carve toolpath generation.
public struct PhotoVCarveConfig: Codable, Sendable {
    /// The scan-line strategy to use.
    public var strategy: PhotoVCarveStrategy
    /// Distance between adjacent scan lines in mm.
    public var lineSpacing: Double
    /// Maximum cut depth in mm (darkest pixels).
    public var maxDepth: Double
    /// Minimum cut depth in mm (lightest pixels; 0 means white is not cut).
    public var minDepth: Double
    /// When true, white maps to deepest cut and black to shallowest.
    public var invertImage: Bool
    /// Contrast adjustment factor. Range 0-2; 1 is unchanged.
    public var contrast: Double
    /// Brightness offset. Range -1 to 1; 0 is unchanged.
    public var brightness: Double
    /// Margin around the image area in mm.
    public var borderMargin: Double
    /// Number of sample points per mm along each scan line.
    public var resolution: Double
    /// XY feed rate in mm/min.
    public var feedRate: Double
    /// Z plunge rate in mm/min.
    public var plungeRate: Double
    /// Safe retract height in mm above the workpiece surface.
    public var safeZ: Double

    public init(
        strategy: PhotoVCarveStrategy = .horizontal,
        lineSpacing: Double = 0.5,
        maxDepth: Double = 3.0,
        minDepth: Double = 0.0,
        invertImage: Bool = false,
        contrast: Double = 1.0,
        brightness: Double = 0.0,
        borderMargin: Double = 2.0,
        resolution: Double = 2.0,
        feedRate: Double = 1000,
        plungeRate: Double = 500,
        safeZ: Double = 5.0
    ) {
        self.strategy = strategy
        self.lineSpacing = lineSpacing
        self.maxDepth = maxDepth
        self.minDepth = minDepth
        self.invertImage = invertImage
        self.contrast = contrast
        self.brightness = brightness
        self.borderMargin = borderMargin
        self.resolution = resolution
        self.feedRate = feedRate
        self.plungeRate = plungeRate
        self.safeZ = safeZ
    }
}

// MARK: - Generator

/// Converts a photograph into a V-carved engraving toolpath where line
/// density and depth represent image brightness.
///
/// Algorithm overview:
/// 1. Convert the source image to a normalised grayscale float buffer.
/// 2. Apply brightness / contrast adjustments.
/// 3. For each scan line dictated by the strategy and spacing, sample the
///    image at regular intervals and map brightness to cut depth.
/// 4. Generate toolpath moves with zigzag ordering, rapid retracts over
///    regions that would not be cut, and safe-Z transitions.
public struct PhotoVCarveToolpath: Sendable {

    // MARK: - Public entry points

    /// Generate a photo V-carve toolpath from an `NSImage`.
    ///
    /// - Parameters:
    ///   - image: Source photograph.
    ///   - config: Generation parameters.
    ///   - tool: The cutting tool.
    ///   - origin: Bottom-left corner of the engraving area in workpiece coordinates.
    ///   - width: Engraving width in mm.
    ///   - height: Engraving height in mm.
    /// - Returns: A fully computed toolpath.
    public static func generate(
        image: NSImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> ComputedToolpath {
        guard let grayscale = imageToGrayscale(image: image) else {
            return emptyToolpath(tool: tool)
        }
        return generateFromGrayscale(
            grayscale: grayscale,
            config: config,
            tool: tool,
            origin: origin,
            width: width,
            height: height
        )
    }

    /// Generate a photo V-carve toolpath from raw image data (PNG, JPEG, TIFF, etc.).
    public static func generate(
        imageData: Data,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> ComputedToolpath {
        guard let nsImage = NSImage(data: imageData) else {
            return emptyToolpath(tool: tool)
        }
        return generate(
            image: nsImage,
            config: config,
            tool: tool,
            origin: origin,
            width: width,
            height: height
        )
    }

    // MARK: - Grayscale buffer

    /// A row-major grayscale image where 0 = black and 1 = white.
    private struct GrayscaleImage: Sendable {
        let pixels: [Float]
        let width: Int
        let height: Int

        /// Sample brightness at fractional pixel coordinates using bilinear interpolation.
        /// Coordinates are clamped to image bounds.
        func sample(px: Double, py: Double) -> Double {
            let fx = max(0, min(Double(width - 1), px))
            let fy = max(0, min(Double(height - 1), py))

            let x0 = Int(fx)
            let y0 = Int(fy)
            let x1 = min(x0 + 1, width - 1)
            let y1 = min(y0 + 1, height - 1)

            let dx = fx - Double(x0)
            let dy = fy - Double(y0)

            let v00 = Double(pixels[y0 * width + x0])
            let v10 = Double(pixels[y0 * width + x1])
            let v01 = Double(pixels[y1 * width + x0])
            let v11 = Double(pixels[y1 * width + x1])

            let top = v00 * (1 - dx) + v10 * dx
            let bottom = v01 * (1 - dx) + v11 * dx
            return top * (1 - dy) + bottom * dy
        }
    }

    /// Convert an NSImage to a normalized grayscale float buffer.
    private static func imageToGrayscale(image: NSImage) -> GrayscaleImage? {
        guard let cgImage = image.cgImage(forProposedRect: nil, context: nil, hints: nil) else {
            return nil
        }

        let w = cgImage.width
        let h = cgImage.height
        guard w > 0 && h > 0 else { return nil }

        let colorSpace = CGColorSpaceCreateDeviceGray()
        let bytesPerRow = w
        var rawPixels = [UInt8](repeating: 0, count: w * h)

        guard let context = CGContext(
            data: &rawPixels,
            width: w,
            height: h,
            bitsPerComponent: 8,
            bytesPerRow: bytesPerRow,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.none.rawValue
        ) else {
            return nil
        }

        // Draw flipped so row 0 = top of image = max Y in workpiece space.
        context.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))

        let floats = rawPixels.map { Float($0) / 255.0 }
        return GrayscaleImage(pixels: floats, width: w, height: h)
    }

    // MARK: - Brightness / contrast

    /// Apply brightness and contrast adjustments to a brightness value in [0,1].
    /// Contrast stretches values around 0.5; brightness shifts linearly.
    private static func adjustBrightness(
        _ value: Double,
        contrast: Double,
        brightness: Double
    ) -> Double {
        var v = value
        // Contrast: scale around midpoint 0.5
        v = (v - 0.5) * contrast + 0.5
        // Brightness: linear shift
        v += brightness
        return max(0, min(1, v))
    }

    // MARK: - Depth mapping

    /// Map an adjusted brightness value to a cut depth.
    /// Darker pixels produce deeper cuts.
    private static func brightnessToDepth(
        _ brightness: Double,
        config: PhotoVCarveConfig
    ) -> Double {
        let b = config.invertImage ? brightness : (1.0 - brightness)
        return config.minDepth + b * (config.maxDepth - config.minDepth)
    }

    /// Threshold below which a depth is considered "no cut" and can be skipped.
    private static let depthEpsilon: Double = 0.01

    // MARK: - Core generation

    private static func generateFromGrayscale(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> ComputedToolpath {
        let margin = config.borderMargin
        let cutOrigin = Vector2D(origin.x + margin, origin.y + margin)
        let cutWidth = max(0, width - 2 * margin)
        let cutHeight = max(0, height - 2 * margin)

        guard cutWidth > 0 && cutHeight > 0 else {
            return emptyToolpath(tool: tool)
        }

        let moves: [ToolpathMove]

        switch config.strategy {
        case .horizontal:
            moves = generateHorizontal(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
        case .vertical:
            moves = generateVertical(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
        case .crossHatch:
            let hMoves = generateHorizontal(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
            let vMoves = generateVertical(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
            moves = hMoves + vMoves
        case .diagonal:
            moves = generateDiagonal(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
        case .stipple:
            moves = generateStipple(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
        case .spiral:
            moves = generateSpiral(
                grayscale: grayscale, config: config, tool: tool,
                origin: cutOrigin, width: cutWidth, height: cutHeight
            )
        }

        var toolpath = ComputedToolpath(
            configID: UUID(),
            toolID: tool.id,
            moves: moves
        )
        toolpath.calculateEstimates()
        return toolpath
    }

    // MARK: - Sampling helpers

    /// Convert a workpiece coordinate to a pixel coordinate in the grayscale image.
    /// The image maps onto the cut area: cutOrigin .. cutOrigin + (cutWidth, cutHeight).
    /// Pixel row 0 = top of image = highest Y in workpiece space.
    private static func worldToPixel(
        wx: Double, wy: Double,
        cutOrigin: Vector2D, cutWidth: Double, cutHeight: Double,
        imageWidth: Int, imageHeight: Int
    ) -> (px: Double, py: Double) {
        let u = (wx - cutOrigin.x) / cutWidth   // 0..1 across width
        let v = (wy - cutOrigin.y) / cutHeight  // 0..1 across height
        let px = u * Double(imageWidth - 1)
        // Flip Y: workpiece Y=0 is bottom, image row 0 is top.
        let py = (1.0 - v) * Double(imageHeight - 1)
        return (px, py)
    }

    /// Sample brightness from the image at a workpiece coordinate, with
    /// brightness/contrast adjustments already applied.
    private static func sampleAt(
        wx: Double, wy: Double,
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        cutOrigin: Vector2D,
        cutWidth: Double,
        cutHeight: Double
    ) -> Double {
        let (px, py) = worldToPixel(
            wx: wx, wy: wy,
            cutOrigin: cutOrigin, cutWidth: cutWidth, cutHeight: cutHeight,
            imageWidth: grayscale.width, imageHeight: grayscale.height
        )
        let raw = grayscale.sample(px: px, py: py)
        return adjustBrightness(raw, contrast: config.contrast, brightness: config.brightness)
    }

    /// Sample depth at a workpiece coordinate.
    private static func depthAt(
        wx: Double, wy: Double,
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        cutOrigin: Vector2D,
        cutWidth: Double,
        cutHeight: Double
    ) -> Double {
        let brightness = sampleAt(
            wx: wx, wy: wy,
            grayscale: grayscale, config: config,
            cutOrigin: cutOrigin, cutWidth: cutWidth, cutHeight: cutHeight
        )
        return brightnessToDepth(brightness, config: config)
    }

    // MARK: - Horizontal strategy

    private static func generateHorizontal(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let stepAlongLine = 1.0 / config.resolution
        let numLines = max(1, Int(ceil(height / config.lineSpacing)))
        var leftToRight = true

        for lineIndex in 0...numLines {
            let y = origin.y + min(Double(lineIndex) * config.lineSpacing, height)
            let startX = leftToRight ? origin.x : origin.x + width
            let endX = leftToRight ? origin.x + width : origin.x
            let step = leftToRight ? stepAlongLine : -stepAlongLine

            let scanPoints = buildScanLine(
                fixedCoord: y, startMoving: startX, endMoving: endX,
                step: step, isHorizontal: true,
                grayscale: grayscale, config: config,
                cutOrigin: origin, cutWidth: width, cutHeight: height
            )

            appendScanLineMoves(
                scanPoints: scanPoints,
                config: config,
                moves: &moves
            )

            leftToRight = !leftToRight
        }

        return moves
    }

    // MARK: - Vertical strategy

    private static func generateVertical(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let stepAlongLine = 1.0 / config.resolution
        let numLines = max(1, Int(ceil(width / config.lineSpacing)))
        var bottomToTop = true

        for lineIndex in 0...numLines {
            let x = origin.x + min(Double(lineIndex) * config.lineSpacing, width)
            let startY = bottomToTop ? origin.y : origin.y + height
            let endY = bottomToTop ? origin.y + height : origin.y
            let step = bottomToTop ? stepAlongLine : -stepAlongLine

            let scanPoints = buildScanLine(
                fixedCoord: x, startMoving: startY, endMoving: endY,
                step: step, isHorizontal: false,
                grayscale: grayscale, config: config,
                cutOrigin: origin, cutWidth: width, cutHeight: height
            )

            appendScanLineMoves(
                scanPoints: scanPoints,
                config: config,
                moves: &moves
            )

            bottomToTop = !bottomToTop
        }

        return moves
    }

    // MARK: - Diagonal strategy

    private static func generateDiagonal(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let stepAlongLine = 1.0 / config.resolution

        // 45-degree lines perpendicular spacing = lineSpacing
        // Actual diagonal distance between parallel lines at 45 deg:
        let perpSpacing = config.lineSpacing
        let diagLength = width + height  // max diagonal extent
        let numLines = max(1, Int(ceil(diagLength / perpSpacing)))
        var forward = true

        // Lines run from bottom-left toward top-right at 45 degrees.
        // We parameterise by the intercept along the bottom + left edges.
        for lineIndex in 0...numLines {
            let offset = Double(lineIndex) * perpSpacing

            // Start and end of the diagonal within the bounding rectangle.
            let (x0, y0, x1, y1) = diagonalLineClip(
                offset: offset,
                originX: origin.x, originY: origin.y,
                width: width, height: height
            )

            guard (x1 - x0) != 0 || (y1 - y0) != 0 else { continue }

            let dx = x1 - x0
            let dy = y1 - y0
            let lineLen = sqrt(dx * dx + dy * dy)
            guard lineLen > stepAlongLine else { continue }

            let numSamples = max(1, Int(ceil(lineLen / stepAlongLine)))
            var scanPoints: [(Vector2D, Double)] = []

            for si in 0...numSamples {
                let t = Double(si) / Double(numSamples)
                let tDir = forward ? t : (1.0 - t)
                let wx = x0 + dx * tDir
                let wy = y0 + dy * tDir

                let depth = depthAt(
                    wx: wx, wy: wy,
                    grayscale: grayscale, config: config,
                    cutOrigin: origin, cutWidth: width, cutHeight: height
                )
                scanPoints.append((Vector2D(wx, wy), depth))
            }

            appendScanLineMoves(scanPoints: scanPoints, config: config, moves: &moves)
            forward = !forward
        }

        return moves
    }

    /// Clip a 45-degree diagonal line to the engraving rectangle.
    /// `offset` is the perpendicular distance from the bottom-left corner.
    private static func diagonalLineClip(
        offset: Double,
        originX: Double, originY: Double,
        width: Double, height: Double
    ) -> (x0: Double, y0: Double, x1: Double, y1: Double) {
        // The family of 45-degree lines: y - originY = (x - originX) - d
        // where d shifts along the perpendicular. d = offset * sqrt(2).
        let d = offset * sqrt(2.0)

        // Line equation in local coords (lx, ly from origin):
        //   ly = lx - d
        // Intersect with the rectangle [0, width] x [0, height].
        var lx0: Double
        var ly0: Double
        var lx1: Double
        var ly1: Double

        // Entry: either left edge (lx=0) or bottom edge (ly=0)
        if d <= 0 {
            // Line enters from the left edge
            lx0 = 0
            ly0 = -d
        } else {
            // Line enters from the bottom edge
            lx0 = d
            ly0 = 0
        }

        // Exit: either right edge (lx=width) or top edge (ly=height)
        let lyAtRight = width - d
        let lxAtTop = height + d

        if lyAtRight <= height {
            lx1 = width
            ly1 = lyAtRight
        } else {
            lx1 = lxAtTop
            ly1 = height
        }

        // Clamp to rectangle
        lx0 = max(0, min(width, lx0))
        ly0 = max(0, min(height, ly0))
        lx1 = max(0, min(width, lx1))
        ly1 = max(0, min(height, ly1))

        return (originX + lx0, originY + ly0, originX + lx1, originY + ly1)
    }

    // MARK: - Stipple strategy

    private static func generateStipple(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let spacing = config.lineSpacing
        let numX = max(1, Int(ceil(width / spacing)))
        let numY = max(1, Int(ceil(height / spacing)))

        for iy in 0..<numY {
            for ix in 0..<numX {
                let wx = origin.x + (Double(ix) + 0.5) * spacing
                let wy = origin.y + (Double(iy) + 0.5) * spacing

                // Ensure within bounds
                guard wx <= origin.x + width && wy <= origin.y + height else { continue }

                let depth = depthAt(
                    wx: wx, wy: wy,
                    grayscale: grayscale, config: config,
                    cutOrigin: origin, cutWidth: width, cutHeight: height
                )

                guard depth > depthEpsilon else { continue }

                let pos = Vector2D(wx, wy)
                moves.append(.rapid(to: Vector3D(pos, z: config.safeZ)))
                moves.append(.linear(to: Vector3D(pos, z: -depth), feed: config.plungeRate))
                moves.append(.rapid(to: Vector3D(pos, z: config.safeZ)))
            }
        }

        return moves
    }

    // MARK: - Spiral strategy

    private static func generateSpiral(
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        tool: Tool,
        origin: Vector2D,
        width: Double,
        height: Double
    ) -> [ToolpathMove] {
        var moves: [ToolpathMove] = []
        let cx = origin.x + width / 2.0
        let cy = origin.y + height / 2.0
        let maxRadius = sqrt(width * width + height * height) / 2.0
        let angularStep = 1.0 / (config.resolution * maxRadius)  // radians per step (approx)
        let radiusPerRev = config.lineSpacing  // radial growth per full revolution

        var angle = 0.0
        var inCut = false

        // Archimedean spiral: r = (radiusPerRev / 2pi) * theta
        let a = radiusPerRev / (2.0 * .pi)

        while true {
            let r = a * angle
            if r > maxRadius { break }

            let wx = cx + r * cos(angle)
            let wy = cy + r * sin(angle)

            // Check if point is within the engraving rectangle
            let inBounds = wx >= origin.x && wx <= origin.x + width
                        && wy >= origin.y && wy <= origin.y + height

            if inBounds {
                let depth = depthAt(
                    wx: wx, wy: wy,
                    grayscale: grayscale, config: config,
                    cutOrigin: origin, cutWidth: width, cutHeight: height
                )

                if depth > depthEpsilon {
                    let pos = Vector2D(wx, wy)
                    if !inCut {
                        moves.append(.rapid(to: Vector3D(pos, z: config.safeZ)))
                        moves.append(.linear(to: Vector3D(pos, z: -depth), feed: config.plungeRate))
                        inCut = true
                    } else {
                        moves.append(.linear(to: Vector3D(pos, z: -depth), feed: config.feedRate))
                    }
                } else {
                    if inCut {
                        let pos = Vector2D(wx, wy)
                        moves.append(.rapid(to: Vector3D(pos, z: config.safeZ)))
                        inCut = false
                    }
                }
            } else {
                if inCut {
                    // Retract when leaving bounds
                    let pos = Vector2D(
                        max(origin.x, min(origin.x + width, wx)),
                        max(origin.y, min(origin.y + height, wy))
                    )
                    moves.append(.rapid(to: Vector3D(pos, z: config.safeZ)))
                    inCut = false
                }
            }

            // Adaptive angular step: keep arc-length roughly constant.
            let effectiveStep = r > 1.0 ? angularStep * (maxRadius / r) : angularStep
            angle += max(effectiveStep, 0.01)
        }

        // Final retract
        if inCut, let lastPos = moves.last?.position {
            moves.append(.rapid(to: Vector3D(lastPos.xy, z: config.safeZ)))
        }

        return moves
    }

    // MARK: - Scan-line helpers

    /// A scan point: position in workpiece coordinates and the corresponding depth.
    private typealias ScanPoint = (position: Vector2D, depth: Double)

    /// Build an array of (position, depth) samples along a single scan line.
    ///
    /// - Parameters:
    ///   - fixedCoord: The coordinate that stays constant (Y for horizontal, X for vertical).
    ///   - startMoving: Start value of the varying coordinate.
    ///   - endMoving: End value of the varying coordinate.
    ///   - step: Increment along the moving axis (sign indicates direction).
    ///   - isHorizontal: If true the moving axis is X; otherwise Y.
    private static func buildScanLine(
        fixedCoord: Double,
        startMoving: Double,
        endMoving: Double,
        step: Double,
        isHorizontal: Bool,
        grayscale: GrayscaleImage,
        config: PhotoVCarveConfig,
        cutOrigin: Vector2D,
        cutWidth: Double,
        cutHeight: Double
    ) -> [ScanPoint] {
        var points: [ScanPoint] = []
        let totalDist = abs(endMoving - startMoving)
        let numSamples = max(1, Int(ceil(totalDist * config.resolution)))

        for i in 0...numSamples {
            let t = Double(i) / Double(numSamples)
            let movingCoord = startMoving + (endMoving - startMoving) * t

            let wx: Double
            let wy: Double
            if isHorizontal {
                wx = movingCoord
                wy = fixedCoord
            } else {
                wx = fixedCoord
                wy = movingCoord
            }

            let depth = depthAt(
                wx: wx, wy: wy,
                grayscale: grayscale, config: config,
                cutOrigin: cutOrigin, cutWidth: cutWidth, cutHeight: cutHeight
            )

            points.append((Vector2D(wx, wy), depth))
        }

        return points
    }

    /// Convert a series of scan points into toolpath moves with rapids over
    /// non-cutting regions and safe-Z retracts between disconnected segments.
    private static func appendScanLineMoves(
        scanPoints: [ScanPoint],
        config: PhotoVCarveConfig,
        moves: inout [ToolpathMove]
    ) {
        guard !scanPoints.isEmpty else { return }

        var inCut = false

        for (position, depth) in scanPoints {
            if depth > depthEpsilon {
                if !inCut {
                    // Plunge into material
                    moves.append(.rapid(to: Vector3D(position, z: config.safeZ)))
                    moves.append(.linear(to: Vector3D(position, z: -depth), feed: config.plungeRate))
                    inCut = true
                } else {
                    moves.append(.linear(to: Vector3D(position, z: -depth), feed: config.feedRate))
                }
            } else {
                if inCut {
                    // Retract out of material
                    moves.append(.rapid(to: Vector3D(position, z: config.safeZ)))
                    inCut = false
                }
            }
        }

        // Final retract if we ended in a cut
        if inCut, let lastPoint = scanPoints.last {
            moves.append(.rapid(to: Vector3D(lastPoint.position, z: config.safeZ)))
        }
    }

    // MARK: - Utilities

    private static func emptyToolpath(tool: Tool) -> ComputedToolpath {
        ComputedToolpath(configID: UUID(), toolID: tool.id, moves: [])
    }
}
