import Foundation
import AppKit
import ClaudeCarveCore

// MARK: - Tracing Mode

/// Determines how an image is interpreted for vector tracing.
public enum TracingMode: String, Codable, Sendable {
    /// Trace center lines (for engraving). Uses morphological thinning/skeletonization.
    case centerline
    /// Trace outlines (for cutting). Produces closed contour paths.
    case outline
    /// Simple threshold black/white conversion before tracing.
    case blackWhite
    /// Separate into color-based layers, tracing each independently.
    case colorLayers
}

// MARK: - Tracing Configuration

/// Configuration parameters controlling the bitmap-to-vector tracing process.
public struct TracingConfig: Codable, Sendable {
    /// The tracing mode determining how the image is interpreted.
    public var mode: TracingMode
    /// Brightness threshold for binary conversion (0 = black, 1 = white).
    public var threshold: Double
    /// Smoothing factor applied to traced paths (0 = none, 1 = maximum).
    public var smoothing: Double
    /// Angle threshold in radians for corner detection during Bezier fitting.
    public var cornerThreshold: Double
    /// Minimum path length in coordinate units; paths shorter than this are discarded.
    public var minPathLength: Double
    /// Tolerance for Douglas-Peucker path simplification.
    public var simplifyTolerance: Double
    /// Whether to invert the binary image (swap black and white).
    public var invertColors: Bool
    /// Remove isolated pixel groups smaller than NxN pixels.
    public var despeckleSize: Int

    public init(
        mode: TracingMode = .outline,
        threshold: Double = 0.5,
        smoothing: Double = 0.5,
        cornerThreshold: Double = Double.pi / 4,
        minPathLength: Double = 5.0,
        simplifyTolerance: Double = 1.0,
        invertColors: Bool = false,
        despeckleSize: Int = 3
    ) {
        self.mode = mode
        self.threshold = threshold
        self.smoothing = smoothing
        self.cornerThreshold = cornerThreshold
        self.minPathLength = minPathLength
        self.simplifyTolerance = simplifyTolerance
        self.invertColors = invertColors
        self.despeckleSize = despeckleSize
    }
}

// MARK: - Bitmap Representation

/// An internal grayscale bitmap used during the tracing pipeline.
/// Pixel values are stored as 0.0 (black) to 1.0 (white).
/// The coordinate system has (0,0) at the top-left corner.
private struct GrayscaleBitmap: Sendable {
    let width: Int
    let height: Int
    var pixels: [Double]

    init(width: Int, height: Int, fill: Double = 0.0) {
        self.width = width
        self.height = height
        self.pixels = Array(repeating: fill, count: width * height)
    }

    func index(_ x: Int, _ y: Int) -> Int {
        y * width + x
    }

    func isValid(_ x: Int, _ y: Int) -> Bool {
        x >= 0 && x < width && y >= 0 && y < height
    }

    func pixel(_ x: Int, _ y: Int) -> Double {
        guard isValid(x, y) else { return 0.0 }
        return pixels[index(x, y)]
    }

    mutating func setPixel(_ x: Int, _ y: Int, _ value: Double) {
        guard isValid(x, y) else { return }
        pixels[index(x, y)] = value
    }
}

/// A binary (black/white) bitmap for contour tracing.
/// `true` represents a foreground (black) pixel, `false` represents background (white).
private struct BinaryBitmap: Sendable {
    let width: Int
    let height: Int
    var pixels: [Bool]

    init(width: Int, height: Int, fill: Bool = false) {
        self.width = width
        self.height = height
        self.pixels = Array(repeating: fill, count: width * height)
    }

    func index(_ x: Int, _ y: Int) -> Int {
        y * width + x
    }

    func isValid(_ x: Int, _ y: Int) -> Bool {
        x >= 0 && x < width && y >= 0 && y < height
    }

    func pixel(_ x: Int, _ y: Int) -> Bool {
        guard isValid(x, y) else { return false }
        return pixels[index(x, y)]
    }

    mutating func setPixel(_ x: Int, _ y: Int, _ value: Bool) {
        guard isValid(x, y) else { return }
        pixels[index(x, y)] = value
    }
}

// MARK: - Image Tracer

/// Converts raster images to vector paths using a Potrace-inspired algorithm.
///
/// The tracing pipeline:
/// 1. Convert to grayscale
/// 2. Apply threshold to produce a binary bitmap
/// 3. Despeckle (remove small isolated pixel groups)
/// 4. Find contours using square tracing / Moore neighborhood tracing
/// 5. Simplify paths with Douglas-Peucker
/// 6. Fit cubic Bezier curves to simplified point sequences
/// 7. Package as `VectorPath` objects
public struct ImageTracer: Sendable {

    // MARK: - Public API

    /// Trace an NSImage to vector paths.
    ///
    /// - Parameters:
    ///   - image: The source image to trace.
    ///   - config: Tracing parameters controlling the conversion.
    /// - Returns: An array of `VectorPath` objects representing the traced vector data.
    public static func trace(image: NSImage, config: TracingConfig) -> [VectorPath] {
        guard let tiffData = image.tiffRepresentation,
              let bitmap = NSBitmapImageRep(data: tiffData) else {
            return []
        }
        let grayscale = convertToGrayscale(bitmap: bitmap)
        return traceGrayscale(grayscale, config: config)
    }

    /// Trace from raw image data bytes.
    ///
    /// - Parameters:
    ///   - imageData: Image file data (PNG, JPEG, TIFF, etc.).
    ///   - config: Tracing parameters controlling the conversion.
    /// - Returns: An array of `VectorPath` objects representing the traced vector data.
    public static func trace(imageData: Data, config: TracingConfig) -> [VectorPath] {
        guard let bitmap = NSBitmapImageRep(data: imageData) else {
            return []
        }
        let grayscale = convertToGrayscale(bitmap: bitmap)
        return traceGrayscale(grayscale, config: config)
    }

    // MARK: - Pipeline

    /// Main tracing pipeline operating on a grayscale bitmap.
    private static func traceGrayscale(_ grayscale: GrayscaleBitmap, config: TracingConfig) -> [VectorPath] {
        switch config.mode {
        case .outline, .blackWhite:
            return traceOutline(grayscale, config: config)
        case .centerline:
            return traceCenterline(grayscale, config: config)
        case .colorLayers:
            return traceColorLayers(grayscale, config: config)
        }
    }

    /// Outline / black-white tracing: threshold, despeckle, find contours, simplify, fit curves.
    private static func traceOutline(_ grayscale: GrayscaleBitmap, config: TracingConfig) -> [VectorPath] {
        var binary = applyThreshold(grayscale, threshold: config.threshold, invert: config.invertColors)
        if config.despeckleSize > 1 {
            binary = despeckle(binary, minSize: config.despeckleSize)
        }
        let contours = findContours(binary)
        return contoursToVectorPaths(
            contours,
            imageHeight: grayscale.height,
            config: config,
            closed: true
        )
    }

    /// Centerline tracing: threshold, despeckle, skeletonize, trace skeleton paths.
    private static func traceCenterline(_ grayscale: GrayscaleBitmap, config: TracingConfig) -> [VectorPath] {
        var binary = applyThreshold(grayscale, threshold: config.threshold, invert: config.invertColors)
        if config.despeckleSize > 1 {
            binary = despeckle(binary, minSize: config.despeckleSize)
        }
        let skeleton = zhangSuenThinning(binary)
        let chains = traceSkeletonChains(skeleton)
        return contoursToVectorPaths(
            chains,
            imageHeight: grayscale.height,
            config: config,
            closed: false
        )
    }

    /// Color layer tracing: quantize grayscale into bands, trace each band separately.
    private static func traceColorLayers(_ grayscale: GrayscaleBitmap, config: TracingConfig) -> [VectorPath] {
        let layerCount = max(2, min(8, Int(1.0 / max(0.05, config.threshold))))
        var allPaths: [VectorPath] = []

        for layer in 0..<layerCount {
            let low = Double(layer) / Double(layerCount)
            let high = Double(layer + 1) / Double(layerCount)
            var layerBinary = BinaryBitmap(width: grayscale.width, height: grayscale.height)
            for y in 0..<grayscale.height {
                for x in 0..<grayscale.width {
                    let v = grayscale.pixel(x, y)
                    let inRange = v >= low && v < high
                    layerBinary.setPixel(x, y, config.invertColors ? !inRange : inRange)
                }
            }
            if config.despeckleSize > 1 {
                layerBinary = despeckle(layerBinary, minSize: config.despeckleSize)
            }
            let contours = findContours(layerBinary)
            let paths = contoursToVectorPaths(
                contours,
                imageHeight: grayscale.height,
                config: config,
                closed: true
            )
            allPaths.append(contentsOf: paths)
        }

        return allPaths
    }

    // MARK: - Image Conversion

    /// Convert an NSBitmapImageRep to a grayscale bitmap.
    /// Uses the luminance formula: 0.299*R + 0.587*G + 0.114*B.
    private static func convertToGrayscale(bitmap: NSBitmapImageRep) -> GrayscaleBitmap {
        let w = bitmap.pixelsWide
        let h = bitmap.pixelsHigh
        var result = GrayscaleBitmap(width: w, height: h)

        for y in 0..<h {
            for x in 0..<w {
                if let color = bitmap.colorAt(x: x, y: y) {
                    // Convert to sRGB to get consistent R/G/B values
                    let srgb = color.usingColorSpace(.sRGB) ?? color
                    let r = srgb.redComponent
                    let g = srgb.greenComponent
                    let b = srgb.blueComponent
                    let gray = 0.299 * r + 0.587 * g + 0.114 * b
                    result.setPixel(x, y, gray)
                } else {
                    result.setPixel(x, y, 0.0)
                }
            }
        }

        return result
    }

    // MARK: - Thresholding

    /// Convert a grayscale bitmap to a binary bitmap using a brightness threshold.
    /// Pixels darker than the threshold become foreground (true).
    private static func applyThreshold(
        _ grayscale: GrayscaleBitmap,
        threshold: Double,
        invert: Bool
    ) -> BinaryBitmap {
        var binary = BinaryBitmap(width: grayscale.width, height: grayscale.height)
        for y in 0..<grayscale.height {
            for x in 0..<grayscale.width {
                let v = grayscale.pixel(x, y)
                let isForeground = v < threshold
                binary.setPixel(x, y, invert ? !isForeground : isForeground)
            }
        }
        return binary
    }

    // MARK: - Despeckle

    /// Remove small isolated pixel groups using connected-component labeling.
    /// Groups with fewer than minSize*minSize pixels are removed.
    private static func despeckle(_ bitmap: BinaryBitmap, minSize: Int) -> BinaryBitmap {
        let minPixels = minSize * minSize
        var result = bitmap
        var visited = BinaryBitmap(width: bitmap.width, height: bitmap.height)
        let directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
                          (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for y in 0..<bitmap.height {
            for x in 0..<bitmap.width {
                guard bitmap.pixel(x, y) && !visited.pixel(x, y) else { continue }

                // Flood-fill to find connected component
                var component: [(Int, Int)] = []
                var stack: [(Int, Int)] = [(x, y)]
                visited.setPixel(x, y, true)

                while let (cx, cy) = stack.popLast() {
                    component.append((cx, cy))
                    for (dx, dy) in directions {
                        let nx = cx + dx
                        let ny = cy + dy
                        if bitmap.isValid(nx, ny) && bitmap.pixel(nx, ny) && !visited.pixel(nx, ny) {
                            visited.setPixel(nx, ny, true)
                            stack.append((nx, ny))
                        }
                    }
                }

                // Remove component if too small
                if component.count < minPixels {
                    for (cx, cy) in component {
                        result.setPixel(cx, cy, false)
                    }
                }
            }
        }

        return result
    }

    // MARK: - Contour Finding (Moore Neighborhood Tracing)

    /// Find all contour boundaries in a binary bitmap using the Moore neighborhood
    /// tracing algorithm. Each contour is returned as a sequence of boundary pixel
    /// coordinates.
    private static func findContours(_ bitmap: BinaryBitmap) -> [[(Int, Int)]] {
        var contours: [[(Int, Int)]] = []
        var visited = BinaryBitmap(width: bitmap.width, height: bitmap.height)

        // Moore neighborhood: 8 directions starting from east, going clockwise
        //   7 0 1
        //   6 . 2
        //   5 4 3
        let mooreX = [ 1,  1,  1,  0, -1, -1, -1,  0]
        let mooreY = [ 0, -1, -1, -1,  0,  1,  1,  1]  // note: image y increases downward

        for y in 0..<bitmap.height {
            for x in 0..<bitmap.width {
                // Look for an unvisited foreground pixel with a background pixel to its left
                guard bitmap.pixel(x, y) && !visited.pixel(x, y) else { continue }
                let leftIsBg = !bitmap.isValid(x - 1, y) || !bitmap.pixel(x - 1, y)
                guard leftIsBg else { continue }

                // Trace contour using Moore neighborhood
                var contour: [(Int, Int)] = []
                let startX = x
                let startY = y

                var curX = startX
                var curY = startY
                // Enter from the left (direction index 6 = west, meaning we came from the west)
                var backtrackDir = 6

                var steps = 0
                let maxSteps = bitmap.width * bitmap.height * 2

                repeat {
                    contour.append((curX, curY))
                    visited.setPixel(curX, curY, true)

                    // Start searching from the pixel after the backtrack direction
                    let searchStart = (backtrackDir + 1) % 8
                    var foundNext = false

                    for i in 0..<8 {
                        let dir = (searchStart + i) % 8
                        let nx = curX + mooreX[dir]
                        let ny = curY + mooreY[dir]

                        if bitmap.isValid(nx, ny) && bitmap.pixel(nx, ny) {
                            // The backtrack direction for the next pixel is opposite of
                            // the direction we used to reach it
                            backtrackDir = (dir + 4) % 8
                            curX = nx
                            curY = ny
                            foundNext = true
                            break
                        }
                    }

                    if !foundNext {
                        break  // Isolated pixel
                    }

                    steps += 1
                } while (curX != startX || curY != startY) && steps < maxSteps

                if contour.count >= 3 {
                    contours.append(contour)
                }
            }
        }

        return contours
    }

    // MARK: - Skeletonization (Zhang-Suen Thinning)

    /// Apply the Zhang-Suen thinning algorithm to produce a one-pixel-wide skeleton.
    /// This is used for centerline tracing mode.
    private static func zhangSuenThinning(_ bitmap: BinaryBitmap) -> BinaryBitmap {
        var current = bitmap
        var changed = true

        while changed {
            changed = false

            // Sub-iteration 1
            var toRemove: [(Int, Int)] = []
            for y in 1..<(current.height - 1) {
                for x in 1..<(current.width - 1) {
                    guard current.pixel(x, y) else { continue }
                    if zhangSuenStep1Check(current, x: x, y: y) {
                        toRemove.append((x, y))
                    }
                }
            }
            for (x, y) in toRemove {
                current.setPixel(x, y, false)
                changed = true
            }

            // Sub-iteration 2
            toRemove.removeAll()
            for y in 1..<(current.height - 1) {
                for x in 1..<(current.width - 1) {
                    guard current.pixel(x, y) else { continue }
                    if zhangSuenStep2Check(current, x: x, y: y) {
                        toRemove.append((x, y))
                    }
                }
            }
            for (x, y) in toRemove {
                current.setPixel(x, y, false)
                changed = true
            }
        }

        return current
    }

    /// Get 8-neighbor values for Zhang-Suen: P2..P9 in clockwise order from north.
    ///   P9 P2 P3
    ///   P8 P1 P4
    ///   P7 P6 P5
    private static func zhangSuenNeighbors(_ bitmap: BinaryBitmap, x: Int, y: Int) -> [Bool] {
        [
            bitmap.pixel(x, y - 1),      // P2 (north)
            bitmap.pixel(x + 1, y - 1),  // P3
            bitmap.pixel(x + 1, y),      // P4 (east)
            bitmap.pixel(x + 1, y + 1),  // P5
            bitmap.pixel(x, y + 1),      // P6 (south)
            bitmap.pixel(x - 1, y + 1),  // P7
            bitmap.pixel(x - 1, y),      // P8 (west)
            bitmap.pixel(x - 1, y - 1),  // P9
        ]
    }

    /// Count the number of 0->1 transitions in the ordered sequence P2..P9..P2.
    private static func transitions(_ neighbors: [Bool]) -> Int {
        var count = 0
        for i in 0..<neighbors.count {
            let next = (i + 1) % neighbors.count
            if !neighbors[i] && neighbors[next] {
                count += 1
            }
        }
        return count
    }

    /// Zhang-Suen sub-iteration 1 check.
    private static func zhangSuenStep1Check(_ bitmap: BinaryBitmap, x: Int, y: Int) -> Bool {
        let n = zhangSuenNeighbors(bitmap, x: x, y: y)
        let b = n.filter { $0 }.count  // number of non-zero neighbors
        guard b >= 2 && b <= 6 else { return false }
        guard transitions(n) == 1 else { return false }
        // P2 * P4 * P6 == 0  (at least one is background)
        let cond3 = !n[0] || !n[2] || !n[4]
        // P4 * P6 * P8 == 0
        let cond4 = !n[2] || !n[4] || !n[6]
        return cond3 && cond4
    }

    /// Zhang-Suen sub-iteration 2 check.
    private static func zhangSuenStep2Check(_ bitmap: BinaryBitmap, x: Int, y: Int) -> Bool {
        let n = zhangSuenNeighbors(bitmap, x: x, y: y)
        let b = n.filter { $0 }.count
        guard b >= 2 && b <= 6 else { return false }
        guard transitions(n) == 1 else { return false }
        // P2 * P4 * P8 == 0
        let cond3 = !n[0] || !n[2] || !n[6]
        // P2 * P6 * P8 == 0
        let cond4 = !n[0] || !n[4] || !n[6]
        return cond3 && cond4
    }

    // MARK: - Skeleton Chain Tracing

    /// Trace the skeleton bitmap into chains of connected pixels.
    /// Each chain is a sequence of pixel coordinates forming a path segment.
    private static func traceSkeletonChains(_ skeleton: BinaryBitmap) -> [[(Int, Int)]] {
        var chains: [[(Int, Int)]] = []
        var visited = BinaryBitmap(width: skeleton.width, height: skeleton.height)
        let directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
                          (1, 1), (1, -1), (-1, 1), (-1, -1)]

        // Find endpoint or junction pixels to use as chain starting points
        for y in 0..<skeleton.height {
            for x in 0..<skeleton.width {
                guard skeleton.pixel(x, y) && !visited.pixel(x, y) else { continue }

                // Count neighbors
                var neighborCount = 0
                for (dx, dy) in directions {
                    if skeleton.isValid(x + dx, y + dy) && skeleton.pixel(x + dx, y + dy) {
                        neighborCount += 1
                    }
                }

                // Start tracing from endpoints (1 neighbor) or isolated points (0 neighbors)
                // Also start from junction points (3+ neighbors) or unvisited regular points
                let isEndpoint = neighborCount <= 1
                let isJunction = neighborCount >= 3

                if isEndpoint || isJunction || !visited.pixel(x, y) {
                    let chain = traceChainFrom(
                        skeleton: skeleton,
                        visited: &visited,
                        startX: x,
                        startY: y,
                        directions: directions
                    )
                    if chain.count >= 2 {
                        chains.append(chain)
                    }
                }
            }
        }

        return chains
    }

    /// Trace a single chain from a starting pixel, following connected skeleton pixels.
    private static func traceChainFrom(
        skeleton: BinaryBitmap,
        visited: inout BinaryBitmap,
        startX: Int,
        startY: Int,
        directions: [(Int, Int)]
    ) -> [(Int, Int)] {
        var chain: [(Int, Int)] = [(startX, startY)]
        visited.setPixel(startX, startY, true)

        var curX = startX
        var curY = startY

        while true {
            var foundNext = false
            for (dx, dy) in directions {
                let nx = curX + dx
                let ny = curY + dy
                if skeleton.isValid(nx, ny) && skeleton.pixel(nx, ny) && !visited.pixel(nx, ny) {
                    visited.setPixel(nx, ny, true)
                    chain.append((nx, ny))
                    curX = nx
                    curY = ny
                    foundNext = true
                    break
                }
            }
            if !foundNext { break }
        }

        return chain
    }

    // MARK: - Contour to VectorPath Conversion

    /// Convert pixel-coordinate contours into VectorPath objects.
    /// Applies coordinate transformation (Y-flip), simplification, smoothing, and Bezier fitting.
    private static func contoursToVectorPaths(
        _ contours: [[(Int, Int)]],
        imageHeight: Int,
        config: TracingConfig,
        closed: Bool
    ) -> [VectorPath] {
        var paths: [VectorPath] = []

        for contour in contours {
            // Convert pixel coordinates to Vector2D.
            // Pixel (0,0) is top-left; Vector2D Y increases upward.
            // Scale: 1 pixel = 1mm.
            let points = contour.map { (px, py) in
                Vector2D(Double(px), Double(imageHeight - 1 - py))
            }

            // Simplify with Douglas-Peucker
            let simplified = douglasPeucker(points: points, tolerance: config.simplifyTolerance)

            // Check minimum path length
            let pathLength = computePolylineLength(simplified)
            guard pathLength >= config.minPathLength else { continue }
            guard simplified.count >= 2 else { continue }

            // Fit Bezier curves
            let segments = fitBezierCurves(
                points: simplified,
                cornerThreshold: config.cornerThreshold,
                smoothing: config.smoothing
            )

            let vectorPath = VectorPath(
                startPoint: simplified[0],
                segments: segments,
                isClosed: closed
            )
            paths.append(vectorPath)
        }

        return paths
    }

    // MARK: - Douglas-Peucker Simplification

    /// Simplify a polyline using the Douglas-Peucker recursive algorithm.
    /// Removes points that deviate less than `tolerance` from the connecting line segment.
    public static func douglasPeucker(points: [Vector2D], tolerance: Double) -> [Vector2D] {
        guard points.count > 2 else { return points }

        // Find the point with the maximum distance from the line (first -> last)
        let first = points[0]
        let last = points[points.count - 1]
        var maxDist = 0.0
        var maxIndex = 0

        for i in 1..<(points.count - 1) {
            let d = perpendicularDistance(point: points[i], lineStart: first, lineEnd: last)
            if d > maxDist {
                maxDist = d
                maxIndex = i
            }
        }

        if maxDist > tolerance {
            // Recursively simplify both halves
            let left = douglasPeucker(
                points: Array(points[0...maxIndex]),
                tolerance: tolerance
            )
            let right = douglasPeucker(
                points: Array(points[maxIndex..<points.count]),
                tolerance: tolerance
            )
            // Combine (avoid duplicating the split point)
            return Array(left.dropLast()) + right
        } else {
            // All intermediate points are within tolerance; keep only endpoints
            return [first, last]
        }
    }

    /// Perpendicular distance from a point to a line segment.
    public static func perpendicularDistance(
        point: Vector2D,
        lineStart: Vector2D,
        lineEnd: Vector2D
    ) -> Double {
        let dx = lineEnd.x - lineStart.x
        let dy = lineEnd.y - lineStart.y
        let lenSq = dx * dx + dy * dy

        if lenSq < 1e-12 {
            return point.distance(to: lineStart)
        }

        // Project point onto line, clamping to segment
        let t = max(0, min(1, ((point.x - lineStart.x) * dx + (point.y - lineStart.y) * dy) / lenSq))
        let projection = Vector2D(lineStart.x + t * dx, lineStart.y + t * dy)
        return point.distance(to: projection)
    }

    // MARK: - Bezier Curve Fitting

    /// Fit cubic Bezier curves to a sequence of points.
    /// Detects corners using the angle threshold, then fits smooth Bezier segments
    /// between each pair of corners.
    public static func fitBezierCurves(
        points: [Vector2D],
        cornerThreshold: Double,
        smoothing: Double
    ) -> [PathSegment] {
        guard points.count >= 2 else { return [] }

        if points.count == 2 {
            return [.lineTo(points[1])]
        }

        // Detect corner indices (including first and last)
        let corners = detectCorners(points: points, threshold: cornerThreshold)

        var segments: [PathSegment] = []

        for i in 0..<(corners.count - 1) {
            let startIdx = corners[i]
            let endIdx = corners[i + 1]
            let subPoints = Array(points[startIdx...endIdx])

            if subPoints.count <= 2 {
                // Too few points for a curve; use a line segment
                segments.append(.lineTo(subPoints.last!))
            } else if smoothing < 0.01 {
                // No smoothing: use line segments
                for j in 1..<subPoints.count {
                    segments.append(.lineTo(subPoints[j]))
                }
            } else {
                // Fit a cubic Bezier to this smooth section
                let bezierSegments = fitCubicBezierToPoints(subPoints, smoothing: smoothing)
                segments.append(contentsOf: bezierSegments)
            }
        }

        return segments
    }

    /// Detect corner points where the turning angle exceeds the threshold.
    /// Always includes the first and last point indices.
    public static func detectCorners(points: [Vector2D], threshold: Double) -> [Int] {
        var corners: [Int] = [0]

        for i in 1..<(points.count - 1) {
            let v1 = points[i] - points[i - 1]
            let v2 = points[i + 1] - points[i]

            let len1 = v1.length
            let len2 = v2.length
            guard len1 > 1e-12 && len2 > 1e-12 else { continue }

            let cosAngle = v1.dot(v2) / (len1 * len2)
            let clampedCos = max(-1.0, min(1.0, cosAngle))
            let angle = acos(clampedCos)

            // The deviation from straight (pi radians) is the turning angle
            let turningAngle = Double.pi - angle
            if turningAngle > threshold {
                corners.append(i)
            }
        }

        if corners.last != points.count - 1 {
            corners.append(points.count - 1)
        }

        return corners
    }

    /// Fit one or more cubic Bezier segments to a smooth sequence of points.
    /// Uses a least-squares approach to compute optimal control points.
    private static func fitCubicBezierToPoints(
        _ points: [Vector2D],
        smoothing: Double
    ) -> [PathSegment] {
        guard points.count >= 2 else { return [] }

        if points.count == 2 {
            return [.lineTo(points[1])]
        }

        if points.count == 3 {
            // For 3 points, create a single cubic through the midpoint
            let p0 = points[0]
            let p1 = points[1]
            let p2 = points[2]
            let c1 = p0 + (p1 - p0) * (2.0 / 3.0 * smoothing)
            let c2 = p2 + (p1 - p2) * (2.0 / 3.0 * smoothing)
            return [.cubicTo(control1: c1, control2: c2, end: p2)]
        }

        // For longer sequences, split into segments of manageable size
        // and fit each with a cubic Bezier
        let maxSegmentPoints = 20
        if points.count > maxSegmentPoints {
            return fitLongSequence(points, smoothing: smoothing, maxSegment: maxSegmentPoints)
        }

        // Parameterize by chord length
        let params = chordLengthParameterize(points)
        let p0 = points[0]
        let p3 = points[points.count - 1]

        // Estimate tangent directions at endpoints
        let tangent1 = estimateTangent(points: points, atStart: true)
        let tangent2 = estimateTangent(points: points, atStart: false)

        // Solve for control points using least-squares
        let (c1, c2) = solveBezierControlPoints(
            points: points,
            params: params,
            p0: p0,
            p3: p3,
            tangent1: tangent1,
            tangent2: tangent2,
            smoothing: smoothing
        )

        // Check the fit quality
        let maxError = computeBezierFitError(
            points: points,
            params: params,
            p0: p0,
            c1: c1,
            c2: c2,
            p3: p3
        )

        if maxError > 4.0 && points.count > 4 {
            // Split and re-fit each half
            let mid = points.count / 2
            let left = fitCubicBezierToPoints(Array(points[0...mid]), smoothing: smoothing)
            let right = fitCubicBezierToPoints(Array(points[mid..<points.count]), smoothing: smoothing)
            return left + right
        }

        return [.cubicTo(control1: c1, control2: c2, end: p3)]
    }

    /// Fit a long sequence by splitting into overlapping sub-sequences.
    private static func fitLongSequence(
        _ points: [Vector2D],
        smoothing: Double,
        maxSegment: Int
    ) -> [PathSegment] {
        var segments: [PathSegment] = []
        var i = 0

        while i < points.count - 1 {
            let end = min(i + maxSegment, points.count)
            let subPoints = Array(points[i..<end])
            let subSegments = fitCubicBezierToPoints(subPoints, smoothing: smoothing)
            segments.append(contentsOf: subSegments)
            i = end - 1  // overlap by one point for continuity
        }

        return segments
    }

    /// Compute chord-length parameterization for a sequence of points.
    /// Returns parameter values in [0, 1] proportional to cumulative distance.
    private static func chordLengthParameterize(_ points: [Vector2D]) -> [Double] {
        var params: [Double] = [0.0]
        var totalLength = 0.0

        for i in 1..<points.count {
            totalLength += points[i - 1].distance(to: points[i])
            params.append(totalLength)
        }

        // Normalize to [0, 1]
        if totalLength > 1e-12 {
            for i in 0..<params.count {
                params[i] /= totalLength
            }
        }

        return params
    }

    /// Estimate the tangent direction at the start or end of a point sequence.
    private static func estimateTangent(points: [Vector2D], atStart: Bool) -> Vector2D {
        if atStart {
            // Use first few points to estimate starting tangent
            let lookAhead = min(3, points.count - 1)
            let tangent = points[lookAhead] - points[0]
            return tangent.length > 1e-12 ? tangent.normalized : Vector2D(1, 0)
        } else {
            // Use last few points to estimate ending tangent
            let lookBack = max(0, points.count - 1 - 3)
            let tangent = points[points.count - 1] - points[lookBack]
            return tangent.length > 1e-12 ? tangent.normalized : Vector2D(1, 0)
        }
    }

    /// Solve for optimal Bezier control points using a least-squares fit.
    ///
    /// Given endpoints p0, p3 and tangent directions, finds c1, c2 that minimize
    /// the sum of squared distances from the data points to the curve.
    private static func solveBezierControlPoints(
        points: [Vector2D],
        params: [Double],
        p0: Vector2D,
        p3: Vector2D,
        tangent1: Vector2D,
        tangent2: Vector2D,
        smoothing: Double
    ) -> (Vector2D, Vector2D) {
        // Build the system: we want alpha1, alpha2 such that
        //   c1 = p0 + alpha1 * tangent1
        //   c2 = p3 - alpha2 * tangent2  (note: negative because tangent2 points away)
        // Minimize sum_i |B(t_i) - points[i]|^2

        var a11 = 0.0, a12 = 0.0, a22 = 0.0
        var b1 = 0.0, b2 = 0.0

        for i in 0..<points.count {
            let t = params[i]
            let mt = 1.0 - t

            // Bernstein basis values for the two middle control points
            let basis1 = 3.0 * mt * mt * t          // B_1(t)
            let basis2 = 3.0 * mt * t * t            // B_2(t)

            // A1 = basis1 * tangent1, A2 = basis2 * tangent2
            let a1x = basis1 * tangent1.x
            let a1y = basis1 * tangent1.y
            let a2x = basis2 * tangent2.x
            let a2y = basis2 * tangent2.y

            a11 += a1x * a1x + a1y * a1y
            a12 += a1x * a2x + a1y * a2y
            a22 += a2x * a2x + a2y * a2y

            // Right-hand side: point[i] minus contribution of fixed endpoints
            let basis0 = mt * mt * mt               // B_0(t)
            let basis3 = t * t * t                   // B_3(t)
            let residualX = points[i].x - basis0 * p0.x - basis3 * p3.x
            let residualY = points[i].y - basis0 * p0.y - basis3 * p3.y

            b1 += a1x * residualX + a1y * residualY
            b2 += a2x * residualX + a2y * residualY
        }

        // Solve the 2x2 system
        let det = a11 * a22 - a12 * a12
        var alpha1: Double
        var alpha2: Double

        if abs(det) > 1e-12 {
            alpha1 = (a22 * b1 - a12 * b2) / det
            alpha2 = (a11 * b2 - a12 * b1) / det
        } else {
            // Fallback: use chord length as the distance estimate
            let dist = p0.distance(to: p3) / 3.0
            alpha1 = dist
            alpha2 = dist
        }

        // Clamp to reasonable range to avoid wild control points
        let segLength = p0.distance(to: p3)
        let maxAlpha = segLength * 2.0
        alpha1 = max(0.0, min(alpha1, maxAlpha))
        alpha2 = max(0.0, min(alpha2, maxAlpha))

        // Apply smoothing factor
        alpha1 *= smoothing
        alpha2 *= smoothing

        // Ensure minimum handle length to avoid degenerate curves
        let minAlpha = segLength * 0.01
        if alpha1 < minAlpha { alpha1 = minAlpha }
        if alpha2 < minAlpha { alpha2 = minAlpha }

        let c1 = p0 + tangent1 * alpha1
        let c2 = p3 - tangent2 * alpha2

        return (c1, c2)
    }

    /// Compute the maximum fitting error of a cubic Bezier against the sample points.
    private static func computeBezierFitError(
        points: [Vector2D],
        params: [Double],
        p0: Vector2D,
        c1: Vector2D,
        c2: Vector2D,
        p3: Vector2D
    ) -> Double {
        var maxError = 0.0

        for i in 0..<points.count {
            let t = params[i]
            let curvePoint = evaluateCubicBezier(p0: p0, p1: c1, p2: c2, p3: p3, t: t)
            let error = points[i].distanceSquared(to: curvePoint)
            maxError = max(maxError, error)
        }

        return sqrt(maxError)
    }

    /// Evaluate a cubic Bezier curve at parameter t.
    public static func evaluateCubicBezier(
        p0: Vector2D,
        p1: Vector2D,
        p2: Vector2D,
        p3: Vector2D,
        t: Double
    ) -> Vector2D {
        let mt = 1.0 - t
        let mt2 = mt * mt
        let t2 = t * t
        return p0 * (mt2 * mt)
             + p1 * (3.0 * mt2 * t)
             + p2 * (3.0 * mt * t2)
             + p3 * (t2 * t)
    }

    // MARK: - Utility

    /// Compute the total length of a polyline defined by a sequence of points.
    public static func computePolylineLength(_ points: [Vector2D]) -> Double {
        var length = 0.0
        for i in 1..<points.count {
            length += points[i - 1].distance(to: points[i])
        }
        return length
    }
}
