import Foundation
import Testing
@testable import ClaudeCarveCore
@testable import ClaudeCarveIO

@Suite("SVG Importer Tests")
struct SVGImporterTests {
    @Test func importSimplePath() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <path d="M 0 0 L 100 0 L 100 100 L 0 100 Z"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }

    @Test func importCubicBezier() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <path d="M 10 80 C 40 10, 65 10, 95 80"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(!paths[0].isClosed)
    }

    @Test func importRect() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <rect x="10" y="20" width="100" height="50"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }

    @Test func importCircle() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <circle cx="50" cy="50" r="25"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }

    @Test func importLine() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <line x1="0" y1="0" x2="100" y2="100"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(!paths[0].isClosed)
    }

    @Test func importPolygon() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <polygon points="50,0 100,100 0,100"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }

    @Test func importMultiplePaths() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <rect x="0" y="0" width="50" height="50"/>
          <circle cx="100" cy="100" r="20"/>
          <line x1="0" y1="200" x2="200" y2="200"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 3)
    }

    @Test func importRelativeMoveTo() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <path d="m 10 10 l 50 0 l 0 50 l -50 0 z"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }

    @Test func importHorizontalVerticalLines() throws {
        let svg = """
        <svg xmlns="http://www.w3.org/2000/svg">
          <path d="M 0 0 H 100 V 50 H 0 Z"/>
        </svg>
        """

        let importer = SVGImporter()
        let paths = try importer.importData(Data(svg.utf8))

        #expect(paths.count == 1)
        #expect(paths[0].isClosed)
    }
}

@Suite("SVG Exporter Tests")
struct SVGExporterTests {
    @Test func exportBasicPath() {
        let path = VectorPath(
            startPoint: Vector2D(0, 0),
            segments: [
                .lineTo(Vector2D(100, 0)),
                .lineTo(Vector2D(100, 100)),
                .lineTo(Vector2D(0, 100)),
                .lineTo(Vector2D(0, 0)),
            ],
            isClosed: true
        )

        let bounds = BoundingBox2D(min: Vector2D(0, 0), max: Vector2D(200, 200))
        let exporter = SVGExporter()
        let svg = exporter.export(paths: [path], materialBounds: bounds)

        #expect(svg.contains("<svg"))
        #expect(svg.contains("<path"))
        #expect(svg.contains("</svg>"))
        #expect(svg.contains(" Z"))
    }

    @Test func exportIncludesMaterialBounds() {
        let bounds = BoundingBox2D(min: Vector2D(0, 0), max: Vector2D(300, 200))
        let exporter = SVGExporter()
        let svg = exporter.export(paths: [], materialBounds: bounds, showMaterialBounds: true)

        #expect(svg.contains("<rect"))
        #expect(svg.contains("300.0000"))
    }
}
