import SwiftUI
import UniformTypeIdentifiers
import VCarveCore

/// The document type for VCarve files.
public struct VCarveAppDocument: FileDocument {
    public var document: VCarveDocument

    public static var readableContentTypes: [UTType] { [.vcarveDocument, .json] }
    public static var writableContentTypes: [UTType] { [.vcarveDocument, .json] }

    public init(document: VCarveDocument = VCarveDocument()) {
        self.document = document
    }

    public init(configuration: ReadConfiguration) throws {
        guard let data = configuration.file.regularFileContents else {
            throw CocoaError(.fileReadCorruptFile)
        }

        let decoder = JSONDecoder()
        self.document = try decoder.decode(VCarveDocument.self, from: data)
    }

    public func fileWrapper(configuration: WriteConfiguration) throws -> FileWrapper {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        let data = try encoder.encode(document)
        return FileWrapper(regularFileWithContents: data)
    }
}

extension UTType {
    /// VCarve document type (.vcarve)
    public static var vcarveDocument: UTType {
        UTType(exportedAs: "com.vcarve.macos.document", conformingTo: .json)
    }
}
