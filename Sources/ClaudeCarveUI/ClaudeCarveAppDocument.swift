import SwiftUI
import UniformTypeIdentifiers
import ClaudeCarveCore

/// The document type for ClaudeCarve files.
public struct ClaudeCarveAppDocument: FileDocument {
    public var document: ClaudeCarveDocument

    public static var readableContentTypes: [UTType] { [.claudecarveDocument, .json] }
    public static var writableContentTypes: [UTType] { [.claudecarveDocument, .json] }

    public init(document: ClaudeCarveDocument = ClaudeCarveDocument()) {
        self.document = document
    }

    public init(configuration: ReadConfiguration) throws {
        guard let data = configuration.file.regularFileContents else {
            throw CocoaError(.fileReadCorruptFile)
        }

        let decoder = JSONDecoder()
        self.document = try decoder.decode(ClaudeCarveDocument.self, from: data)
    }

    public func fileWrapper(configuration: WriteConfiguration) throws -> FileWrapper {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        let data = try encoder.encode(document)
        return FileWrapper(regularFileWithContents: data)
    }
}

extension UTType {
    /// ClaudeCarve document type (.claudecarve)
    public static var claudecarveDocument: UTType {
        UTType(exportedAs: "com.claudecarve.macos.document", conformingTo: .json)
    }
}
