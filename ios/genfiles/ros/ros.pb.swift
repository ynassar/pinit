// DO NOT EDIT.
//
// Generated by the Swift generator plugin for the protocol buffer compiler.
// Source: ros.proto
//
// For information on using the generated types, please see the documenation:
//   https://github.com/apple/swift-protobuf/

import Foundation
import SwiftProtobuf

// If the compiler emits an error on this type, it is because this file
// was generated by a version of the `protoc` Swift plug-in that is
// incompatible with the version of SwiftProtobuf to which you are linking.
// Please ensure that your are building against the same version of the API
// that was used to generate this file.
fileprivate struct _GeneratedWithProtocGenSwiftVersion: SwiftProtobuf.ProtobufAPIVersionCheck {
  struct _2: SwiftProtobuf.ProtobufAPIVersion_2 {}
  typealias Version = _2
}

struct RosToServerCommunication {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  /// An identifier for the ros node to send requests to.
  var robotName: String = String()

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

struct ServerToRosMappingRequest {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var requestType: ServerToRosMappingRequest.MappingRequestType = .startMapping

  var direction: ServerToRosMappingRequest.Direction = .forward

  var unknownFields = SwiftProtobuf.UnknownStorage()

  /// Describes a request for mapping to be sent to a recieving ROS node.
  enum MappingRequestType: SwiftProtobuf.Enum {
    typealias RawValue = Int
    case startMapping // = 0
    case stopMapping // = 1
    case direction // = 2
    case UNRECOGNIZED(Int)

    init() {
      self = .startMapping
    }

    init?(rawValue: Int) {
      switch rawValue {
      case 0: self = .startMapping
      case 1: self = .stopMapping
      case 2: self = .direction
      default: self = .UNRECOGNIZED(rawValue)
      }
    }

    var rawValue: Int {
      switch self {
      case .startMapping: return 0
      case .stopMapping: return 1
      case .direction: return 2
      case .UNRECOGNIZED(let i): return i
      }
    }

  }

  enum Direction: SwiftProtobuf.Enum {
    typealias RawValue = Int
    case forward // = 0
    case backward // = 1
    case right // = 2
    case left // = 3
    case UNRECOGNIZED(Int)

    init() {
      self = .forward
    }

    init?(rawValue: Int) {
      switch rawValue {
      case 0: self = .forward
      case 1: self = .backward
      case 2: self = .right
      case 3: self = .left
      default: self = .UNRECOGNIZED(rawValue)
      }
    }

    var rawValue: Int {
      switch self {
      case .forward: return 0
      case .backward: return 1
      case .right: return 2
      case .left: return 3
      case .UNRECOGNIZED(let i): return i
      }
    }

  }

  init() {}
}

#if swift(>=4.2)

extension ServerToRosMappingRequest.MappingRequestType: CaseIterable {
  // The compiler won't synthesize support with the UNRECOGNIZED case.
  static var allCases: [ServerToRosMappingRequest.MappingRequestType] = [
    .startMapping,
    .stopMapping,
    .direction,
  ]
}

extension ServerToRosMappingRequest.Direction: CaseIterable {
  // The compiler won't synthesize support with the UNRECOGNIZED case.
  static var allCases: [ServerToRosMappingRequest.Direction] = [
    .forward,
    .backward,
    .right,
    .left,
  ]
}

#endif  // swift(>=4.2)

struct ServerToRosCommunication {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var communication: OneOf_Communication? {
    get {return _storage._communication}
    set {_uniqueStorage()._communication = newValue}
  }

  var mappingRequest: ServerToRosMappingRequest {
    get {
      if case .mappingRequest(let v)? = _storage._communication {return v}
      return ServerToRosMappingRequest()
    }
    set {_uniqueStorage()._communication = .mappingRequest(newValue)}
  }

  var unknownFields = SwiftProtobuf.UnknownStorage()

  enum OneOf_Communication: Equatable {
    case mappingRequest(ServerToRosMappingRequest)

  #if !swift(>=4.1)
    static func ==(lhs: ServerToRosCommunication.OneOf_Communication, rhs: ServerToRosCommunication.OneOf_Communication) -> Bool {
      switch (lhs, rhs) {
      case (.mappingRequest(let l), .mappingRequest(let r)): return l == r
      }
    }
  #endif
  }

  init() {}

  fileprivate var _storage = _StorageClass.defaultInstance
}

struct MappingRequest {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  /// Describes a request for mapping to be sent to the RPC server and
  /// forwarded onto a specified ros instance.
  var robotName: String {
    get {return _storage._robotName}
    set {_uniqueStorage()._robotName = newValue}
  }

  var mappingRequest: ServerToRosMappingRequest {
    get {return _storage._mappingRequest ?? ServerToRosMappingRequest()}
    set {_uniqueStorage()._mappingRequest = newValue}
  }
  /// Returns true if `mappingRequest` has been explicitly set.
  var hasMappingRequest: Bool {return _storage._mappingRequest != nil}
  /// Clears the value of `mappingRequest`. Subsequent reads from it will return its default value.
  mutating func clearMappingRequest() {_uniqueStorage()._mappingRequest = nil}

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}

  fileprivate var _storage = _StorageClass.defaultInstance
}

struct MappingResponse {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

// MARK: - Code below here is support for the SwiftProtobuf runtime.

extension RosToServerCommunication: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = "RosToServerCommunication"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "robot_name"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      switch fieldNumber {
      case 1: try decoder.decodeSingularStringField(value: &self.robotName)
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if !self.robotName.isEmpty {
      try visitor.visitSingularStringField(value: self.robotName, fieldNumber: 1)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: RosToServerCommunication, rhs: RosToServerCommunication) -> Bool {
    if lhs.robotName != rhs.robotName {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension ServerToRosMappingRequest: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = "ServerToRosMappingRequest"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "request_type"),
    2: .same(proto: "direction"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      switch fieldNumber {
      case 1: try decoder.decodeSingularEnumField(value: &self.requestType)
      case 2: try decoder.decodeSingularEnumField(value: &self.direction)
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if self.requestType != .startMapping {
      try visitor.visitSingularEnumField(value: self.requestType, fieldNumber: 1)
    }
    if self.direction != .forward {
      try visitor.visitSingularEnumField(value: self.direction, fieldNumber: 2)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: ServerToRosMappingRequest, rhs: ServerToRosMappingRequest) -> Bool {
    if lhs.requestType != rhs.requestType {return false}
    if lhs.direction != rhs.direction {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension ServerToRosMappingRequest.MappingRequestType: SwiftProtobuf._ProtoNameProviding {
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    0: .same(proto: "START_MAPPING"),
    1: .same(proto: "STOP_MAPPING"),
    2: .same(proto: "DIRECTION"),
  ]
}

extension ServerToRosMappingRequest.Direction: SwiftProtobuf._ProtoNameProviding {
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    0: .same(proto: "FORWARD"),
    1: .same(proto: "BACKWARD"),
    2: .same(proto: "RIGHT"),
    3: .same(proto: "LEFT"),
  ]
}

extension ServerToRosCommunication: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = "ServerToRosCommunication"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "mapping_request"),
  ]

  fileprivate class _StorageClass {
    var _communication: ServerToRosCommunication.OneOf_Communication?

    static let defaultInstance = _StorageClass()

    private init() {}

    init(copying source: _StorageClass) {
      _communication = source._communication
    }
  }

  fileprivate mutating func _uniqueStorage() -> _StorageClass {
    if !isKnownUniquelyReferenced(&_storage) {
      _storage = _StorageClass(copying: _storage)
    }
    return _storage
  }

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    _ = _uniqueStorage()
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      while let fieldNumber = try decoder.nextFieldNumber() {
        switch fieldNumber {
        case 1:
          var v: ServerToRosMappingRequest?
          if let current = _storage._communication {
            try decoder.handleConflictingOneOf()
            if case .mappingRequest(let m) = current {v = m}
          }
          try decoder.decodeSingularMessageField(value: &v)
          if let v = v {_storage._communication = .mappingRequest(v)}
        default: break
        }
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      if case .mappingRequest(let v)? = _storage._communication {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 1)
      }
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: ServerToRosCommunication, rhs: ServerToRosCommunication) -> Bool {
    if lhs._storage !== rhs._storage {
      let storagesAreEqual: Bool = withExtendedLifetime((lhs._storage, rhs._storage)) { (_args: (_StorageClass, _StorageClass)) in
        let _storage = _args.0
        let rhs_storage = _args.1
        if _storage._communication != rhs_storage._communication {return false}
        return true
      }
      if !storagesAreEqual {return false}
    }
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension MappingRequest: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = "MappingRequest"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "robot_name"),
    2: .standard(proto: "mapping_request"),
  ]

  fileprivate class _StorageClass {
    var _robotName: String = String()
    var _mappingRequest: ServerToRosMappingRequest? = nil

    static let defaultInstance = _StorageClass()

    private init() {}

    init(copying source: _StorageClass) {
      _robotName = source._robotName
      _mappingRequest = source._mappingRequest
    }
  }

  fileprivate mutating func _uniqueStorage() -> _StorageClass {
    if !isKnownUniquelyReferenced(&_storage) {
      _storage = _StorageClass(copying: _storage)
    }
    return _storage
  }

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    _ = _uniqueStorage()
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      while let fieldNumber = try decoder.nextFieldNumber() {
        switch fieldNumber {
        case 1: try decoder.decodeSingularStringField(value: &_storage._robotName)
        case 2: try decoder.decodeSingularMessageField(value: &_storage._mappingRequest)
        default: break
        }
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      if !_storage._robotName.isEmpty {
        try visitor.visitSingularStringField(value: _storage._robotName, fieldNumber: 1)
      }
      if let v = _storage._mappingRequest {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 2)
      }
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: MappingRequest, rhs: MappingRequest) -> Bool {
    if lhs._storage !== rhs._storage {
      let storagesAreEqual: Bool = withExtendedLifetime((lhs._storage, rhs._storage)) { (_args: (_StorageClass, _StorageClass)) in
        let _storage = _args.0
        let rhs_storage = _args.1
        if _storage._robotName != rhs_storage._robotName {return false}
        if _storage._mappingRequest != rhs_storage._mappingRequest {return false}
        return true
      }
      if !storagesAreEqual {return false}
    }
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension MappingResponse: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = "MappingResponse"
  static let _protobuf_nameMap = SwiftProtobuf._NameMap()

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let _ = try decoder.nextFieldNumber() {
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: MappingResponse, rhs: MappingResponse) -> Bool {
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}
