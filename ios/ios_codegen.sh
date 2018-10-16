protoc --swift_out=genfiles/account_management --swiftgrpc_out=genfiles/account_management --proto_path=../proto/account_management ../proto/account_management/*.proto
protoc --swift_out=genfiles/login --swiftgrpc_out=genfiles/login --proto_path=../proto/login ../proto/login/*.proto
protoc --swift_out=genfiles/ros --swiftgrpc_out=genfiles/ros --proto_path=../proto/ros ../proto/ros/*.proto