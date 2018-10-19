# Runs the protocol buffer compiler.
# Invoke as "bash run_codegen.sh".
# TODO: Make this a python script.

source .env/backend/bin/activate

python3 -m grpc_tools.protoc -I proto/login \
--python_out=proto/login --grpc_python_out=proto/login proto/login/*.proto
python3 -m grpc_tools.protoc -I proto/account_management \
--python_out=proto/account_management --grpc_python_out=proto/account_management proto/account_management/*.proto
python3 -m grpc_tools.protoc -I proto/ros \
--python_out=proto/ros --grpc_python_out=proto/ros proto/ros/*.proto

# Run 2to3 to fix auto-generated imports.
2to3 ./proto -w -n > /dev/null --fix=import

if [[ -x "$(commnd -v protoc-gen-swift)" ]] && [[ -x "$(command -v protoc-gen-swiftgrpc)" ]]
then
    echo "Swift protobuf compiler detected. Generating swift code."
    protoc --swift_out=ios/genfiles/account_management --swiftgrpc_out=ios/genfiles/account_management --proto_path=proto/account_management proto/account_management/*.proto
    protoc --swift_out=ios/genfiles/login --swiftgrpc_out=ios/genfiles/login --proto_path=proto/login proto/login/*.proto
    protoc --swift_out=ios/genfiles/ros --swiftgrpc_out=ios/genfiles/ros --proto_path=proto/ros proto/ros/*.proto
fi

deactivate
