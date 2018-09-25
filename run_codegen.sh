# Runs the protocol buffer compiler.
# TODO: Make this a python script.
python3 -m grpc_tools.protoc -I ./proto --python_out=./proto --grpc_python_out=./proto ./proto/*.proto