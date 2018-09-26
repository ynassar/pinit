# Runs the protocol buffer compiler.
# Invoke as "bash run_codegen.sh".
# TODO: Make this a python script.

# This will produce generated proto files in directories under root,
# corresponding to directores under proto/
python3 -m grpc_tools.protoc -I proto \
--python_out=. --grpc_python_out=. proto/login_proto/*.proto
python3 -m grpc_tools.protoc -I proto \
--python_out=. --grpc_python_out=. proto/account_management_proto/*.proto