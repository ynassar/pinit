# Runs the protocol buffer compiler.
# Invoke as "bash run_codegen.sh".
# TODO: Make this a python script.

python3 -m grpc_tools.protoc -I proto/login \
--python_out=proto/login --grpc_python_out=proto/login proto/login/*.proto
python3 -m grpc_tools.protoc -I proto/account_management \
--python_out=proto/account_management --grpc_python_out=proto/account_management proto/account_management/*.proto
# Run 2to3 to fix auto-generated imports.
2to3 ./proto -w -n > /dev/null --fix=import