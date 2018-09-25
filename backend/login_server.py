import grpc
import time


from concurrent import futures
import login_service
from proto import login_pb2_grpc

_ONE_DAY_IN_SECONDS = 60*60*24

def main():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    login_pb2_grpc.add_AuthenticationServiceServicer_to_server(login_service.LoginService(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    main()