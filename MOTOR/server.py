import socket

def start_server(host='127.0.0.1', port=5005):
    """ UDP 소켓 서버를 시작하는 함수 """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((host, port))
        print(f"서버가 {host}:{port}에서 시작되었습니다.")

        while True:
            data, addr = server_socket.recvfrom(1024)  # 데이터 수신
            print(f"수신한 데이터: {data.decode()} from {addr}")

if __name__ == "__main__":
    start_server()
