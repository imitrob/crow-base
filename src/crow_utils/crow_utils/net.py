import socket
import time

def wait_for_open(*, ip, port):

    while True:
        print(f"Waiting for {ip}:{port}...")

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = int(sock.connect_ex((ip,port)))
        sock.close()

        if result == 0:
            return
        
        time.sleep(1)