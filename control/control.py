


# echo-server.py

import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        i = 0
        while True:
            data = conn.recv(13)
            print("looping")
            if not data:
                break
            i += 1
            print("received: ", data)
            bti = lambda x: int.from_bytes(data[x:x+2], 'big', signed=True) # Bytes to integers.
            itb = lambda x: int(x).to_bytes(2, 'big', signed=True) # Integer to bytes
            print(bti(3), bti(5), bti(7), bti(9), bti(11))
            #data = b'mm2' + itb(bti(3)) + itb(bti(5)) + itb(bti(7)) + itb(bti(9)) + itb(bti(11))
            conn.sendall(data)
            print("sent: ", data)