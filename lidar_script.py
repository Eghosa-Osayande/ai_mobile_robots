import socket
import time
from traceback import print_stack

host = "0.0.0.0"
port = 9999

while True:
    try:
        with socket.create_connection((host, port), timeout=5) as s:
            print(f"connected to {host}:{port}")

            while True:
                try:
                    cmd = input("cmd> ").strip()
                except EOFError:
                    break

                if not cmd:
                    continue
                if cmd in {"0"}:
                    raise 0

                if cmd in {"exit", "q"}:
                    break

                msg = cmd + "\n"
                s.sendall(msg.encode("utf-8"))
    except:
        time.sleep(1)
        print("reconnect")
        ...
