#!/usr/bin/env python3
import socket, time

HOST, PORT = "192.168.0.20", 2111
STX, ETX   = b"\x02", b"\x03"

def send(sock, cmd, expect_reply=True, timeout=2.0):
    sock.settimeout(timeout)
    sock.sendall(STX + cmd.encode("ascii") + ETX)
    if not expect_reply:
        return b""
    try:
        data = sock.recv(65535)
        return data
    except socket.timeout:
        return b""

def setup_once():
    with socket.create_connection((HOST, PORT), timeout=3) as s:
        print("Login...")
        print(send(s, "sMN SetAccessMode 03 F4724744").decode(errors="ignore"))
        print("Apply LMDscandatacfg...")
        print(send(s, "sWN LMDscandatacfg 01 00 1 1 0 00 00 0 1 0 0 +1").decode(errors="ignore"))
        print("Save...")
        print(send(s, "sMN mEEwriteall").decode(errors="ignore"))
        print("Run...")
        print(send(s, "sMN Run").decode(errors="ignore"))

def stream_demo(seconds=3):
    with socket.create_connection((HOST, PORT), timeout=3) as s:
        print(send(s, "sRN DeviceIdent").decode(errors="ignore"))
        print("Enable stream...")
        send(s, "sEN LMDscandata 1", expect_reply=False)
        t0 = time.time()
        s.settimeout(2.0)
        leftover = b""
        try:
            while time.time() - t0 < seconds:
                try:
                    chunk = s.recv(65535)
                    if not chunk: break
                    text = (leftover + chunk).decode("ascii", errors="ignore")
                    # Print raw; you should see sRA LMDscandata lines
                    print(text.strip())
                    leftover = b""
                except socket.timeout:
                    pass
        finally:
            print("Disable stream...")
            try:
                send(s, "sEN LMDscandata 0", expect_reply=False)
            except Exception:
                pass

if __name__ == "__main__":
    # Run setup_once() the first time (or any time you change config)
    # setup_once()
    stream_demo(5)
