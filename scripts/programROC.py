import socket
import time
import sys
import os
import select
import argparse

# --- Constants for Reliability ---
# Speed Limit: ~25 KB/s (1024 bytes every 0.04s)
CHUNK_SIZE = 1024
SLEEP_PER_CHUNK = 0.04 

def create_parser():
    parser = argparse.ArgumentParser(description="Upload firmware via TCP Socket (Silent Mode)")
    
    # Required Arguments
    parser.add_argument("file", help="Path to the binary firmware file (.bin)")
    parser.add_argument("-f", "--flash", type=int, choices=[1, 2, 3], required=True,
                        help="Flash Target: 1=FLSOCK1, 2=FLSOCK2, 3=FLSOCK3")

    # Optional Arguments (with Defaults)
    parser.add_argument("-i", "--ip", default="192.168.157.91", 
                        help="Target IP Address (default: 192.168.157.91)")
    parser.add_argument("-p", "--port", type=int, default=5002, 
                        help="Target Port (default: 5002)")
    
    return parser

def main():
    parser = create_parser()
    args = parser.parse_args()

    # 1. Validation
    if not os.path.exists(args.file):
        print(f"Error: File not found: {args.file}")
        sys.exit(1)
        
    file_size = os.path.getsize(args.file)
    command_str = f"FLSOCK{args.flash}"
    command_bytes = f"{command_str}\r".encode()
    
    # Calculate stats
    speed_kbs = (CHUNK_SIZE / SLEEP_PER_CHUNK) / 1000
    print(f"--- Configuration ---")
    print(f"Target:   {args.ip}:{args.port}")
    print(f"File:     {args.file} ({file_size} bytes)")
    print(f"Command:  {command_str}")
    print(f"Speed:    {speed_kbs:.1f} kB/s (Throttled for safety)")
    print(f"---------------------")

    # 2. Load File
    try:
        with open(args.file, 'rb') as f:
            binaryData = f.read()
    except IOError as e:
        print(f"Error reading file: {e}")
        sys.exit(1)

    # 3. Connect
    print(f"Connecting...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # TCP_NODELAY is critical: it forces OS to send small packets immediately 
    # rather than bunching them up (which causes bursts that crash the bridge).
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) 
    sock.settimeout(0.5)

    try:
        sock.connect((args.ip, args.port))
    except socket.error as e:
        print(f"Connection Failed: {e}")
        sys.exit(1)

    # 4. Send Command
    print(f"Sending {command_str}...")
    sock.sendall(command_bytes)

    # 5. Wait for flash erase
    print("Waiting for flash erase...")
    trigger_found = False
    for k in range(100):
        try:
            chunk = sock.recv(1024)
            if not chunk: break
            
            text = chunk.decode('utf-8', errors='ignore')
            print(text, end="", flush=True) 
            
            if "Waiting for data" in text or "Begin Load Flash" in text:
                print("\n>> Ready!")
                trigger_found = True
                break
        except socket.timeout:
            pass
        time.sleep(0.5)

    if not trigger_found:
        print("\nTimeout waiting for flash erase.")
        sock.close()
        sys.exit(1)

    # 6. Stream Loop (Silent Mode)
    print(f"Streaming file...")
    total_sent = 0

    try:
        for i in range(0, file_size, CHUNK_SIZE):
            chunk = binaryData[i : i + CHUNK_SIZE]
            
            # A. SILENT DRAIN 
            # Read and discard any echo/garbage from the device to keep the TCP window open.
            readable, _, _ = select.select([sock], [], [], 0)
            if readable:
                try:
                    sock.recv(4096) 
                except socket.error:
                    pass

            # B. SEND
            sock.sendall(chunk)
            total_sent += len(chunk)
            
            # C. PROGRESS (Every 20 chunks / ~20KB)
            if i % (CHUNK_SIZE * 20) == 0:
                pct = (total_sent / file_size) * 100
                sys.stdout.write(f"\rProgress: {pct:.1f}%")
                sys.stdout.flush() 
            
            # D. THROTTLE
            time.sleep(SLEEP_PER_CHUNK)

        print(f"\rProgress: 100.0%")
        print("Upload Complete.")

        # 7. Reset and Read Final Response
        print("Finalizing (1s pause)...")
        time.sleep(1.0)
        
        print("Sending RESET...")
        sock.sendall(b"RESET\r")
        
        print("--- Device Response ---")
        # Temporarily increase timeout to allow device to process the reset
        sock.settimeout(2.0)
        try:
            while True:
                # Keep reading until the device stops talking or closes connection
                final_chunk = sock.recv(1024)
                if not final_chunk: 
                    break # Connection closed by remote
                print(final_chunk.decode('utf-8', errors='ignore'), end='', flush=True)
        except socket.timeout:
            # This is expected if the device stays silent after the message
            print("\n(End of response)")
        
        print("\nDone.")

    except Exception as e:
        print(f"\n\n[CRASH] Error at byte {total_sent}: {e}")

    finally:
        sock.close()

if __name__ == "__main__":
    main()
