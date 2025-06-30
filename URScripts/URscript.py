# Sending URScript
import socket

robotIP = "192.168.1.102"
SECONDARY_PORT = 30002

def send_urscript_command(command: str):

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((robotIP, SECONDARY_PORT))
            command = command + "\n"
            s.sendall(command.encode('utf-8'))
    
    except Exception as e:
        print(f"An error occurred: {e}")

send_urscript_command("set_standard_digital_out(3, True)")
send_urscript_command("set_standard_digital_out(2, False)")
#send_urscript_command("movel(p[0, 0.57, 0.57, 0.0, 0.67, 0.0])")
