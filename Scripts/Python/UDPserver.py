import socket
# from GuiSender import GuiSender
 
UDP_IP = "0.0.0.0"
UDP_PORT = 30001
      


server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_IP, UDP_PORT))


while True:
    print("Waiting")
    bytesAddressPair = server_socket.recvfrom(1024)
    message = bytesAddressPair[0]
    raw_data = message.decode()
    ## parser
    
    ##
    # GuiSender.SendXYZ(px_x,px_y,0)
    print(raw_data)