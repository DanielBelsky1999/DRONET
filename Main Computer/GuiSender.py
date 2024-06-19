import socket
import math


class GuiSender:
    '''
    Class GuiSender with STATIC Methods to update the GUI
    '''
    
    ADDR   = ("127.0.0.1", 20001)
 
    # Create a UDP socket at client side
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def SendJetsonPosition(jetson_indx, Position_vector, LOS_vector):
        message = "JETSON_POS,{indx},{x},{y},{vec1},{vec2};".format(indx=jetson_indx, 
                                                                        x=Position_vector[0], 
                                                                        y=Position_vector[1], 
                                                                        vec1=LOS_vector[0], 
                                                                        vec2=LOS_vector[1])
        GuiSender._SendString(message)
        # print("Sent: ",message)
    
    def SendJetsonPositionStatus(jetson_indx, status_str):
        message = "POS_ORI_STATUS,{indx},{status_STR};".format(indx = jetson_indx, status_STR=status_str)
        GuiSender._SendString(message)
        # print("Sent: ",message)        
        
    def SendConnectionStatus(jetson_indx, status_str):
        message = "CONNECTION_STATUS,{indx},{status_STR};".format(indx = jetson_indx, status_STR=status_str)
        GuiSender._SendString(message)   
        # print("Sent: ",message)        
    
    def SendOriginSet():
        message = "ORIGIN_SET;"
        GuiSender._SendString(message)
        
    def SendXYZ(X,Y,Z):
        message = "XYZ,{x},{y},{z};".format(x=X, y=Y, z=Z)
        GuiSender._SendString(message)
    
    def SendDOP(DOP):
        message = "DOP,{DOP};".format(DOP=DOP)
        GuiSender._SendString(message)
        
    def SendRESET():
        message = "RESET;"
        GuiSender._SendString(message)
        
    def _SendString(message_string):
        bytesToSend = message_string.encode()
        GuiSender.UDPClientSocket.sendto(bytesToSend, GuiSender.ADDR)
        #print("Sent: ",bytesToSend.decode())
        