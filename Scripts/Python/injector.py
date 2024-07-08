import csv
import socket
from time import sleep


SEND_TO_ADDR = ("127.0.0.1",25001)

UDPSendingSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


def STATION_send(i, px1, px2):
    message_string = "STATION,{jetson_indx},{_px1},{_px2};".format(
                               jetson_indx= i,
                               _px1 = px1,
                               _px2 = px2)
    
    bytesToSend = message_string.encode()
    UDPSendingSocket.sendto(bytesToSend, SEND_TO_ADDR)

with open('..//MATLAB//injection_data.csv', mode ='r')as file:
    csvfile = csv.reader(file)
    i=1
    for line in csvfile:
        cam_id = int(line[0])
        pix_1 = int(float(line[1]))
        pix_2 = int(float(line[2]))
        print(cam_id, pix_1, pix_2)
        
        STATION_send(cam_id, pix_1, pix_2)
        sleep(0.01)
        
        