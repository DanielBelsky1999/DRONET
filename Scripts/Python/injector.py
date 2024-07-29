import csv
import socket
import time


SEND_TO_ADDR = ("127.0.0.1",25001)

UDPSendingSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


def STATION_send(i, px1, px2, time_stamp):
    message_string = "STATION,{jetson_indx},{_px1},{_px2},{_time_stamp};".format(
                               jetson_indx= i,
                               _px1 = px1,
                               _px2 = px2,
                               _time_stamp = time_stamp)
    
    bytesToSend = message_string.encode()
    UDPSendingSocket.sendto(bytesToSend, SEND_TO_ADDR)


with open('Scripts//MATLAB//InjectedData//injection_data.csv', mode ='r')as file:
    csvfile = csv.reader(file)
    i=1
    for line in csvfile:
        cam_id = int(line[0])
        pix_1 = int(float(line[1]))
        pix_2 = int(float(line[2]))
        time_stamp = float(line[3])
        print(cam_id, pix_1, pix_2, time_stamp)
        
        STATION_send(cam_id, pix_1, pix_2, time_stamp)
        
        time.sleep(0.01)
        
        
input("PRESS ENTER TO CLOSE injector.py")