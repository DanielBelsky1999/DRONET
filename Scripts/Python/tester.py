import socket
from time import sleep


SEND_TO_ADDR = ("127.0.0.1",25001)

UDPSendingSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        
def GUI_send(i, lat, long, alt, az, pitch, roll):
    message_string = "GUI,POS,{jetson_indx},{LAT},{LON},{ALT},{PSI},{THETA},{PHI};".format(
                               jetson_indx= i,
                               LAT = lat,
                               LON = long,
                               ALT = alt,
                               PSI = az,
                               THETA =  pitch,
                               PHI = roll)
    
    bytesToSend = message_string.encode()
    UDPSendingSocket.sendto(bytesToSend, SEND_TO_ADDR)
    
def STATION_send(i, px1, px2):
    message_string = "STATION,{jetson_indx},{_px1},{_px2};".format(
                               jetson_indx= i,
                               _px1 = px1,
                               _px2 = px2)
    
    bytesToSend = message_string.encode()
    UDPSendingSocket.sendto(bytesToSend, SEND_TO_ADDR)
 
def TestSend(px1, px2):
    message_string = "TEST,{_px1},{_px2};".format(
                               _px1 = px1,
                               _px2 = px2)
    
    bytesToSend = message_string.encode()
    UDPSendingSocket.sendto(bytesToSend, SEND_TO_ADDR)

# GUI_send(0, 32, 35, 240, 0, 0, 0)
# input("sent")
# GUI_send(1, 32.00005, 35, 240, 90, 0, 0)
# input("sent")
# GUI_send(2, 32.00010, 35, 240, 90, 0, 0)
# input("sent")
# GUI_send(3, 32.00015, 35, 240, 90, 0, 0)
# input("sent")
# GUI_send(4, 32.00018, 35, 240, 90, 0, 0)
# input("sent")

# while True:
    # STATION_send(1, 1920, 1080)
    # sleep(0.01)
    # STATION_send(3, 0, 0)
    # sleep(0.01)
    # STATION_send(2, 352, 468)
    # sleep(0.01)
    # STATION_send(4, 1568, 12)
    # sleep(0.01)
    # input("Asd")
    
    # # # #STATION_send(2, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(1, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(3, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(4, 1920, 1080)
    # # # # sleep(0.01)
    # # # # input("asd")
    
    # # # # STATION_send(4, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(3, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(2, 1920, 1080)
    # # # # sleep(0.01)
    # # # # STATION_send(1, 1920, 1080)
    # # # # sleep(0.01)
    # # # input("asd")
    


# TestSend(1,2)
# TestSend(4,2)
# TestSend(8,2)
# TestSend(8,4)
# TestSend(3,2)
# TestSend(1,1985)
# TestSend(1,1985)
# TestSend(1,1985)
# TestSend(1,1985)TestSend(1,2)
TestSend(0, 0)
TestSend(1920,1080)
TestSend(8,4)
TestSend(3,2)
TestSend(5,890)
TestSend(242,153)
TestSend(1590,1000)
TestSend(1645,10)


print("Done")

