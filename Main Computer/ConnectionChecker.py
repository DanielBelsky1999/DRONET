import time
from threading import Lock  

class ConnectionChecker:
    
    WaitTime = 5  # [seconds]
    
    LOCK = Lock()
     
    def __init__(self, num_of_stations):
        self.num_of_stations = num_of_stations
        self.station_status = [0 for i in range(num_of_stations)]
        self.last_update_time = [0 for i in range(num_of_stations)]
        
        
    def NewMessage(self, jetson_id):
        with ConnectionChecker.LOCK:
            self.last_update_time[jetson_id - 1] = time.time()
            if (self.station_status[jetson_id - 1] == 0): # if was marked as disconnected
                # Mark as connected
                self.station_status[jetson_id - 1] = 1
                return True
            return False
            
    def Check(self):
        time.sleep(0.1)
        jetsons_disconnected = []
        
        with ConnectionChecker.LOCK:
            current_time = time.time()
            for jetson_index in range(self.num_of_stations):
                if (self.station_status[jetson_index] == 1): # Station was connected
                    if (current_time - self.last_update_time[jetson_index] > ConnectionChecker.WaitTime):
                        # Station Disconnected
                        self.station_status[jetson_index] = 0 
                        jetsons_disconnected.append(jetson_index + 1)
            return jetsons_disconnected
