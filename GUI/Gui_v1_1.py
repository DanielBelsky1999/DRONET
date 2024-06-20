try:
    from tkinter import * 
    import matplotlib.pyplot as plt
    import matplotlib.table as tbl
    import matplotlib.ticker as ticker
    import matplotlib.animation as animation
    from matplotlib.figure import Figure 
    from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk) 
    import socket
    import threading
    from time import sleep
    import time
    import math
    import pyproj
except ModuleNotFoundError as e:
    import ctypes
    err_str = str(e)
    ctypes.windll.user32.MessageBoxW(None, err_str, u"GUI ERROR", 0)
    exit()
    
    
deg2rad = math.pi/180

class JetsonStation:
    FOV = 90 # deg
    LINE_LENGTH = 4.5
    
    def __init__(self,jetson_id, X,Y, vector_x_y):
        # vector_x_y should look like [x1,y1]
        self.jetson_id = jetson_id
        self.X = X
        self.Y = Y
        norm = (vector_x_y[0]**2 + vector_x_y[1]**2)**0.5
        self.unit_vec = [vector_x_y[0]/norm, vector_x_y[1]/norm]
    
    def Update(self, X,Y,vector_x_y):
        self.X = X
        self.Y = Y
        norm = (vector_x_y[0]**2 + vector_x_y[1]**2)**0.5
        self.unit_vec = [vector_x_y[0]/norm, vector_x_y[1]/norm]
    
    def SetLine2D(self,line_obj):
        v1, v2 = self.GetFOV_vectors()
        line_obj.set_xdata([JetsonStation.LINE_LENGTH*v1[0]+self.X, self.X, JetsonStation.LINE_LENGTH*v2[0]+self.X])
        line_obj.set_ydata([JetsonStation.LINE_LENGTH*v1[1]+self.Y, self.Y, JetsonStation.LINE_LENGTH*v2[1]+self.Y])
        return 
    
    def GetFOV_vectors(self):
        # returns vectors which are the LOS vector rotated by half the FOV angle in both directions
        global deg2rad
        v1_x = math.cos(JetsonStation.FOV*deg2rad/2)*self.unit_vec[0]-math.sin(JetsonStation.FOV*deg2rad/2)*self.unit_vec[1]
        v1_y = math.sin(JetsonStation.FOV*deg2rad/2)*self.unit_vec[0]+math.cos(JetsonStation.FOV*deg2rad/2)*self.unit_vec[1]
        v2_x = math.cos(JetsonStation.FOV*deg2rad/2)*self.unit_vec[0]+math.sin(JetsonStation.FOV*deg2rad/2)*self.unit_vec[1]
        v2_y = -math.sin(JetsonStation.FOV*deg2rad/2)*self.unit_vec[0]+math.cos(JetsonStation.FOV*deg2rad/2)*self.unit_vec[1]
        return [v1_x, v1_y], [v2_x, v2_y]
        

class Window:
    # Static Variables of class "Window" 
    VERSION = "v1.1"
    WINDOW_SIZE = "1000x700"
    WINDOW_NAME = "DRONET GUI " +  VERSION
    UDP_IP = "0.0.0.0"
    UDP_PORT = 20001
    LOCK_MAIN_PLOT = threading.Lock()
    LOCK_BAR_PLOT = threading.Lock()
    LOCK_DOP_PLOT = threading.Lock()
    LOCK_ORIGIN_SET_TEXT = threading.Lock()
    
    SEND_TO_ADDR = ("127.0.0.1",25001)
    
    def __init__(self):
    
        # Initialize common variables
        self.CLOSE_ALL = False
        
        self.XYH_coordinates = [-99,-99,-99]
        self.DOP = [-1 for i in range(50)]
        self.DOP_indx = [i for i in range(50)]
        self.packet_time = -1;
        self.time_since_last_XYZ = [-1 for i in range(50)] # Initial value -1. At any given point will be greater than 1
        self.time_indx = [i for i in range(50)]
        
        # Jetson Stations
        self.Jetsons = []  # objects
        self.Jetson_ids = []
        self.JetsonPlotLines = []
        
        # Status Table cells' colors
        self.cell_colors = [[(0.8274509,0,0,1) for c in range(2)] for r in range(6)]
        
        # Set up SENDING socket
        self.UDPSendingSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        
        # Create Main Gui Window, set title, size, etc.
        self.CreateMainWindow()
        # Create all UI elemets - buttons, plots, tables, etc.
        self.CreateUIElemets()
        
        # Set up RECEIVING Socket
        self.SetUpSocket()
        
        # Make the Socket Listener Independent of main thread
        self.t1 = threading.Thread(target=self.RecvData)
        self.t1.start()
        
        try:
            # Animation of all plots (Happens in parallel! and in a loop! - not blocking)
            animation_main_plot = animation.FuncAnimation(self.fig1, self.animate_main_plot, interval=50, blit=True) 
            animation_bar_plot = animation.FuncAnimation(self.fig1, self.animate_bar_plot, interval=50, blit=True) 
            animation_bar_plot = animation.FuncAnimation(self.fig2, self.animate_DOP_plot, interval=50, blit=True)
            animation_FPS_plot = animation.FuncAnimation(self.fig1, self.animate_FPS_plot, interval=50, blit=True)
            animation_Table = animation.FuncAnimation(self.fig3, self.animate_Table, interval=500, blit=True)
        except:
            print("Animation Exception")
        
        # Send UPDATE_ME message:
        self.SendUPDATE_ME()
        
        # run the gui as an infinite loop of updates
        while (not self.CLOSE_ALL):          
            try:
                # Window mainloop - should always stay in main thread!
                self.CheckOriginSet()
                self.window.update_idletasks()
                self.window.update()
                
            except: # If the window has closed, set a flag so all threads could know this
                self.CLOSE_ALL = True
                print("Window was Closed")
                
            
            
    def SetUpSocket(self):        
        self.communication_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.communication_socket.bind((Window.UDP_IP, Window.UDP_PORT))
        self.communication_socket.setblocking(0)
    
    def RecvData(self):
        try:
            while (not self.CLOSE_ALL):
                try:
                    bytesAddressPair = self.communication_socket.recvfrom(1024)
                    message = bytesAddressPair[0]
                    # address = bytesAddressPair[1]
                    raw_data = message.decode()
                    self.ParseMessage(raw_data)
                except BlockingIOError:
                    pass # if no data in the buffer -> just keep going
        except:
            print("Exception in RecvData(). The function is finished.")
           
    
    def ParseMessage(self, raw_data):
        data_packets = raw_data.split(";");
        
        for data_packet in data_packets[0:-1]:
            packet_fields = data_packet.split(",")
            PACKET_HEADER =  packet_fields[0]
            PACKET_BODY = packet_fields[1:]
            # switch PACKET_HEADER:
            if (PACKET_HEADER == "XYZ"):
                with Window.LOCK_MAIN_PLOT:
                    self.XYH_coordinates[0] = float(PACKET_BODY[0])
                    self.XYH_coordinates[1] = float(PACKET_BODY[1])
                with Window.LOCK_BAR_PLOT:
                    self.XYH_coordinates[2] = float(PACKET_BODY[2])
                self.time_since_last_XYZ = self.time_since_last_XYZ[-49:]
                current_time = self.current_milli_time()
                if (self.packet_time<0):
                    self.time_since_last_XYZ.append(0)
                else:
                    try:
                        self.time_since_last_XYZ.append(1000/(current_time-self.packet_time))
                    except ZeroDivisionError:
                        print(current_time, self.packet_time)
                self.packet_time = current_time
                
            elif (PACKET_HEADER == "DOP"):
                with Window.LOCK_DOP_PLOT:
                    # keep the DOP-array 50 places long and append the new DOP value
                    self.DOP = self.DOP[-49:]
                    self.DOP.append(float(PACKET_BODY[0])) 
            
            elif (PACKET_HEADER == "JETSON_POS"):  
                jetson_id = int(PACKET_BODY[0])
                jetson_x = float(PACKET_BODY[1])
                jetson_y = float(PACKET_BODY[2])
                jetson_v1 = float(PACKET_BODY[3])
                jetson_v2 = float(PACKET_BODY[4])
                
                with Window.LOCK_MAIN_PLOT:
                    if jetson_id not in self.Jetson_ids: # New Jetson
                        self.new_Jetson = JetsonStation(jetson_id, jetson_x, jetson_y, [jetson_v1,jetson_v2])
                        self.new_jetson_line, = self.plot1.plot([-1],[-1],color="#4cbb17") # Create Line
                        self.new_Jetson.SetLine2D(self.new_jetson_line)

                        self.JetsonPlotLines.append(self.new_jetson_line)
                        self.Jetsons.append(self.new_Jetson)
                        self.Jetson_ids.append(jetson_id)
                        # print(str(len(self.JetsonPlotLines)), str(len(self.Jetsons)), str(self.Jetson_ids))
                    else: # Existing Jetson
                        for i,jetson in enumerate(self.Jetsons):
                            if jetson.jetson_id == jetson_id:
                                jetson.Update(jetson_x, jetson_y, [jetson_v1,jetson_v2])
                                jetson.SetLine2D(self.JetsonPlotLines[i])
                                break

            elif (PACKET_HEADER == "ORIGIN_SET"):
                with Window.LOCK_ORIGIN_SET_TEXT:
                    self.origin_set_status_changed = True
                    self.Origin_Set = True
                
            elif (PACKET_HEADER == "CONNECTION_STATUS"):
                jetson_id = int(PACKET_BODY[0])
                jetson_connection_status = PACKET_BODY[1]
                if jetson_connection_status == "CONNECTED":
                    self.cell_colors[jetson_id-1][0] = (0.239215686, 0.7372549019607, 0.25098039215686, 1) # Green color, alpha 1
                elif jetson_connection_status == "DISCONNECTED":
                    self.cell_colors[jetson_id-1][0] = (0.8274509804, 0, 0, 1) # Red color, alpha 1
            
            elif (PACKET_HEADER == "POS_ORI_STATUS"):
                jetson_id = int(PACKET_BODY[0])
                jetson_position_status = PACKET_BODY[1]
                if jetson_position_status == "SET":
                    self.cell_colors[jetson_id-1][1] = (0.239215686, 0.7372549019607, 0.25098039215686, 1) # Green color, alpha 1
                elif jetson_position_status == "NOT_SET":
                    self.cell_colors[jetson_id-1][1] = (0.8274509804, 0, 0, 1) # Red color, alpha 1

            elif (PACKET_HEADER == "RESET"):
                with Window.LOCK_MAIN_PLOT:
                    self.RESET()
            
            else:
                print("Illegal Packet. HEADER: \"{HEADER}\"".format(HEADER=PACKET_HEADER))

    def animate_main_plot(self,t):
        with Window.LOCK_MAIN_PLOT:
            self.XY_POINT.set_xdata([self.XYH_coordinates[0]])
            self.XY_POINT.set_ydata([self.XYH_coordinates[1]])
            # print(len(self.JetsonPlotLines))
            return [l for l in self.JetsonPlotLines]+[self.XY_POINT,]


    def animate_bar_plot(self,t):
        with Window.LOCK_BAR_PLOT:
            self.bar.set_height([self.XYH_coordinates[2]])
        return [self.bar,]
     
    def animate_DOP_plot(self,t):
        with Window.LOCK_DOP_PLOT:
            self.DOP_line.set_xdata([self.DOP_indx])
            self.DOP_line.set_ydata([self.DOP])
        return self.DOP_line,
        
    def animate_FPS_plot(self,t):
        self.FPS_line.set_xdata([self.time_indx])
        self.FPS_line.set_ydata([self.time_since_last_XYZ])
        return self.FPS_line,
    
    def animate_Table(self,t):
        for cell_id,cell in self.TABLETOUPDATE._cells.items():
            if ((-1 not in cell_id) and cell_id not in [(0,0), (0,1)]):
                # print("Cell: " , cell_id)
                cell.set_facecolor(self.cell_colors[cell_id[0]-1][cell_id[1]])
        
        return self.TABLETOUPDATE,

    def CreateMainWindow(self):
        # the main Tkinter window 
        self.window = Tk() 
        
        # setting the title  
        self.window.title(Window.WINDOW_NAME) 
        
        # dimensions of the main window 
        self.window.geometry(Window.WINDOW_SIZE) 
        
        self.window.configure(bg='white')
        self.window.resizable(False, False)
        
    def CreateUIElemets(self):
        self.SetUpPlots()
        self.SetUpStatusBoard()
        self.SetUpEntries()
        self.SetUpButtons()
        
    
    def SetUpEntries(self):
        
        # Set-up entries:
        
        self.station_number = Entry(master=self.window, width=10, bg="#ADD8E6",fg='grey', font=10) 
        self.station_number.pack()
        self.station_number.place(x=550, y=470)     
        
        self.Station_Lat = Entry(master=self.window, width=11, bg="#ADD8E6",fg='grey', font=10)
        self.Station_Lat.pack()
        self.Station_Lat.place(x=695, y=450)
        self.Station_Lon = Entry(master=self.window, width=11, bg="#ADD8E6",fg='grey', font=10) 
        self.Station_Lon.pack()
        self.Station_Lon.place(x=695, y=495)
        self.Station_Alt = Entry(master=self.window, width=11, bg="#ADD8E6",fg='grey', font=10)
        self.Station_Alt.pack()
        self.Station_Alt.place(x=695, y=540)
        
        self.Station_Az = Entry(master=self.window, width=9, bg="#ADD8E6",fg='grey', font=10)
        self.Station_Az.pack()
        self.Station_Az.place(x=850, y=450)
        self.Station_Pitch = Entry(master=self.window, width=9, bg="#ADD8E6",fg='grey', font=10)
        self.Station_Pitch.pack()
        self.Station_Pitch.place(x=850, y=495)
        self.Station_Roll = Entry(master=self.window, width=9, bg="#ADD8E6",fg='grey', font=10)
        self.Station_Roll.pack()
        self.Station_Roll.place(x=850, y=540)
        
        # Azimuth Calculator Entries:
        
        self.long1 = Entry(master=self.window, width=10, bg="#4F8394",fg='white',font=("Calibri 14")) 
        self.long1.pack()
        self.long1.place(x=550, y=625)
        self.lat1 = Entry(master=self.window, width=10, bg="#4F8394",fg='white',font=("Calibri 14")) 
        self.lat1.pack()
        self.lat1.place(x=550, y=655)
        self.long2 = Entry(master=self.window, width=10, bg="#4F8394",fg='white',font=("Calibri 14")) 
        self.long2.pack()
        self.long2.place(x=660, y=625) 
        self.lat2 = Entry(master=self.window, width=10, bg="#4F8394",fg='white',font=("Calibri 14")) 
        self.lat2.pack()
        self.lat2.place(x=660, y=655) 
        
        self.azimuth_entry = Text(master=self.window, bg="white",fg='grey',width=10, height=1, font=("Calibri 14")) 
        self.azimuth_entry.pack()
        self.azimuth_entry.place(x=870, y=625)
        self.distance_entry = Text(master=self.window, bg="white",fg='grey',width=10, height=1, font=("Calibri 14")) 
        self.distance_entry.pack()
        self.distance_entry.place(x=870, y=655)
        
        # 
        self.origin_set_text_entry = Text(master=self.window, width=11, height=1, bd=1, bg="#d30000",fg='white', font=("Calibri 14"),relief=FLAT) 
        self.origin_set_text_entry.pack()
        self.origin_set_text_entry.place(x=748, y=80)
        self.Origin_Set = False
        
        # # 
        # self.system_config_text_entry = Text(master=self.window, width=17, height=1, bd=1, bg="#d30000",fg='white', font=("Calibri 14"),relief=FLAT) 
        # self.system_config_text_entry.pack()
        # self.system_config_text_entry.place(x=780, y=80)
        # self.System_Config = False
        
        self.widget_dictionary = { self.station_number:("Station No.","grey","black"),
                                   self.Station_Lat:("Station_Lat","grey","black"),
                                   self.Station_Lon:("Station_Lon","grey","black"),
                                   self.Station_Alt:("Station_Alt","grey","black"),
                                   self.Station_Az:("Yaw-Left","grey","black"),
                                   self.Station_Pitch:("Pitch-Up","grey","black"),
                                   self.Station_Roll:("Roll-Right","grey","black"),
                                   self.long1:("From long.","white","black"),
                                   self.lat1:("From lat.","white","black"),
                                   self.long2:("To long.","white","black"),
                                   self.lat2:("To lat.","white","black")}
                            
        for widget in self.widget_dictionary:
            widget_touple = self.widget_dictionary[widget]
            widget.insert(0, widget_touple[0]) 
            widget.bind("<FocusIn>", self.handle_focus_in)
            widget.bind("<FocusOut>", self.handle_focus_out)
        
        self.origin_set_text_entry.insert("end-1c", "Origin isn't set")
        self.origin_set_text_entry.config(state=DISABLED)
        self.origin_set_status_changed = False
        # self.system_config_text_entry.insert("end-1c", "System ISN'T Configured")
        # self.system_config_text_entry.config(state=DISABLED)
        # self.system_config_status_changed = False
        self.azimuth_entry.insert("end-1c", "azimuth")
        self.azimuth_entry.config(state=DISABLED)
        self.distance_entry.insert("end-1c", "distance")
        self.distance_entry.config(state=DISABLED)
        
    def handle_focus_in(self,event):
        widget = event.widget
        widget_touple = self.widget_dictionary[widget]
        if widget.get() == widget_touple[0]:
            widget.delete(0, END)
            widget.config(fg=widget_touple[2])

    def handle_focus_out(self,event):
        widget = event.widget
        widget_touple = self.widget_dictionary[widget]
        if widget.get() == "":
            widget.insert(0, widget_touple[0])
            widget.config(fg=widget_touple[1])
        
    def SetUpButtons(self):
        # Set coordinates button:
        self.SET_button = Button(master = self.window,  
                             command = self.SetButtonPress, 
                             height = 2,  
                             width = 10, 
                             text = "SET",
                             bg='#54FA9B') 
        # place the button  
        # in main window 
        self.SET_button.pack()
        self.SET_button.place(x=570, y=510)
        
        # Calculator button: 
        self.Calculate_button = Button(master = self.window,  
                             command = self.CalcButtonPress, 
                             height = 3,  
                             width = 9, 
                             text = "CALCULATE\n→",
                             bg='#54FA9B') 
        # place the button  
        # in main window 
        self.Calculate_button.pack()
        self.Calculate_button.place(x=780, y=625)
        
        
        
    def SetUpStatusBoard(self):
        val1 = ["Connection", "Position"] 
        val2 = ["  {}  ".format(i+1) for i in range(6)] 
        val3 = [["\n\n" for c in range(2)] for r in range(6)] 
        
        self.fig3 = Figure(figsize = (4,4.4), dpi = 100) 
        self.table_axes = self.fig3.add_subplot()
        self.canvas3 = FigureCanvasTkAgg(self.fig3, 
                           master = self.window) 
        self.table_axes.set_axis_off() 
        tbl.Table.AXESPAD = 0.12
        self.StatusBoard = tbl.table( 
            self.table_axes, 
            cellText = val3, 
            rowLabels = val2, 
            colLabels = val1,      
            cellColours = [["#d30000" for c in range(2)] for r in range(6)],
            rowColours = ["palegreen"] * 6, 
            colColours =["palegreen"] * 2, 
            cellLoc ='center',  
            animated = True,
            loc ='upper left') 
        self.StatusBoard.scale(0.8,2.5)
        self.table_axes.set_title("Status Board\n")
        self.TABLETOUPDATE = self.table_axes.add_table(self.StatusBoard)
        
        # print(self.TABLETOUPDATE._cells)
        # for cell_id,cell in self.TABLETOUPDATE._cells.items():
            # if (cell_id = ()):
                # print(cell_id, cell.fill, cell._facecolor)
                # cell.set_facecolor((0.5, 0.5, 0.5, 1))
                # print(cell_id, cell.fill, cell._facecolor)
        

        self.canvas3.get_tk_widget().place(x=600, y=30)
        self.canvas3.draw()
        
    def SetUpPlots(self):
        self.fig1 = Figure(figsize = (5.2,7.5), dpi = 100) 
        self.fig1.subplots_adjust(left=0.2)
        
        # adding the subplot 
        self.plot1 = self.fig1.add_subplot(11,1, (1,7))
        self.plot1.set_xlim(-5, 50)
        self.plot1.set_ylim(-5, 50)
        # self.plot1.yaxis.tick_right()
        self.plot1.set_title("X-Y Position")
        self.plot1.set_ylabel("Y [m]")
        self.plot1.set_xlabel("X [m]")
        self.plot1.grid()
        self.XY_POINT, = self.plot1.plot([-99],[-99], marker="o"); # Fake plot out of bounds just to return a line
        
        self.plot2 = self.fig1.add_subplot(11,1,(9))
        self.plot2.set_ylabel("DOP")
        self.DOP_line, = self.plot2.plot([-1],[-1]); # Fake plot out of bounds just to return a line
        self.plot2.set_ylim(0, 50)
        self.plot2.set_xlim(0, 50)       
        self.plot2.tick_params(axis='x',which='both', bottom=False, top=False, labelbottom=False)

        
        self.plot3 = self.fig1.add_subplot(11,1,(11))
        self.plot3.set_title("Updates Per Second")
        self.FPS_line, = self.plot3.plot([-1],[-1]); # Fake plot out of bounds just to return a line
        self.plot3.set_ylim(0, 30)
        self.plot3.set_xlim(0, 50)  
        self.plot3.set_yticks([i*10 for i in range(4)])       
        self.plot3.tick_params(axis='x',which='both', bottom=False, top=False, labelbottom=False)
        self.plot3.grid()  

        # creating the Tkinter canvas 
        # containing the Matplotlib figure 
        self.canvas = FigureCanvasTkAgg(self.fig1, 
                                   master = self.window) 
                           
        # placing the canvas on the Tkinter window 
        self.canvas.get_tk_widget().pack()
        self.canvas.get_tk_widget().place(x=0, y=-24)
        self.canvas.draw()



        
        self.fig2 = Figure(figsize = (1,4.7), dpi = 100) 
        self.bar_plot = self.fig2.add_subplot(1,3,(1))
        self.canvas2 = FigureCanvasTkAgg(self.fig2, 
                           master = self.window) 
        self.bar_plot.set_title("  Height")
        self.bar_plot.yaxis.tick_right()
        # placing the canvas on the Tkinter window 
        self.canvas2.get_tk_widget().pack()
        self.bar_plot.set_ylim(0, 20)
        self.bar_plot.set_xlim(-1.1,1)
        self.bar, = self.bar_plot.bar([0],[7], width=1.87)
        self.bar_plot.tick_params(axis='x',which='both', bottom=False, top=False, labelbottom=False)
        self.canvas2.get_tk_widget().place(x=500, y=10)
        self.canvas2.draw()

        
    def SetButtonPress(self):
        message_string = "GUI,POS,{jetson_indx},{LAT},{LON},{ALT},{PSI},{THETA},{PHI};".format(
                               jetson_indx=self.station_number.get(),
                               LAT = self.Station_Lat.get(),
                               LON = self.Station_Lon.get(),
                               ALT = self.Station_Alt.get(),
                               PSI = self.Station_Az.get(),
                               THETA = self.Station_Pitch.get(),
                               PHI = self.Station_Roll.get())

        bytesToSend = message_string.encode()
        self.UDPSendingSocket.sendto(bytesToSend, Window.SEND_TO_ADDR)
        print("SET DATA")
        
    def CalcButtonPress(self):
        geodesic = pyproj.Geod(ellps='WGS84')
        try: 
            lon1 = float(self.long1.get())
            lon2 = float(self.long2.get())
            lat1 = float(self.lat1.get())
            lat2 = float(self.lat2.get())

            fwd_azimuth,back_azimuth,distance = geodesic.inv(lon1, lat1, lon2, lat2)
            
            self.azimuth_entry.config(state=NORMAL)
            self.azimuth_entry.config(fg='black')
            self.azimuth_entry.delete("1.0", END)
            self.azimuth_entry.insert("1.0",str(fwd_azimuth))
            self.azimuth_entry.config(state=DISABLED)
            
            self.distance_entry.config(state=NORMAL)
            self.distance_entry.config(fg='black')
            self.distance_entry.delete("1.0",END)
            self.distance_entry.insert("1.0",str(distance))
            self.distance_entry.config(state=DISABLED)
        
        except ValueError:
            self.azimuth_entry.config(state=NORMAL)
            self.azimuth_entry.delete("1.0", END)
            self.azimuth_entry.insert("1.0", "ERROR")
            self.azimuth_entry.config(state=DISABLED)
            
            self.distance_entry.config(state=NORMAL)
            self.distance_entry.delete("1.0",END)
            self.distance_entry.insert("1.0","ERROR")
            self.distance_entry.config(state=DISABLED)
    
    def CheckOriginSet(self):
        if (self.origin_set_status_changed): # only if an update occured:
            with Window.LOCK_ORIGIN_SET_TEXT:
                if (self.Origin_Set):
                    self.origin_set_text_entry.config(state=NORMAL)
                    self.origin_set_text_entry.config(bg='#3DBC40')
                    self.origin_set_text_entry.delete("1.0",END)
                    self.origin_set_text_entry.insert("1.0","    Origin Set")
                    self.origin_set_text_entry.config(state=DISABLED)
                else:
                    self.origin_set_text_entry.config(state=NORMAL)
                    self.origin_set_text_entry.config(bg='#d30000')
                    self.origin_set_text_entry.delete("1.0",END)
                    self.origin_set_text_entry.insert("1.0","Origin isn't set")
                    self.origin_set_text_entry.config(state=DISABLED)
                
                self.origin_set_status_changed = False
                
            
            
    def current_milli_time(self):
        return (time.time() * 1000)
        
    def SendUPDATE_ME(self):
        message_string = "GUI,UPDATE_ME;"
        bytesToSend = message_string.encode()
        self.UDPSendingSocket.sendto(bytesToSend, Window.SEND_TO_ADDR)
        print("UPDATE ME")
        
    def RESET(self):
        self.XYH_coordinates = [-99,-99,-99] # Reset Target point
        # Jetson Stations reset
        self.Jetsons = []
        self.Jetson_ids = []
        self.JetsonPlotLines = []
            
        with Window.LOCK_BAR_PLOT:
            self.DOP = [-1 for i in range(50)]   # Reset DOP 

        # Reset Status Table
        self.cell_colors = [[(0.8274509,0,0,1) for c in range(2)] for r in range(6)]
 
        with Window.LOCK_ORIGIN_SET_TEXT:
            self.origin_set_status_changed = True
            self.Origin_Set = False

        print("RESET")
        
        

def main():
    GUI_Window = Window()


if (__name__ == "__main__"):
    main()