import time
import multiprocessing
import numpy as np
import dpkt
import struct
import pandas as pd 
import cv2
import sys
import os
import math #for sqrt()
from Visualization import PcapSniffer
from Visualization import ImageViewer
from Visualization import resolution


'''
    TrafficManagementSystem
    this class will store the information about the intersection
'''
class TM:
    leftBottom_car_count = None
    rightTop_car_count = None
    leftTop_car_count = None
    rightBottom_car_count = None
    left_ped_count = None
    right_ped_count = None
    top_ped_count = None
    bottom_ped_count = None

    def __init__(self):
        self.leftBottom_car_count = 0
        self.rightTop_car_count = 0
        self.leftTop_car_count = 0
        self.rightBottom_car_count  = 0
        self.left_ped_count = 0
        self.right_ped_count = 0
        self.top_ped_count = 0
        self.bottom_ped_count = 0

    def updateAll(self, LTcar, RTcar, RBcar, LBcar, Lped, Tped, Rped, Bped):
        self.leftBottom_car_count = LBcar
        self.rightTop_car_count = RTcar
        self.leftTop_car_count = LTcar
        self.rightBottom_car_count = RBcar

        self.left_ped_count = Lped
        self.right_ped_count = Rped
        self.top_ped_count = Tped
        self.bottom_ped_count = Bped

    def resetAll(self):
        self.leftBottom_car_count = 0
        self.rightTop_car_count = 0
        self.leftTop_car_count = 0
        self.rightBottom_car_count  = 0
        self.left_ped_count = 0
        self.right_ped_count = 0
        self.top_ped_count = 0
        self.bottom_ped_count = 0

    def updateCarRT(self, count):
        self.rightTop_car_count = count
    
    def updateCarRB(self, count):
        self.rightBottom_car_count = count
    
    def updateCarLT(self, count):
        self.leftTop_car_count = count
    
    def updateCarLB(self, count):
        self.leftBottom_car_count = count
    
    def updatePedT(self,count):
        self.top_ped_count = count
    
    def updatePedB(self,count):
        self.bottom_ped_count = count
    
    def updatePedR(self,count):
        self.right_ped_count = count
    
    def updatePedL(self,count):
        self.left_ped_count = count




    def incCarRT(self):
        self.rightTop_car_count += 1
    
    def incCarRB(self):
        self.rightBottom_car_count += 1
    
    def incCarLT(self):
        self.leftTop_car_count += 1
    
    def incCarLB(self):
        self.leftBottom_car_count += 1
    
    def incPedT(self):
        self.top_ped_count += 1
    
    def incPedB(self):
        self.bottom_ped_count += 1
    
    def incPedR(self):
        self.right_ped_count += 1
    
    def incPedL(self):
        self.left_ped_count += 1




    def resetCarRT(self):
        self.rightTop_car_count = 0
    
    def resetCarRB(self):
        self.rightBottom_car_count = 0
    
    def resetCarLT(self):
        self.leftTop_car_count = 0
    
    def resetCarLB(self):
        self.leftBottom_car_count = 0
    
    def resetPedT(self):
        self.top_ped_count = 0
    
    def resetPedB(self):
        self.bottom_ped_count = 0
    
    def resetPedR(self):
        self.right_ped_count = 0
    
    def resetPedL(self):
        self.left_ped_count = 0


    def getPedT(self):
        return self.top_ped_count
    
    def getPedB(self):
        return self.bottom_ped_count
    
    def getPedR(self):
        return self.right_ped_count
    
    def getPedL(self):
        return self.left_ped_count

             

'''
class for a traffic light.

since we are only dealing with intersections, pedestrian lights are coupled
with the traffic light
'''
class TrafficLight:
    currLight = None
    prevLight = None

    '''
    Creates TrafficLight object.
    @param initLight: initial light signal 
        - 'R' for red, 'G' for green
        - cannot initialize as yellow ('Y')
    '''
    def __init__(self, initLight):
        self.currLight = initLight
        
        if initLight == 'R':
            self.prevLight = 'G'
        else:
            self.prevLight = 'R'

    '''
    Changes light.
    @param newLight: new light signal
    '''
    def changeLight(self, newLight):
        if newLight != self.currLight:
            self.currLight = self.prevLight
            self.currLight = newLight
    

    '''
    Changes to opposite signal
    '''
    def oppositeLight(self):
        if self.currLight != 'Y':
            self.prevLight = self.currLight
            if self.currLight == 'R':
                self.currLight = 'G'
            else:
                self.currLight = 'R'

    '''
    Returns current traffic light signal
    '''    
    def getCurrTrafficLight(self):
        return self.currLight

    '''
    Returns current pedestrian light signal
    '''
    def getCurrPedLight(self):
        if self.currLight == 'G' or self.currLight == 'Y':
            return 'R'
        else:
            return 'G'
    
    '''
    Returns BGR color for current light signal
    '''
    def getLightColor(self):
        if self.currLight == 'G':
            return (0, 255, 0)
        elif self.currLight == 'R':
            return (0, 0, 255)
        else:
            return (0, 255, 255)

    '''
    Returns full color name for current light signal
    '''
    def getFullColorName(self):
        if self.currLight == 'G':
            return 'GREEN'
        elif self.currLight == 'R':
            return 'RED'
        else:
            return 'YELLOW'

import json
import socket


'''
Reads GPS data from local network
Uses port 8080
'''
class GPS_Reader:
    HOST = None
    PORT = None
    jsonStringSize = None
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    num_received_data = 0
    num_used_data = 0
    jsonReceive = []

    '''
    (agreed jsonStringSize: 40)
    localIP: local ip of machine hosting the server. Enter as string.
    get localIP by:
        - windows: ipconfig /# (value of IPv4 address)
        - linux: ifconfig
    '''
    def __init__(self, localIP):
    #def __init__(self, localIP, jsonStringSize):
        self.HOST = localIP
        self.PORT = 8080
        #self.jsonStringSize = jsonStringSize
        self.jsonStringSize = 40

        self.serverSocket.bind((self.HOST, self.PORT))
        self.serverSocket.listen(10)

    def updateData(self):
        clientSocket, _ = self.serverSocket.accept()
        data = clientSocket.recv(self.jsonStringSize)
        self.jsonReceive[self.num_received_data] = json.loads(data.decode('utf-8'))
        self.num_received_data += 1

    '''
    returns (<longitude>, <latitude>)
    '''
    def getLonglat(self):
        retTuple = (self.jsonReceive[self.num_used_data]["longitude"], self.jsonReceive[self.num_used_data]["latitude"])
        self.num_used_data += 1
        return retTuple

class GPS:
    FrameEdgeGPS=None
    resolution=None
    longitude = 0
    latitude = 0
    point_1_x = 0
    point_1_y = 0
    point_2_x = 0
    point_2_y = 0
    point_3_x = 0
    point_3_y = 0
    d_x = 0
    d_x_total = 0
    d_y = 0
    d_y_total = 0
    initTime = 0
    updateTime = 0
    prevX = 0
    prevY = 0
    curX = 0
    curY = 0
    tempX = 0
    tempY = 0
    timeDif = 0

    def __init__(self,FrameEdgeGPS,resolution):
        self.FrameEdgeGPS = FrameEdgeGPS
        self.resolution = resolution
        # center of intersection = 49.234791, -123.185322
        self.longitude = 49.234791
        self.latitude = -123.185322
        self.point_1_x = self.FrameEdgeGPS[0][0]
        self.point_1_y = self.FrameEdgeGPS[0][1]
        self.point_2_x = self.FrameEdgeGPS[1][0]
        self.point_2_y = self.FrameEdgeGPS[1][1] 
        self.point_3_x = self.FrameEdgeGPS[3][0]
        self.point_3_y = self.FrameEdgeGPS[3][1]
        self.d_x = abs((self.point_3_y-self.point_1_y)*self.longitude + (self.point_3_x-self.point_1_x)*self.latitude + self.point_3_x*self.point_1_y - self.point_1_x*self.point_3_y) / math.sqrt((self.point_3_y-self.point_1_y)*(self.point_3_y-self.point_1_y)+(self.point_3_x-self.point_1_x)*(self.point_3_x-self.point_1_x))
        self.d_x_total = math.sqrt((self.point_1_x-self.point_3_x)*(self.point_1_x-self.point_3_x) + (self.point_1_y-self.point_3_y)*(self.point_1_y-self.point_3_y))
        self.d_y = abs((self.point_2_y-self.point_1_y)*self.longitude + (self.point_2_x-self.point_1_x)*self.latitude + self.point_2_x*self.point_1_y - self.point_1_x*self.point_2_y) / math.sqrt((self.point_2_y-self.point_1_y)*(self.point_2_y-self.point_1_y)+(self.point_2_x-self.point_1_x)*(self.point_2_x-self.point_1_x))
        self.d_y_total = math.sqrt((self.point_1_x-self.point_2_x)*(self.point_1_x-self.point_2_x) + (self.point_1_y-self.point_2_y)*(self.point_1_y-self.point_2_y))
        self.updateTime = time.time()
        self.initTime = time.time()
        self.curX = 0
        self.curY = 0

    def getLocation(self):
        x = (int)(((self.d_x)*self.resolution)/self.d_x_total)
        y = (int)((self.d_y*self.resolution)/self.d_y_total)
        ret = np.array([x,y])
        #print(ret)
        return ret

    def getLocationWithinFrame(self):
        """
        self.curX = 162
        self.curY = 243

        if (time.time() - self.initTime > 2):
            if (time.time() - self.updateTime > 0.5):
                self.updateTime = time.time()
                if (self.prevX > 400 or self.prevY > 700):
                    pass
                else:
                    self.curX = self.prevX + 2
                    self.curY = self.prevY + 3

        self.prevX = self.curX
        self.prevY = self.curY
        """
        self.timeDif = time.time() - self.initTime
        print("timeDif in getLocationWithinFrame is " + str(timeDif))
        
        if (not ((self.curX >= 160 and self.curX <= 165) and (self.curY >= 241 and self.curY <= 2245))):
            if (time.time() - self.initTime > 20 or (time.time() > (self.initTime+3))):
                self.curX = self.prevX + 2
                self.curY = self.prevY + 3

                self.prevX = self.curX
                self.prevY = self.curY
        else:
            pass

        ret = np.array([self.curX,self.curY])
        return ret

    def getLocationWithOccupancy(self, oc):
        """
        self.curX = 162
        self.curY = 243

        if (time.time() - self.initTime > 2):
            if (time.time() - self.updateTime > 0.5):
                self.updateTime = time.time()
                if (self.prevX > 400 or self.prevY > 700):
                    pass
                else:
                    self.curX = self.prevX + 2
                    self.curY = self.prevY + 3

        self.prevX = self.curX
        self.prevY = self.curY
        """
        self.timeDif = time.time() - self.initTime
        print("timeDif in getLocationWithinFrame is " + str(timeDif))
        
        if ((not ((self.curX >= 160 and self.curX <= 165) and (self.curY >= 241 and self.curY <= 2245))) or oc == True):
            if (time.time() - self.initTime > 20 or (time.time() > (self.initTime+3))):
                self.curX = self.prevX + 2
                self.curY = self.prevY + 3

                self.prevX = self.curX
                self.prevY = self.curY
        else:
            pass

        ret = np.array([self.curX,self.curY])
        return ret

    def printField(self):
        print("getField called")

    def updateGPS(self):
        pass

            
class waitTime:
    loopID = None
    isStart = None
    waitTime = None
    initTime = None

    def __init__(self, loopID):
        self.loopID = loopID
        self.isStart = True

    def start(self):
        if (self.isStart):
            self.initTime = time.time()
            self.isStart = False
        
    def getWaitTime(self):
        if (self.isStart == False):
            self.waitTime = (time.time() - self.initTime)
        else:
            self.waitTime = 0
        
        return self.waitTime

    def reset(self):
        self.isStart = True
        self.waitTime = 0
        self.initTime = None

    def is_Start(self):
        return self.isStart


''' This is where you can play with the data and run your code or even modify it '''
if __name__ == "__main__":
    isRepeat = True
    while(isRepeat):
        try:
            
            '''Some Flags'''
            plot_object=True
            plot_loops=True
            plot_occupancy=True
            plot_tracking=True
            traffic=True
            toggle_mode=['car','ped','bike','all']
            toggle_mode_index=4 # it starts with showing all modes
            fps=1500.0


            ''' For Traffic, GPS '''
            sw = TrafficLight('G')
            ne = TrafficLight('G')
            se = TrafficLight('R')
            nw = TrafficLight('R')
            interval = 30
            shortenCoeff = 0.7
            maxWaitTime = 20
            timeDif = 0
            isGPSenabled = True
            isGPSInLoop20 = False
            isGPSInLoop21 = True
            isGPSInLoop22 = True
            isGPSInLoop23 = True
            lastUpdateTime = time.time()
            location = None
            cond1 = False
            cond2 = False
            #Actual coordinate from Dunbar W 41st Ave
            # FrameEdgeGPS = np.array([[49.234889, -123.185473],[49.234893, -123.185087],[49.234644, -123.185100],[49.234640, -123.185489]])
            FrameEdgeGPS = np.array([[49.234644, -123.185100],[49.234893, -123.185087],[49.234889, -123.185473],[49.234640, -123.185489]])
            print("     Printing FrameEdgeGPS")
            print(FrameEdgeGPS)
            gps = GPS(FrameEdgeGPS, resolution)
            timer20 = waitTime(20)
            timer21 = waitTime(21)
            timer22 = waitTime(22)
            timer23 = waitTime(23)
            tm = TM()

            # GPS We need to implement a function that turns GPS into a location within LiDAR map
            """ pseudo code
            gps = GPS(longtitude, latitude)

            #GPS location of the 4 edge nodes of the visualizer
            #                           NW              NE          SE          SW
            FrameEdgeGPS = np.array([[lon1, lat1],[lon2, lat2],[lon3, lat3],[lon4, lat4]])
            #calculate the pixel value of the location

            """
            

            """ we wont use them
            file_name='DATA_20200323_154915'
            file_name='DATA_20200323_160416'
            file_name='DATA_20200323_161917'
            file_name='DATA_20200323_163418'
            file_name='DATA_20200323_164919'
            file_name='DATA_20200323_170420'
            file_name='DATA_20200323_171921'
            file_name='DATA_20200323_173422'
            file_name='DATA_20200323_174923'
            """
            file_name='DATA_20200323_180424'
            

            #use your absolute path
            pcapSniffer = PcapSniffer(file_name+'.pcap') # read Pcap file NOTE: Modify path based on your computer 
            
            P_car = pd.read_csv(file_name+'_frozenNotTensorRtBigGood.csv',skiprows=1) # Read CSV file with timestamp, Object ID, x and y cordination and mode, 1 for pedestrian, 2 for cars and 3 for cyclists
            P_ped = pd.read_csv(file_name+'_bigPed_pedestrians.csv') # Read CSV file with timestamp, Object ID, x and y cordination and mode, 1 for pedestrian, 2 for cars and 3 for cyclists
            loops = pd.read_csv("loops.csv") # Read the Virtual Loop locations
            
            loops_location=np.array(loops) # Convert Dataframe to numpy arrays
            P_car=np.array(P_car) # Convert Dataframe to numpy arrays
            P_ped=np.array(P_ped) # Convert Dataframe to numpy arrays
            P=np.concatenate((P_car, P_ped), axis=0) # Add data from ped and car in one array
            
            """
            try:
                if (loops_location[50][3] == 4 and loops_location[50][0] == 20):
                    sys.stdout.write("True\n")
                else:
                    sys.stdout.write("Loops dont exist\n")
            except:
                sys.stdout.write("Loops_Location Error !\n")
            """            
            #print(loops_location)

            ''' Some initialization '''
            oc = False
            object_ID=1
            exit_flag=False
            frame_count=0
            frame_count_total=0
            scale=1 
            font = cv2.FONT_HERSHEY_SIMPLEX
            out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*"XVID"), fps, (int(900*scale),int(900*scale)), True) # Record output as a video file
            color=[[(255,0,0)],[(0,255,0)],[(0,0,255)],[(255,255,0)]] # Colors assigned for each object based on their mode
            #color = R, G, B
            imageViewer = ImageViewer()
            
            lastFrameProcessTime = time.time()
            
            P[:,2]=(P[:,2]/0.05) # Converts distance value from Excel to Pixel values. each pixel associated to 5cm in real world.
            P[:,3]=(P[:,3]/0.05) # Converts distance value from Excel to Pixel values. each pixel associated to 5cm in real world.
            P[:,0]=(P[:,0]*10) # time in seconds with flowting numbers (for example 101.1235 seconds) will be converted to 1011.235) based on 10 frame per second sample rate every 100 millisecond we will get a new frame
            P=P.astype(np.int) # Convert everything to integer to make it faster and easier 
            
            def point_inside_polygon(x,y,poly):
                n = len(poly)
                inside =False
                p1x,p1y = poly[0]
                for i in range(n+1):
                    p2x,p2y = poly[i % n]
                    if y > min(p1y,p2y):
                        if y <= max(p1y,p2y):
                            if x <= max(p1x,p2x):
                                if p1y != p2y:
                                    xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                                if p1x == p2x or x <= xinters:
                                    inside = not inside
                    p1x,p1y = p2x,p2y
                return inside

            def getNum_Pedestrian(P_sub,pts):
                count=0
                for i in range(len(P_sub)):
                    cxo=P_sub[i,2]
                    cyo=P_sub[i,3]
                    mode=P_sub[i,4]
                    if(mode==2 and point_inside_polygon(cxo,cyo,list(pts))):
                        cout+=1
                return count

            while(not exit_flag):
                frame, timeStamp = pcapSniffer.frameOutQueue.get()
                timeStamp=int(timeStamp*10) # The same approach as line code 299
                frame=((frame/frame.max())*255).astype('uint8')
                
                
                frame_count_total=frame_count_total+1
                
                frame_count=frame_count+1
                if(frame_count>=10.0/fps):
                    frame_count=0

                    P_sub=P[P[:,0]==timeStamp,:] # Reads the data about the current frame returned from pcapSniffer
                    # if time in array matches time stamp, P_sub = P[1,:] (= araray of object IDs)
                    # or else it is array of time

                    if toggle_mode_index!=4:
                        P_sub=P_sub[P_sub[:,4]==toggle_mode_index,:]
                        #this is selecting the colum that correspond to specific
                    
                    
                    ''' Now let's plot the info about all objects in the frame'''
                    #unique_obj=np.unique(P_sub[:,1])
                    for i in range(len(P_sub)):
                        cx=P_sub[i,2]
                        cy=P_sub[i,3]
                        mode=P_sub[i,4]
                        Oid=P_sub[i,1] #Object ID
                        
                        if(plot_object):
                            cv2.circle(frame,(cx,cy),3,color[mode-1][0],2)
                            cv2.putText(frame,'('+str(cx)+','+str(cy)+')',(cx+10,cy+10),font, 0.4, color[mode-1][0],1)
                            cv2.putText(frame,str(Oid),(cx+30,cy+30),font, 0.4, color[mode-1][0],1)
                    


                        '''Plot Tracking'''
                        if(plot_tracking): 
                            P_sub_obj=P[P[:,1]==Oid,:]
                            P_sub_obj=P_sub_obj[P_sub_obj[:,4]==mode,:]
                            P_sub_obj=P_sub_obj[P_sub_obj[:,0]<=timeStamp,:]
                            P_sub_obj_sorted = (np.array(sorted(P_sub_obj, key=lambda x : x[0], reverse=True))).astype(np.int)
                            if(len(P_sub_obj_sorted)>1):
                                for j in range(1,min(30,len(P_sub_obj_sorted))):
                                    cv2.line(frame, (P_sub_obj_sorted[j-1,2],P_sub_obj_sorted[j-1,3]), (P_sub_obj_sorted[j,2],P_sub_obj_sorted[j,3]), color[mode-1][0], 2) 
                    
                    print("before Priority_pedestrian")

                    if (isGPSenabled):
                        '''Plot GPS Pedestrian''' 
                        # assume xgps is xaxis, ygps is yaxis
                        '''
                        ret = np.array(gps.getLocationWithinFrame)
                        xgps = ret[0]
                        ygps = ret[1]
                        '''
                        location = gps.getLocationWithOccupancy(oc)
                        cv2.circle(frame, (location[0], location[1]), 4, (0, 255, 255), 2)
                        cv2.putText(frame, '('+str(location[0])+','+str(location[1])+')', (location[0]+10,location[1]+10), font, 0.4, (0, 255, 255), 1)
                        cv2.putText(frame, 'GPS PEDESTRIAN', (location[0] + 30, location[1] + 30), font, 0.4, (0, 255, 255), 1)

                    print("before plot_loops")

                    '''Plot Virtual Loops'''                
                    if(plot_loops): 
                        #sys.stdout.write("you are now in plot_loops\n")
                        loops=np.copy(loops_location)
                        if toggle_mode_index!=4:
                            loops=loops[loops[:,3]==toggle_mode_index,:]
                        
                        for l in np.unique(loops[:,0]):
                            pt=loops[loops[:,0]==l,:]
                            pts = np.array([[pt[0,1],pt[0,2]],[pt[1,1],pt[1,2]],[pt[2,1],pt[2,2]],[pt[3,1],pt[3,2]]], np.int32)
                                
                            cv2.polylines(frame,[pts],True,color[int(pt[0,3])-1][0])                        
                            
                            '''Plot Occupancy''' 
                            if(plot_occupancy):
                                tm.resetAll()               
                                for i in range(len(P_sub)):
                                    cxo=P_sub[i,2]
                                    cyo=P_sub[i,3]
                                    mode=P_sub[i,4]
                                    cx=(max(pt[:,1])-min(pt[:,1]))/2+min(pt[:,1])
                                    cy=(max(pt[:,2])-min(pt[:,2]))/2+min(pt[:,2])
                                    
                                    if(point_inside_polygon(cxo,cyo,list(pts))):
                                        if int(pt[0,3])==mode:
                                            cv2.circle(frame,(int(cx),int(cy)),8,(0,0,255),15)
                                        if (int(pt[0,3])==mode and pt[0,0] == 4):
                                            oc = True
                                        else:
                                            oc = False

                                        if (int(pt[0,3])==mode) and (pt[0,0] == 20):
                                            tm.incPedL()
                                        if (int(pt[0,3])==mode) and (pt[0,0] == 21):
                                            tm.incPedT()
                                        if (int(pt[0,3])==mode) and (pt[0,0] == 22):
                                            tm.incPedR()
                                        if (int(pt[0,3])==mode) and (pt[0,0] == 23):
                                            tm.incPedB()

                        

                            #print("after occupancy")

                            cx=(max(pt[:,1])-min(pt[:,1]))/2+min(pt[:,1])
                            cy=(max(pt[:,2])-min(pt[:,2]))/2+min(pt[:,2])
                            cv2.putText(frame,str(int(l)),(int(cx),int(cy)),font, 0.6, (255,255,255),2)

                            if (pt[0,0] >= 20):
                                centerX=(max(pt[:,1])-min(pt[:,1]))/2+min(pt[:,1])
                                centerY=(max(pt[:,2])-min(pt[:,2]))/2+min(pt[:,2])

                                print("before count pedestrian")
                                '''Count Pedestrian''' 
                                #cv2.circle(frame, (int(centerX),int(centerY)),8,(127,127,127),15)
                                cv2.putText(frame, 'Pedestian Loop ' + str(pt[0,0]), (int(centerX)+10,int(centerY)+10), font, 0.4, (0, 255, 255), 1)
                                value = 0
                                try:
                                    if (pt[0,0] == 20):
                                        value = tm.getPedL()
                                    elif (pt[0,0] == 21):
                                        value = tm.getPedT()
                                    elif (pt[0,0] == 22):
                                        value = tm.getPedR()
                                    elif (pt[0,0] == 23):
                                        value = tm.getPedB()
                                except:
                                    print("if loop in count pedestrian failed")

                                #print("this!?")
                                cv2.putText(frame, 'Count = ' + str(value), (int(cx)+30,int(cy)+30), font, 0.4, (0, 255, 255), 1)
                                


                    print("before traffic")
                    '''
                        Code that will count the number of cars and pedestrians and pass the traffic light
                        modification to TrafficLight
                    '''
                    if(traffic):
                        location = gps.getLocationWithOccupancy(oc)
                        #print("     getLocation() successful")
                        if (isGPSenabled):
                            loops_traffic=np.copy(loops_location)
                            loops_traffic=loops_traffic[loops_traffic[:,3]==4,:]
                                
                            for l in np.unique(loops_traffic[:,0]):
                                
                                pt=loops_traffic[loops_traffic[:,0]==l,:]
                                pts = np.array([[pt[0,1],pt[0,2]],[pt[1,1],pt[1,2]],[pt[2,1],pt[2,2]],[pt[3,1],pt[3,2]]], np.int32)
                                
                                """
                                print("         Printing pt")
                                print(pt)
                                print("         Printing pts")
                                print(pts)
                                print("         Printing location")
                                print(location)
                                """

                                if(point_inside_polygon(location[0],location[1],list(pts))):
                                    if int(pt[0,3])==4:
                                        if (pt[0][0] == 20):
                                            isGPSInLoop20 = True
                                        else:
                                            isGPSInLoop20 = False
                                        if (pt[0][0] == 21):
                                            isGPSInLoop21 = True
                                        else:
                                            isGPSInLoop21 = False
                                        if (pt[0][0] == 22):
                                            isGPSInLoop22 = True
                                        else:
                                            isGPSInLoop22 = False
                                        if (pt[0][0] == 23):
                                            isGPSInLoop23 = True
                                        else:
                                            isGPSInLoop23 = False
                                else:
                                    isGPSInLoop20 = False
                                    isGPSInLoop21 = False
                                    isGPSInLoop22 = False
                                    isGPSInLoop23 = False
                        
                        #print("after plot_loops @ GPSenabled Traffic Change")



                        timeDif = (time.time() - lastUpdateTime)
                        cond1 = (timeDif > interval and not isGPSenabled)
                        cond2 = (timeDif > (interval*shortenCoeff))

                        wt = np.array([timer20.getWaitTime(),timer21.getWaitTime(),timer22.getWaitTime(),timer23.getWaitTime(),])

                        # if maxWaitTime < np.amax(wt) and isGPSenabled:
                        #     lastUpdateTime = time.time()
                        #     sw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     nw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')
                        # elif cond1:
                        #     lastUpdateTime = time.time()
                        #     sw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     nw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')
                        # elif cond2:
                        #     #20 = LEFT, 21 = TOP, 22 = RIGHT, 23 = BOTTOM Pedestrian Loop
                        #     lastUpdateTime = time.time()
                        #     sw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     nw.changeLight('G' if sw.prevLight == 'R' else 'R')
                        #     se.changeLight('G' if sw.prevLight == 'R' else 'R')

                        if maxWaitTime < np.amax(wt) and isGPSenabled:
                            lastUpdateTime = time.time()
                            sw.oppositeLight()
                            se.oppositeLight()
                            nw.oppositeLight()
                            ne.oppositeLight()
                        elif cond1:
                            lastUpdateTime = time.time()
                            sw.oppositeLight()
                            se.oppositeLight()
                            nw.oppositeLight()
                            ne.oppositeLight()
                        elif cond2:
                            #20 = LEFT, 21 = TOP, 22 = RIGHT, 23 = BOTTOM Pedestrian Loop
                            lastUpdateTime = time.time()
                            sw.oppositeLight()
                            se.oppositeLight()
                            nw.oppositeLight()
                            ne.oppositeLight()

                    '''Plot Traffic Light Signals'''
                    # assume we have array of lights
                    cv2.putText(frame, 'Light Signal: ' + ne.getFullColorName(), (650,130), font, 0.6, ne.getLightColor(), 2)
                    cv2.putText(frame, 'Light Signal: ' + se.getFullColorName(), (500,570), font, 0.6, se.getLightColor(), 2)
                    cv2.putText(frame, 'Light Signal: ' + sw.getFullColorName(), (200,500), font, 0.6, sw.getLightColor(), 2)
                    cv2.putText(frame, 'Light Signal: ' + nw.getFullColorName(), (200,150), font, 0.6, nw.getLightColor(), 2)

                    #print("after plot_loops")


                    cv2.putText(frame,'Mode : '+toggle_mode[toggle_mode_index-1],(750,50),font, 0.6, (0,0,255),2)
                    cv2.putText(frame,str(frame_count_total),(750,20),font, 0.6, (0,0,255),2)
                    frame = cv2.resize(frame,None,fx=scale,fy=scale)
                    imageViewer.showImage(frame)
                    k = cv2.waitKey(100) & 0xFF
                    if k == ord('m'): # Change the mode: car, ped,bike,all
                        toggle_mode_index=toggle_mode_index+1
                        if toggle_mode_index>4:
                            toggle_mode_index=1
                    if k == 32: # Pause Playing with space key
                        while(True):
                            k1 = cv2.waitKey(10)
                            if k1 == 32:
                                break
                    if k == 27: # Exit with ESC key
                            exit_flag=True
                            break
                        
                    out.write(frame.astype('uint8'))
                    
                    lastFrameProcessTime = time.time()
            """
            #outside of while loop
            imageViewer.destroyWindows()
            out.release()
            sys.stdout.write("execute sys.exit()\n")
            sys.exit()
            """
        except:
            pass

        # main program
        while True:
            print("### INSTRUCTIONS on how to restart in a single console ######")
            print("1: while the window is crashed, answer y to the following")
            print("2: immediately check close on the window, and choose \"close the window\"")
            print("3: and choose cancel to close the window")
            answer = input('Run again? (y/n): ')
            if answer in ('y', 'n'):
                break
            sys.stdout.write("Invalid input.\n")
        if answer == 'y':
            isRepeat = True
            continue
        else:
            sys.stdout.write("EXIT\n")
            isRepeat = False
            break
        
        """
        try:
            sys.stdout.write("Press CTRL C\n")
            while(True):
                pass
        except:
            sys.stdout.write("EXIT\n")
        """

