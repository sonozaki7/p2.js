# -*- coding: utf-8 -*-
"""
Created on Sat Feb 15 14:15:00 2020

@author: asadl
"""

# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

"""
This file has two classes ImageViewer and PcapSniffer
"""


def getNum_Pedestrian(P_sub,pts):
    count=0
    for i in range(len(P_sub)):
        cxo=P_sub[i,2]
        cyo=P_sub[i,3]
        mode=P_sub[i,4]
        if(mode==2 and point_inside_polygon(cxo,cyo,list(pts))):
            cout+=1
    return count

    
import multiprocessing
import numpy as np
import dpkt
import struct
import time
#from copy import deepcopy
#from time import gmtime, strftime
import pandas as pd 
import cv2
import sys



''' This block of code is mainly for reading raw packet of data received from a lidar sensor and convert it to arrays of distance measurements or frames, DO NOT edit it '''
#pedestrians
#resolution = int(9*100)
resolution = int(9*100)
radiusToCover = 22.5 #32.5 #3.75 # in meters
distancePixelRatio = resolution / (2 * radiusToCover)

# for vlp-32c
elevations = ((-25, -1, -1.667, -15.639, -11.31, 0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4, -4.667, 1.667, 1, -3.667, -3.333, 3.333, 2.333, -2.667, -3, 7, 4.667, -2.333, -2, 15, 10.333, -1.333))
sinElevations = np.array(np.sin(np.radians([[-25, -1, -1.667, -15.639, -11.31, 0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4, -4.667, 1.667, 1, -3.667, -3.333, 3.333, 2.333, -2.667, -3, 7, 4.667, -2.333, -2, 15, 10.333, -1.333]])))
cosElevations = np.array(np.cos(np.radians([[-25, -1, -1.667, -15.639, -11.31, 0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4, -4.667, 1.667, 1, -3.667, -3.333, 3.333, 2.333, -2.667, -3, 7, 4.667, -2.333, -2, 15, 10.333, -1.333]])))
deltas = np.array(np.radians([[1.4,-4.2,1.4,-1.4,1.4,-1.4,4.2,-1.4,1.4,-4.2,1.4,-1.4,4.2,-1.4,4.2,-1.4,1.4,-4.2,1.4,-4.2,4.2,-1.4,1.4,-1.4,1.4,-1.4,1.4,-4.2,4.2,-1.4,1.4,-1.4]]))
distanceMultiplier = 0.004

class PcapSniffer(multiprocessing.Process):
    def __init__(self,fileName,startTime=0.0,startPacket=0):
        self.fileName = fileName
        self.startTime = startTime
        self.startPacket = startPacket
        self.numberOfPackets = 0
        self.firstPacketTime = 0.0
        self.endPacketTime = 0.0
        self.maxPacketTime = 0.0
        self.hasPassedMaxTime = False
        self.frameOutQueue = multiprocessing.Queue()
        self.controlQueue = multiprocessing.Queue()
        self.measurePcap()
        multiprocessing.Process.__init__(self)
        self.start()

    def run(self):
        # loop to open the pcap after a control input is received
        while(True):
            try:
                self.startTime, self.startPacket = self.controlQueue.get(block=False)
                break
            except:
                pass
            f = open(self.fileName,'rb')
            pcap = dpkt.pcap.Reader(f)

            frameData = []
            lastAzimuth = 0.0
            timeStamp = 0.0

            # loop through packets in pcap
            for _,buf in pcap:
                lastFrameProcessTime = time.time()

                try:
                    self.startTime, self.startPacket = self.controlQueue.get(block=False)
                    if(self.startTime == 0.0 and self.startPacket != 0.0):
                        self.startTime = (timeStamp - (timeStamp %5.0)) + self.startPacket
                    if(self.startTime < timeStamp):
                        break
                except:
                    pass

                if(len(buf) == 1248):
                    data = np.frombuffer(buf, dtype=np.uint8)[42:]
                    if(timeStamp > (struct.unpack_from('I', data[-6:-2])[0] / 1000000)):
                        self.hasPassedMaxTime = True

                    timeStamp = (float(self.hasPassedMaxTime)*self.maxPacketTime) + struct.unpack_from('I', data[-6:-2])[0] / 1000000

                    if(timeStamp > self.startTime - 1.0):
                        # wait for last frame to be taken off the predictionsOutQueue
                        while (not self.frameOutQueue.empty()):
                            time.sleep(0.001)
                            pass

                        azimuth = ((data[3] * 256 + data[2]) * 0.01)

                        frameData.append(data)
                        if (lastAzimuth < 180.0 and azimuth > 180.0):
                            payloadsArray = np.array(frameData)
                            frameData = []
                            payloadsArray = payloadsArray[:, 0:1200]
                            payloadsArray = payloadsArray.reshape((-1, 100))
                            azimuths = np.radians((payloadsArray[:, 3] * 256 + payloadsArray[:, 2]) * 0.01).reshape(
                                (-1, 1)) + 0.0*np.pi

                            payloadsArray = payloadsArray[:, 4:]
                            payloadsArray = payloadsArray.reshape((-1, 3))
                            distances = ((payloadsArray[:, 1] * 256 + payloadsArray[:, 0]) * distanceMultiplier).reshape(
                                (-1, 32))
                            # reflectivities = (payloadsArray[:,2] / 255)

                            deltaAdjustedAzimuths = (
                                        np.repeat(azimuths, distances.shape[1], axis=1) + np.repeat(deltas, distances.shape[0],
                                                                                                    axis=0)).reshape((-1))
                            cosElevationsRepeated = (np.repeat(cosElevations, distances.shape[0], axis=0)).reshape((-1))
                            # sinElevationsRepeated = (np.repeat(sinElevations, distances.shape[0], axis=0)).reshape((-1))

                            distances = distances.reshape((-1))

                            Xs = (((radiusToCover - distances * cosElevationsRepeated * np.sin(
                                deltaAdjustedAzimuths)) - 0.0) * distancePixelRatio).astype(np.uint16).astype(np.int)
                            Ys = ((distances * cosElevationsRepeated * np.cos(
                                deltaAdjustedAzimuths) - 0.0) * distancePixelRatio).astype(np.uint16).astype(np.int) #0.0 was 5.0
                            # Zs = distances * sinElevationsRepeated

                            validXs = np.where(Xs < resolution)
                            Xs = Xs[validXs]
                            Ys = Ys[validXs]
                            validYs = np.where(Ys < resolution)
                            Xs = Xs[validYs].reshape((-1, 1))
                            Ys = Ys[validYs].reshape((-1, 1))

                            frame = np.zeros(shape=(resolution, resolution, 3), dtype=np.float)
                            frame[Ys, Xs] = np.array((1.0,1.0,1.0)) #(0.5, 0.5, 0.5)) #

                            if (timeStamp > self.startTime):
                                # frame = frame * np.array([1.0, 0.5, 1.0])
                                # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                                # frame = cv2.dilate(frame, kernel, iterations=1)

                                self.frameOutQueue.put((frame,timeStamp))
                                # print(f'{(time.time() - lastFrameProcessTime):.7f}')
                                # lastFrameProcessTime = time.time()


                        lastAzimuth = azimuth

    def pixelCoordinatesToMeters(self,coordinates):
        coordinates = np.array(coordinates).astype(np.float)
        coordinates[:,0:2] = coordinates[:,0:2]/distancePixelRatio
        return coordinates

    def meterCoordinatesToPixels(self,coordinates):
        coordinates = np.array(coordinates)
        coordinates[:,0:2] = coordinates[:,0:2]*distancePixelRatio
        return coordinates

    def measurePcap(self):
        f = open(self.fileName, 'rb')
        pcap = dpkt.pcap.Reader(f)



        for _, buf in pcap:
            if (len(buf) == 1248):
                data = np.frombuffer(buf, dtype=np.uint8)
                self.firstPacketTime = struct.unpack_from('I', data[-6:-2])[0] / 1000000
                self.numberOfPackets += 1
                break
        for _, buf in pcap:
            if (len(buf) == 1248):
                data = np.frombuffer(buf, dtype=np.uint8)
                if((struct.unpack_from('I', data[-6:-2])[0] / 1000000) < self.endPacketTime and self.maxPacketTime == 0.0):
                    self.maxPacketTime = self.endPacketTime
                self.endPacketTime = self.maxPacketTime + (struct.unpack_from('I', data[-6:-2])[0] / 1000000)
                self.numberOfPackets += 1



''' This bkock of code visualizes the frame data using OpenCV library '''
class ImageViewer:

    def __init__(self, mouseCallback=None, trackBarCallback=None): #, playVideo=False
        cv2.namedWindow("output")
        # if(playVideo):
        #     cv2.namedWindow("video")
        if (trackBarCallback is None):
            cv2.createTrackbar('packetTrackbar','output',0,1000,self.passFunc2)
        else:
            cv2.createTrackbar('packetTrackbar','output',0,1000,trackBarCallback)
        if(mouseCallback is None):
            mouseCallback = self.passFunc
        self.callback = mouseCallback
        cv2.setMouseCallback("output", self.callback)

    def passFunc2(self,position):
        pass

    def passFunc(self,event, x, y, flags, param):
        pass

    def showImage(self, frame, dimensions=(0.0, 0.0), name="output"):
        if (dimensions == (0.0, 0.0)):
            cv2.imshow(name, frame)
        else:
            if(dimensions[0]/frame.shape[0] > dimensions[1]/frame.shape[1]):
                dimensions = (int(frame.shape[1]*(dimensions[0]/frame.shape[0])),int(dimensions[0]))
            else:
                dimensions = (int(dimensions[1]),int(frame.shape[0]*(dimensions[1]/frame.shape[1])))

            frame = cv2.resize(frame, dimensions)
            cv2.imshow(name, frame)

    def destroyWindows(self):
        cv2.destroyAllWindows()

                
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
 








''' This is where you can play with the data and run your code or even modify it '''
if __name__ == "__main__":
    
    '''Some Flags'''
    plot_object=True
    plot_loops=True
    plot_occupancy=True
    plot_tracking=True
    toggle_mode=['car','ped','bike','all']
    toggle_mode_index=4 # it starts with showing all modes
    fps=1000.0
    
    ''' Exisitng file names '''
    file_name='DATA_20200323_154915'
    file_name='DATA_20200323_160416'
    file_name='DATA_20200323_161917'
    file_name='DATA_20200323_163418'
    file_name='DATA_20200323_164919'
    file_name='DATA_20200323_170420'
    file_name='DATA_20200323_171921'
    file_name='DATA_20200323_173422'
    file_name='DATA_20200323_174923'
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
    
    
    
    ''' Some initialization '''
    object_ID=1
    exit_flag=False
    frame_count=0
    frame_count_total=0
    scale=1 
    font = cv2.FONT_HERSHEY_SIMPLEX
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*"XVID"), fps, (int(900*scale),int(900*scale)), True) # Record output as a video file
    color=[[(255,0,0)],[(0,255,0)],[(0,0,255)]] # Colors assigned for each object based on their mode
    
    imageViewer = ImageViewer()
    
    lastFrameProcessTime = time.time()
    
    P[:,2]=(P[:,2]/0.05) # Converts distance value from Excel to Pixel values. each pixel associated to 5cm in real world.
    P[:,3]=(P[:,3]/0.05) # Converts distance value from Excel to Pixel values. each pixel associated to 5cm in real world.
    P[:,0]=(P[:,0]*10) # time in seconds with flowting numbers (for example 101.1235 seconds) will be converted to 1011.235) based on 10 frame per second sample rate every 100 millisecond we will get a new frame
    P=P.astype(np.int) # Convert everything to integer to make it faster and easier 
    
    while(not exit_flag):
        frame, timeStamp = pcapSniffer.frameOutQueue.get()
        timeStamp=int(timeStamp*10) # The same approach as line code 299
        frame=((frame/frame.max())*255).astype('uint8')
        
        
        frame_count_total=frame_count_total+1
        
        frame_count=frame_count+1
        if(frame_count>=10.0/fps):
            frame_count=0
            
            
            P_sub=P[P[:,0]==timeStamp,:] # Reads the data about the current frame returned from pcapSniffer
            
            if toggle_mode_index!=4:
                P_sub=P_sub[P_sub[:,4]==toggle_mode_index,:]
            
            
            ''' Now let's plot the info about all objects in the frame'''
            
#            unique_obj=np.unique(P_sub[:,1])
            for i in range(len(P_sub)):
                cx=P_sub[i,2]
                cy=P_sub[i,3]
                mode=P_sub[i,4]
                Oid=P_sub[i,1]
                
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
            
            '''Plot Virtual Loops'''                
            if(plot_loops): 
                loops=np.copy(loops_location)
                if toggle_mode_index!=4:
                    loops=loops[loops[:,3]==toggle_mode_index,:]
                    
                for l in np.unique(loops[:,0]):
                    
                    pt=loops[loops[:,0]==l,:]
                    pts = np.array([[pt[0,1],pt[0,2]],[pt[1,1],pt[1,2]],[pt[2,1],pt[2,2]],[pt[3,1],pt[3,2]]], np.int32)
                            
                    cv2.polylines(frame,[pts],True,color[int(pt[0,3])-1][0])                        
                    
                    '''Plot Occupancy''' 
                    if(plot_occupancy):                        
                        for i in range(len(P_sub)):
                            cxo=P_sub[i,2]
                            cyo=P_sub[i,3]
                            mode=P_sub[i,4]
                            cx=(max(pt[:,1])-min(pt[:,1]))/2+min(pt[:,1])
                            cy=(max(pt[:,2])-min(pt[:,2]))/2+min(pt[:,2])
                            
                            if(point_inside_polygon(cxo,cyo,list(pts))):
                                if int(pt[0,3])==mode:
                                    cv2.circle(frame,(int(cx),int(cy)),8,(0,0,255),15)
                    cx=(max(pt[:,1])-min(pt[:,1]))/2+min(pt[:,1])
                    cy=(max(pt[:,2])-min(pt[:,2]))/2+min(pt[:,2])
                    cv2.putText(frame,str(int(l)),(int(cx),int(cy)),font, 0.6, (255,255,255),2)
                    
                    
                
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

    #outside of while loop
    imageViewer.destroyWindows()
    out.release()  
    sys.exit()


