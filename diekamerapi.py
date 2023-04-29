import cv2 as cv
import numpy as np
#from cv2.cv2 import COLOR_BGself.radius_arm2LUV
import pandas as pd
import time
from random import randint
import serial
from threading import Thread
############################################################

#Dieser Code ist darauf ausgelegt, in einer Shell importiert zu werden, damit die Funktionen einzeln aufgerufen werden können

############################################################


#kamerahöhe:32cm
#weite ursprung:-2,5cm
#nullwinkel:-4.47°
#Radius arm 1:11cm
#Radius arm 2:9,6cm
#Abstand von nullwinkel weit:22,8cm,10 Pixel
#Abstand von Nullwinkel nah: 3,0cm  270 Pixel
#Winkel nah:0.9°
#Winkel fern:32.4°[se
#Umwandlung y: 0.076°/Pixel
#Umwandlung x:
#Spannweite X:+-19.178°

#Umwandlung x: 0.074°/pixel

#Abstand zur Mittellinie[se Rechts (links von mir aus): 12,5cm 305 pixel
#das gleiche für 13.5

# def shuffle_dict(df):
#     vergeben=[]
#     ef=pd.DataFrame(columns=["x1","y1","x2","y2","x3","y3","x4","y4","bias"],index=None)
#     for i in range(1000):
#         z=randint(0,1200)
#         while z in vergeben:
#             z=randint(0,1200)
#         vergeben.append(z)
#         ef.iloc[i]=df.iloc[z]
bias=0
#zu tun:_bezeichnung arme
#koordinatensystem, mittellinie
#erklärung kreise
class context():
    #Handler von der Computerseite für den Pico
    def __init__(self,connection=True):
        if connection:
            self.ser=serial.Serial("/dev/ttyACM0",baudrate=115200)
        self.t1_prev=0 #Speichert Armwinkel des Gelenks am Fundament
        self.Winkel_y_unten=1#Minimaler vertikaler Kamerawinkel
        self.Pixel_Winkel_y=0.125 #Umwandlung von Bildpixel in vertikalen Winkel
        self.Pixel_Winkel_x=0.117#Umwandlung von Bildpixel in horizontalen Winkel
        self.hoehe=320#Höhe der Kamera
        self.weite=-25#Position des Armmittelpunlkts  gegenüber der kamera
        self.radius_arm1=110
        self.radius_arm2=120
        self.haelfte_x=175#Pixelort der Mittellinie
        self.MP1=np.array([0,self.weite])#Mittelpunkt des ersten Gelenks
    def pos_in_Winkel(self,pt):
        #Umwandlung Pixelpunkt in Kamerawinkel
        if np.all(pt==np.array([0,0])):
            return np.array([0,0])
        return np.array([self.Pixel_Winkel_x*(pt[0]-self.haelfte_x),self.Winkel_y_unten+(self.Pixel_Winkel_y*(270-pt[1]))])
    def Winkel_zu_posxy(self,Wink):
        #Umwandlung Kamerawinkel in position
        Wink=Wink*np.pi/180
        y=self.hoehe*np.tan(Wink[1])
        x=np.sqrt(y**2+self.hoehe**2)*np.tan(Wink[0])
        pos=np.array([x,y])
        return pos
    def waehle_t1(self,t_1):
        #Wähle aus 2 Möglichen Winkeln den nächsten zur vorherigen Wahl, siehe berechne_arm1_Koor
        print("t1:",t_1)
        if 1 or self.t1_prev==0:
            return max(t_1)
        return min(t_1,key=lambda A:abs(A-self.t1_prev))
    def berechne_arm1_Koor(self,MP2):
        """Diese Methode bereichnet, wohin das Erste Gelenk verfahren muss, um den Greifarm einen bestimmten Punkt ansteuern zu lassen

        Args:
            MP2 (array(float,2)): angepeilter punkt, zweites gelenk soll mit dem ende sich an der Stelle befinden

        Returns:
            (array(float,2)): Lage des Kamerawinkels
        """
        A=(MP2[0]-self.MP1[0])**2/self.radius_arm1**2+(MP2[1]-self.MP1[1])**2/self.radius_arm1**2-1+(self.radius_arm2/self.radius_arm1)**2
        B=2*(self.MP1[0]-MP2[0])*self.radius_arm2/self.radius_arm1**2
        C=2*(self.MP1[1]-MP2[1])*self.radius_arm2/self.radius_arm1**2
        p=-2*A*B/(B**2+C**2)#Berechnet ein Gelichungssystem mit Parametrischen Kreisgleichungen, um den Armwinkel zu ermitteln
        q=(A**2-C**2)/(B**2+C**2)
        X=np.array([-p/2+np.sqrt((p/2)**2-q),-p/2-np.sqrt((p/2)**2-q)])
        t_1=np.array([(MP2[0]-self.MP1[0]+self.radius_arm2*X[0])/self.radius_arm1,(MP2[0]-self.MP1[0]+self.radius_arm2*X[1])/self.radius_arm1])
        t_1=[i for i in t_1 if i<=1 and i>=-1]#t1 ist der Armwinkel, den das erste Gelenk haben muss, um Punkt anzusteuern.
        #arccos soll gebildet werden, ungünstig, wenn t_1 außerhalb des definitionsbereichs liegt
        X_Koor=np.array([0,0])
        if t_1:
            X=np.arccos(t_1)
            print(t_1)
            x=self.waehle_t1(X)
            print(self.t1_prev,x)
            self.t1_prev=x
            #x=min(X,key=lambda A:abs(A-np.pi/2))
            X_Koor=np.array([self.MP1[0]+self.radius_arm1*np.cos(x),self.MP1[1]+self.radius_arm1*np.sin(x)])
        print(X_Koor)
        return X_Koor
    def pos_in_servo(self,pt1,pt2):
        #Berechung der Armwinkel aus Greifarmpunkten
        arm1=pt1-self.MP1
        arm2=pt2-pt1
        ang1=abs(np.angle(arm1[0]+arm1[1]*1j)-np.angle(1))*180/np.pi
        ang2=(-np.angle(arm2[0]+arm2[1]*1j)+np.angle(arm1[0]+arm1[1]*1j))*180/np.pi
        return ang1,ang2
    def gib_mikrosec(self,pos,debug=True):
        """volle Prozedur, um eine Pixelposition eines ArUcO-MArkers in eine Servoposition umzuwandeln

        Args:
            pos (array(float,2)): Position des Mittelpunkts eines ArUcO-Markers
            debug (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        Winkel=self.pos_in_Winkel(pos)
        posxy=self.Winkel_zu_posxy(Winkel)
        arm1=self.berechne_arm1_Koor(posxy)
        Servowink=self.pos_in_servo(arm1,posxy)
        erg1=550+Servowink[0]*2000/180
        erg2=2600-((((Servowink[1])+22)*2000)/180)
        if debug:
            print("ServoWinkel:",Servowink)
            print("Arm 1:",arm1)
            print("Prosition:",posxy)
            print("Winkel:",Winkel)
            print("Kameraposition:",pos)
            print(erg1,erg2)
        return erg1,erg2
    def schreib(self,st):#Senden einer Seriellen nachrichten an den Raspberry pi Pico und warten auf eine antwort
        self.ser.write((st).encode("utf-8"))
        while(not self.ser.inWaiting()):
            time.sleep(0.3)
        k=self.ser.read(self.ser.inWaiting())
        print("fertig:",st)
        time.sleep(0.5)
    def proz(self,pos):
        """Verfahrensprozedur des Greifarms. Ein ArUcO-Marker wird als Position eingegeben und der Greifarm
        hebt die Probe auf und legt sich woanders wieder hin

        Args:
            pos (array(float,2)): Position der Kamera
        """
        serv1,serv2=self.gib_mikrosec(pos)#Erhalte die Position der Servowinkel, um mit dem Greifarm die Probe aufzuhebenm
        self.schreib("3,"+str(int(serv1)))#Ansteuern Servo 1
        self.schreib("4,"+str(int(serv2)))#Ansteuern Servo 2
        print("durch")
        time.sleep(1)
        self.schreib("0,1")#Anschalten der Pumpe
        time.sleep(0.5)
        self.schreib("1,380")#Schrittmotor runter
        time.sleep(0.5)
        self.schreib("2,380")#Schrittmotor rauf
        self.schreib("4,1400")
        self.schreib("3,800")#Greifarm an rand
        self.schreib("1,100")#ablegen
        self.schreib("0,0")#pumpe aus

        time.sleep(1.5)
        self.schreib("3,900")
        self.schreib("3,700")
        self.schreib("3,900")#Servo wackeln, um probe besser loszuwerden
        print("pumpe aus")
        self.schreib("2,100")
        self.schreib("3,1000")
        self.schreib("4,1500")#zurüsck an startposition
    #def gib_servo(self,ang1,ang2):

    

        

class kam():
    #handler für Kamera
    def __init__(self,cap):
        self.pos=np.array([0,0])
        self.cap=cap
    def trea(self):
        #Dauerprozedur: die Kamera gibt bild aus und dieses wird auf ArUcO-MArker analysiert
        while True:
            suc,img=self.cap.read()
            imag=img.copy()
            arucoParams = cv.aruco.DetectorParameters()

            corners,ids,rejected=cv.aruco.detectMarkers(img, dicter,parameters=arucoParams)
            its=np.array(ids)
            if its.size==0:
                pass
            else:
                if corners:
                    #Marker vorhanden, ausgabe der Pixelposition
                    corner=np.array(corners)[0][0]
                    #print(np.median(corner,axis=0))
                    self.pos=np.median(corner,axis=0)
                    #cv.aruco.drawDetectedMarkers(imag,corners,ids)
                else:
                    self.pos=np.array([0.0,0.0])
            #cv.imshow("ouT",imag)
            #cv.waitKey(1)


cap=cv.VideoCapture("/dev/video2")
dicter=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)  # set new dimensionns to cam object (not cap)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 300)
ind=0
diekam=kam(cap)
t1=Thread(target=diekam.trea)
t1.start()
#t1.join()