#import serial
import numpy as np
import imutils
from imutils.video import FPS
from imutils.video import VideoStream
import cv2
import math
import datetime
import time
#import smbus			#import SMBus module of I2C
from time import sleep          #import

# ############## GPIO ########################
count=0
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pin_bola = [4,17,27]
pin_cyan = [22,10]
 
for pin_io in pin_bola:
      GPIO.setup(pin_io,GPIO.OUT)
for pin_ioo in pin_cyan:
      GPIO.setup(pin_ioo,GPIO.OUT)

import os
import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250



def gyro(count):
                
      
                imu.readSensor()
                imu.computeOrientation()
                
                print(f"x={int(imu.roll)} | y={int(imu.pitch)} | z={int(imu.yaw)}")
                if (int(imu.roll)<140 and int(imu.roll)>0):
                        print("depan")                        
                        #print(count)
                        if (count%5)==0:
                              #print("fix depan")
                              kirim_ke_opencm_2023(int(7))
                elif (int(imu.roll)>=-140 and int(imu.roll)<0):
                        print("belakang")
                        kirim_ke_opencm_2023(int(7))
                elif (int(imu.pitch)>35):
                        print("kanan")
                        #kirim_ke_opencm_2023(int(7))
                elif ((int(imu.pitch)<-35)):
                        print("kiri")
                        #kirim_ke_opencm_2023(int(7))
                else:
                        print("aman")
                        koor_bola_penuh()
                        

################################################################################
databall,datafield,datacyan,datamagenta=0,0,0,0

##Camera Parameter 
im_widht=640  #Lebar Frame
im_height=480 #Tinggi Frame
center_im=im_widht/2,im_height/2 #Titik Tengah Frame
cameranum=0#Index Kamera
camera = VideoStream(src=cameranum).start()####


time.sleep(2.0)
fps = FPS().start()
################################################################################
##Variable Umum
font = cv2.FONT_HERSHEY_SIMPLEX
TextMode=0
count=0
mode=0
data=0
nil_blur=1
################################################################################



print (" Reading Data of Gyroscope and Accelerometer")



##Function Umum
'''def kirim(a,b,c,d,e):
        client = redis.Redis(host='localhost',port=6379,db=0)
        client.set('sudut_ball',a)
        client = redis.Redis(host='localhost',port=6379,db=0)
        client.set('Goal_cyan',b)
        client = redis.Redis(host='localhost',port=6379,db=0)
        client.set('Goal_magenta',c)
        client = redis.Redis(host='localhost',port=6379,db=0)
        client.set('jarak_cyan',d)
        client = redis.Redis(host='localhost',port=6379,db=0)
        client.set('jarak_magenta',e)'''
        
def jarak(ball_x,ball_y):
        global center_im
        nil_x= int(ball_x-center_im[0])
        nil_y= int(ball_y-center_im[1])

        jarak= math.sqrt(pow(nil_x,2)+pow(nil_y,2))
        jarak= 0.2884*jarak
        jarak= ((1*(pow(jarak,2))+(1*jarak)+5))/10
        return jarak
        
def sudut(ball_x,ball_y,mid_x,mid_y,titik_depan_x,titik_depan_y):
        u1=ball_x-mid_x
        u2=ball_y-mid_y

        temp_ball_x=titik_depan_x
        temp_ball_y=titik_depan_y
        

        #ball_distance_pixel = sqrt(pow((temp_ball_x - ball_x), 2) + pow((temp_ball_y - ball_y), 2));
        #ball_tetha = atan2((temp_ball_y - ball_y), -(temp_ball_x - ball_x)) * 57.295779513;
        #ball_distance_real = convert_distance2real(ball_distance_pixel);
        
        #Implementasi Dalil pythagoras cari jarak
        sisib=u1 #
        if sisib<0:
                sisib=sisib*-1

        sisia=u2
        if sisia<0:
                sisia=sisia*-1

        sisic=math.sqrt(math.pow(sisia,2)+math.pow(sisib,2))
        
        #Input(jarak_pixel)=((jarak_pixel*x_variabel_1)+intercept) #Convert jarak pixel ke jarak cm
        #Sudut arah bola
        #Derajat/sudut_a=(sisi_datar_AC*sudut_C)/sisi_miring_AB
        sudut_a=(sisib*90)/sisic

        #Mengubah ke derajat 0-180
        if ball_x < 320:
                derajat=(sisib*90)/sisic
                
        else:
                derajat=((sisib*90)/sisic)+90
                
                

        #print(sisia,sisib)

        if(u1>0):
                titik_depan_x=320
                titik_depan_y=0
                a=1
        elif(u1<0):
                titik_depan_x=160
                titik_depan_y=450
                a=-1
        elif(u1==0):
                if(u2>0):
                    sudut=0
                    a=1
                    return sudut
                if(u2<0):
                    sudut=180
                    a=-1
                    return sudut
                
        v1=titik_depan_x-mid_x
        v2=titik_depan_y-mid_y

        Vect_ball= math.sqrt(pow(u1,2)+pow(u2,2))
        Vect_depan= math.sqrt(pow(v1,2)+pow(v2,2))

        titik_u_v = (u1*v1)+(u2*v2)

        bagi= titik_u_v/(Vect_ball*Vect_depan)

        degree= math.acos(bagi)

        sudut= math.degrees(degree)
        
        angle = math.atan2(ball_y,ball_x)
        angles = angle*180/3.14159265358979323846
        
       
        #if(a==-1):
                #sudut=0
                #sudut=sudut+180

        return int(derajat)


################################################################################
##Menu 
def progHelp():
	print('')
	print('-----------Instruction----------------')
	print('Mulai Deteksi--------------------- [M]')
	print('Setting Threshold----------------- [=]')
	print('Sett Threshold Final-------------- [-]')
	print('Cek Threshold--------------------- [1]')
	print('Destroy All Windows -------------- [Q]')
	print('')
	return None
    
def Help(frame):
        putText(frame,'HELP',130,20,0,0,255,0.8,3)
        putText(frame,'1. Start         [Press M]',10,50,0,0,255,0.5,2)
        putText(frame,'2. Set Threshold [Press =]',10,70,0,0,255,0.5,2)
        putText(frame,'3. Cek Threshold [Press 1]',10,90,0,0,255,0.5,2)
        putText(frame,'4. HELP',10,110,0,0,255,0.5,2)
        cv2.imshow('Help',frame)
    
def putText(frame,word,x,y,b,g,r,height,widht):
    textLine1 = '%s'%(word)
    cv2.putText(frame,textLine1,(int(x),int(y)),
            cv2.FONT_HERSHEY_SIMPLEX,height,(b,g,r),widht)
    return None

################################################################################
##Program Trackbar dan Setting Threshold
def setControl(count):
    global Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin,Ebsize,Dbsize
    global Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin,Efsize,Dfsize
    global Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin,Egsize,Dgsize
    global Hcmax,Hcmin,Scmax,Scmin,Vcmax,Vcmin,Ecsize,Dcsize
    global Hmmax,Hmmin,Smmax,Smmin,Vmmax,Vmmin,Emsize,Dmsize

    if count==0:
            a=Hbmin
            b=Hbmax
            c=Sbmin
            d=Sbmax
            e=Vbmin
            f=Vbmax
            g=Ebsize
            h=Dbsize
            i=b_blur
    elif count==1:
            a=Hfmin
            b=Hfmax
            c=Sfmin
            d=Sfmax
            e=Vfmin
            f=Vfmax
            g=Efsize
            h=Dfsize
            i=f_blur
    elif count==2:
            a=Hgmin
            b=Hgmax
            c=Sgmin
            d=Sgmax
            e=Vgmin
            f=Vgmax
            g=Egsize
            h=Dgsize
            i=g_blur
    elif count==3:
            a=Hcmin
            b=Hcmax
            c=Scmin
            d=Scmax
            e=Vcmin
            f=Vcmax
            g=Ecsize
            h=Dcsize
            i=c_blur
    elif count==4:
            a=Hmmin
            b=Hmmax
            c=Smmin
            d=Smmax
            e=Vmmin
            f=Vmmax
            g=Emsize
            h=Dmsize
            i=m_blur
            
    def nothing(x):
        pass
        
    try:
            cv2.namedWindow('Control')
            cv2.resizeWindow('Control',300,250)
            cv2.createTrackbar('Hmin','Control',a,255,nothing)
            cv2.createTrackbar('Hmax','Control',b,255,nothing)
            cv2.createTrackbar('Smin','Control',c,255,nothing)
            cv2.createTrackbar('Smax','Control',d,255,nothing)
            cv2.createTrackbar('Vmin','Control',e,255,nothing)
            cv2.createTrackbar('Vmax','Control',f,255,nothing)
            cv2.namedWindow('Size')
            cv2.resizeWindow('Size',300,100)
            cv2.createTrackbar('Erosi','Size',g,10,nothing)
            cv2.createTrackbar('Dilasi','Size',h,10,nothing)
            cv2.createTrackbar('Blur','Size',i,25,nothing)
    except:
            cv2.namedWindow('Control')
            cv2.resizeWindow('Control',300,250)
            cv2.createTrackbar('Hmin','Control',Hbmin,255,nothing)
            cv2.createTrackbar('Hmax','Control',Hbmax,255,nothing)
            cv2.createTrackbar('Smin','Control',Sbmin,255,nothing)
            cv2.createTrackbar('Smax','Control',Sbmax,255,nothing)
            cv2.createTrackbar('Vmin','Control',Vbmin,255,nothing)
            cv2.createTrackbar('Vmax','Control',Vbmax,255,nothing)
            cv2.namedWindow('Size')
            cv2.resizeWindow('Size',300,100)
            cv2.createTrackbar('Erosi','Size',Ebsize,10,nothing)
            cv2.createTrackbar('Dilasi','Size',Dbsize,20,nothing)
            cv2.createTrackbar('Blur','Size',b_blur,25,nothing)    
    return None

def SettTrackbars(a,b,c,d,e,f,g,h,i):
    cv2.setTrackbarPos('Hmin','Control',a)
    cv2.setTrackbarPos('Hmax','Control',b)
    cv2.setTrackbarPos('Smin','Control',c)
    cv2.setTrackbarPos('Smax','Control',d)
    cv2.setTrackbarPos('Vmin','Control',e)
    cv2.setTrackbarPos('Vmax','Control',f)
    cv2.setTrackbarPos('Erosi','Size',g)
    cv2.setTrackbarPos('Dilasi','Size',h)
    cv2.setTrackbarPos('Blur','Size',i)
    return None

def settThreshold():
    global Hmax,Hmin,Smax,Smin,Vmax,Vmin,Esize,Dsize,EFinalsize,DFinalsize
    global Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin,Ebsize,Dbsize,b_blur
    global Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin,Efsize,Dfsize,f_blur
    global Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin,Egsize,Dgsize,g_blur
    global Hcmax,Hcmin,Scmax,Scmin,Vcmax,Vcmin,Ecsize,Dcsize,c_blur
    global Hmmax,Hmmin,Smmax,Smmin,Vmmax,Vmmin,Emsize,Dmsize,m_blur
    global frame,count,mode,TextMode,error,nil_blur
    
    loadConfig()
    setControl(0)
    nil_Help=0
    while(1):            
        Hmin= cv2.getTrackbarPos('Hmin','Control')
        Hmax= cv2.getTrackbarPos('Hmax','Control')
        Smin= cv2.getTrackbarPos('Smin','Control')
        Smax= cv2.getTrackbarPos('Smax','Control')
        Vmin= cv2.getTrackbarPos('Vmin','Control')
        Vmax= cv2.getTrackbarPos('Vmax','Control')
        Esize= cv2.getTrackbarPos('Erosi','Size')
        Dsize= cv2.getTrackbarPos('Dilasi','Size')
        nil_blur= cv2.getTrackbarPos('Blur','Size')
        nil_blur= (nil_blur*2)+1

        Lower_val = np.array([Hmin,Smin,Vmin])
        Upper_val = np.array([Hmax,Smax,Vmax])

        gatherThresh() 
        
        #(grabbed,frame) = camera.read()
        frame = camera.read() 
        frame = imutils.resize(frame, width=im_widht)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #blur = cv2.medianBlur(frame,nil_blur)
        try:
                blur = cv2.GaussianBlur(hsv,(nil_blur,nil_blur),0)
        except:
                pass
                      
        if mode==5:
                pass
        else:
                mask  = parseField(blur,Lower_val,Upper_val,Esize,Dsize)        

        if nil_Help==1:
            img = cv2.imread('gambar1.jpg')
            img = imutils.resize(img, width=300)
            Help(img)
            
        if mode==0:
            try:
                color=(0,0,255)
                dataCountor=Contour(mask,color,0)
                error=False
            except:
                error=True
        elif mode==1:
            try:
                color=(255,0,0)
                dataCountor=Contour(mask,color,1)
                error=False
            except:
                error=True
        elif mode==2:
            try:
                print(lineDetection(mask))
                error=False
            except:
                error=True
        elif mode==3:
            try:
                mask1 = parseField(blur,Lower_val,Upper_val,Esize,Dsize)
                mask2 = parseField(blur,f_Lower_val,f_Upper_val,Efsize,Dfsize)
                combine=combineMask(mask2,mask1)
                putText(frame,'Combine',230,430,0,255,0,0.7,3)
                cv2.imshow('Kombinasi',combine)
                error=False
            except:
                error=True
                
        if count==0:
            putText(frame,'BALL',230,20,3,165,252,0.8,2)
        elif count==1:
            putText(frame,'FIELD',230,20,0,255,0,0.8,2)
        elif count==2:
            putText(frame,'GOAL',230,20,255,255,0,0.8,2)
        elif count==3:
            putText(frame,'CYAN',230,20,255,255,0,0.8,2)
        elif count==4:
            putText(frame,'MAGENTA',230,20,255,0,255,0.8,2)

        if error==True:
                setControl(count)
                print('error '+ str(mode))
        else:
                pass
        
        try:
            cv2.imshow('mask',mask)
            degree=sudut(dataCountor[0],dataCountor[1],center_im[0],center_im[1],center_im[0],0)
            text = 'Sudut=  %s'%(degree)
            putText(frame,text,10,50,0,0,255,0.5,2)
        except:
            pass
        mask = cv2.inRange(hsv,Lower_val,Upper_val)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('biar gampang',res)
        #cv2.imshow('blur',blur)
        cv2.imshow('asli',frame) #### res= frame tadi tapi

                              
        k=cv2.waitKey(1) &0xFF
        if k==ord('z'):
            Lower_val=np.array([0,0,0])
            Upper_val=np.array([255,255,255])
            Esize=0
            Dsize=0
            nil_blur=0
            SettTrackbars(Lower_val[0],Upper_val[0],Lower_val[1],Upper_val[1],Lower_val[2],Upper_val[2],Esize,Dsize,nil_blur)
        if k==ord('s'):
            if count==0:
                Hbmin=Hmin
                Hbmax=Hmax
                Sbmin=Smin
                Sbmax=Smax
                Vbmin=Vmin
                Vbmax=Vmax
                Ebsize=Esize
                Dbsize=Dsize
                b_blur=nil_blur
                saveConfig()
            elif count==1:
                Hfmin=Hmin
                Hfmax=Hmax
                Sfmin=Smin
                Sfmax=Smax
                Vfmin=Vmin
                Vfmax=Vmax
                Efsize=Esize
                Dfsize=Dsize
                f_blur=nil_blur
                saveConfig()
            elif count==2:
                Hgmin=Hmin
                Hgmax=Hmax
                Sgmin=Smin
                Sgmax=Smax
                Vgmin=Vmin
                Vgmax=Vmax
                Egsize=Esize
                Dgsize=Dsize
                g_blur=nil_blur
                saveConfig()
            elif count==3:
                Hcmin=Hmin
                Hcmax=Hmax
                Scmin=Smin
                Scmax=Smax
                Vcmin=Vmin
                Vcmax=Vmax
                Ecsize=Esize
                Dcsize=Dsize
                c_blur=nil_blur
                saveConfig()
            elif count==4:
                Hmmin=Hmin
                Hmmax=Hmax
                Smmin=Smin
                Smmax=Smax
                Vmmin=Vmin
                Vmmax=Vmax
                Emsize=Esize
                Dmsize=Dsize
                m_blur=nil_blur
                saveConfig()
        elif k== ord('1'):
            count=count+1
            if count>=5:
                count=0
            if count==0 :
                Lower_val=b_Lower_val
                Upper_val=b_Upper_val
                Esize=Ebsize
                Dsize=Dbsize
                nil_blur=b_blur
                print('Load Ball')
            elif count==1 :
                Lower_val=f_Lower_val
                Upper_val=f_Upper_val
                Esize=Efsize
                Dsize=Dfsize
                nil_blur=f_blur
                print('Load Field')
            elif count==2 :
                Lower_val=g_Lower_val
                Upper_val=g_Upper_val
                Esize=Egsize
                Dsize=Dgsize
                nil_blur=g_blur
                print('Load Goal')
            elif count==3 :
                Lower_val=c_Lower_val
                Upper_val=c_Upper_val
                Esize=Ecsize
                Dsize=Dcsize
                nil_blur=c_blur
                print('Load Cyan')
            elif count==4 :
                Lower_val=m_Lower_val
                Upper_val=m_Upper_val
                Esize=Emsize
                Dsize=Dmsize
                nil_blur=m_blur
                print('Load Magenta')
            SettTrackbars(Lower_val[0],Upper_val[0],Lower_val[1],Upper_val[1],Lower_val[2],Upper_val[2],Esize,Dsize,nil_blur)
        elif k== ord('5'):
            loadConfig()
            mode=mode+1
            if mode>=4:
                mode=0
            print('Load Countor')
            cv2.destroyAllWindows()
            gatherThresh()                    
        elif k== ord('h'):
            nil_Help=nil_Help+1
            if nil_Help>=2:
                    nil_Help=0
            if nil_Help==0:
                    cv2.destroyAllWindows()
        elif k== ord('q'):
            cv2.destroyAllWindows()
            break

################################################################################
##Program Save to Txt
def loadConfig():
        global Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin,Ebsize,Dbsize,b_blur,EFinalsize,DFinalsize
        global Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin,Efsize,Dfsize,f_blur
        global Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin,Egsize,Dgsize,g_blur
        global Hcmax,Hcmin,Scmax,Scmin,Vcmax,Vcmin,Ecsize,Dcsize,c_blur
        global Hmmax,Hmmin,Smmax,Smmin,Vmmax,Vmmin,Emsize,Dmsize,m_blur
        f = open("dataThreshold.txt","r")
        for line in f.readlines():
		#print line
                arr_read = line.split(',')
                
                Hbmax = int(arr_read[0])
                Hbmin = int(arr_read[1])
                Sbmax = int(arr_read[2])
                Sbmin = int(arr_read[3])
                Vbmax = int(arr_read[4])
                Vbmin = int(arr_read[5])

                Hfmax = int(arr_read[6])
                Hfmin = int(arr_read[7])
                Sfmax = int(arr_read[8])
                Sfmin = int(arr_read[9])
                Vfmax = int(arr_read[10])
                Vfmin = int(arr_read[11])

                Hgmax = int(arr_read[12])
                Hgmin = int(arr_read[13])
                Sgmax = int(arr_read[14])
                Sgmin = int(arr_read[15])
                Vgmax = int(arr_read[16])
                Vgmin = int(arr_read[17])

                Ebsize = int(arr_read[18])
                Dbsize = int(arr_read[19])
                Efsize = int(arr_read[20])
                Dfsize = int(arr_read[21])
                Egsize = int(arr_read[22])
                Dgsize = int(arr_read[23])
                EFinalsize = int(arr_read[24])
                DFinalsize = int(arr_read[25])

                b_blur = int(arr_read[26])
                f_blur = int(arr_read[27])
                g_blur = int(arr_read[28])

                Hcmax = int(arr_read[29])
                Hcmin = int(arr_read[30])
                Scmax = int(arr_read[31])
                Scmin = int(arr_read[32])
                Vcmax = int(arr_read[33])
                Vcmin = int(arr_read[34])

                Hmmax = int(arr_read[35])
                Hmmin = int(arr_read[36])
                Smmax = int(arr_read[37])
                Smmin = int(arr_read[38])
                Vmmax = int(arr_read[39])
                Vmmin = int(arr_read[40])

                Ecsize = int(arr_read[41])
                Dcsize = int(arr_read[42])
                Emsize = int(arr_read[43])
                Dmsize = int(arr_read[44])
                c_blur = int(arr_read[45])
                m_blur = int(arr_read[46])
                f.close

        print ('loaded')
        return None

def saveConfig():
        f = open("dataThreshold.txt","w")
        data0= '%d,%d,%d,%d,%d,%d'%(Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin)
        data1= '%d,%d,%d,%d,%d,%d'%(Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin)
        data2= '%d,%d,%d,%d,%d,%d'%(Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin)
        data3= '%d,%d,%d,%d,%d,%d'%(Ebsize,Dbsize,Efsize,Dfsize,Egsize,Dgsize)
        data4= '%d,%d,%d,%d,%d'%(EFinalsize,DFinalsize,b_blur,f_blur,g_blur)
        data5= '%d,%d,%d,%d,%d,%d'%(Hcmax,Hcmin,Scmax,Scmin,Vcmax,Vcmin)
        data6= '%d,%d,%d,%d,%d,%d'%(Hmmax,Hmmin,Smmax,Smmin,Vmmax,Vmmin)
        data7= '%d,%d,%d,%d,%d,%d'%(Ecsize,Dcsize,Emsize,Dmsize,c_blur,m_blur)
        data=data0+','+data1+','+data2+','+data3+','+data4+','+data5+','+data6+','+data7
        f.write(data)
        f.close()
        print(data)
        print ('saved')
        return None

def gatherThresh():
    global Hmax,Hmin,Smax,Smin,Vmax,Vmin,Esize,Dsize,EFinalsize,DFinalsize
    global Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin,Ebsize,Dbsize
    global Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin,Efsize,Dfsize
    global Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin,Egsize,Dgsize
    global Hcmax,Hcmin,Scmax,Scmin,Vcmax,Vcmin,Ecsize,Dcsize
    global Hmmax,Hmmin,Smmax,Smmin,Vmmax,Vmmin,Emsize,Dmsize
    global b_Lower_val,b_Upper_val,f_Lower_val,f_Upper_val,g_Lower_val,g_Upper_val,c_Lower_val,c_Upper_val,m_Lower_val,m_Upper_val
    b_Lower_val = np.array([Hbmin,Sbmin,Vbmin])
    b_Upper_val = np.array([Hbmax,Sbmax,Vbmax])
    f_Lower_val = np.array([Hfmin,Sfmin,Vfmin])
    f_Upper_val = np.array([Hfmax,Sfmax,Vfmax])
    g_Lower_val = np.array([Hgmin,Sgmin,Vgmin])
    g_Upper_val = np.array([Hgmax,Sgmax,Vgmax])
    c_Lower_val = np.array([Hcmin,Scmin,Vcmin])
    c_Upper_val = np.array([Hcmax,Scmax,Vcmax])
    m_Lower_val = np.array([Hmmin,Smmin,Vmmin])
    m_Upper_val = np.array([Hmmax,Smmax,Vmmax])
    
################################################################################
##Program Function
def Contour(mask,color,mode):
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    cnts= cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if mode==0:
        if len(contours) > 0:
            cntr = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cntr)
            hull = cv2.convexHull(cntr)
            mask = np.zeros(frame.shape[:2], np.uint8)
            #mask = np.zeros([im_height, im_width, 3], dtype=np.uint8)
            #mask[100:300, 100:400] = 255
            #cv2.drawContours(mask, [hull], 0, 255,cv2.FILLED)
            #field_image = cv2.bitwise_and(frame,frame,mask = mask)
            cv2.drawContours(frame, [hull], 0, color ,2)
            cv2.rectangle(frame, (int(x) - 3, int(y) - 3), (int(x) + 3, int(y) + 3), (0, 0, 255), -1)
            return x,y,int(radius)
                
    elif mode==1:
        nilPixel_gcontour=0
        if len(contours) > 0:
            cntr = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cntr)
            hull = cv2.convexHull(cntr)
            mask = np.zeros(frame.shape[:2], np.uint8)
            mask = np.zeros([im_height, im_widht, 3], dtype=np.uint8)
            cv2.drawContours(frame, [hull], 0, color,2)
            cv2.rectangle(frame, (int(x) - 3, int(y) - 3), (int(x) + 3, int(y) + 3), (0, 0, 255), -1)
            nilPixel_gcontour=len(cntr)
            #for i in len(cntr):
            #    nilPixel_gcontour=nilPixel_gcontour+1
            return x,y,int(radius),nilPixel_gcontour
                        
            
    elif mode==2:
        if len(contours) > 0:
            cntr = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cntr)
            M = cv2.moments(b_cntr)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.drawContours(frame, [hull], 0, color ,2)
            if radius > 5:
                    cv2.circle(frame, (int(ball_x), int(ball_y)), int(radius),
                    (0, 0, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    cv2.putText(frame,"bola",(int(ball_x-radius),int(ball_y-radius)),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

    elif mode==3:
        if len(cnts) > 0:
                hull=[]
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                try:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                except:
                    pass
                for i in range(len(cnts)):
                        hull.append(cv2.convexHull(cnts[i],False))
                        color_contours=color
                        color=(255,0,0)
                        cv2.drawContours(frame,cnts,i,color_contours,3,8)
                        #cv2.drawContours(image,hull,i,color,3,8)
        
def ContourMask(mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(contours) > 0:
            cntr = max(contours, key=cv2.contourArea)
            Kordinat  = cv2.minEnclosingCircle(cntr)
            hull = cv2.convexHull(cntr)
            cv2.drawContours(mask, [hull], 0, (255,255,255),-1)
            return Kordinat

        
def MaskFieldBall(maskf,maskb,mode):
        contours_f = cv2.findContours(maskf.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(contours_f) > 0:
            cntr_f = max(contours_f, key=cv2.contourArea)
            Kordinat_f  = cv2.minEnclosingCircle(cntr_f)
            hull_f = cv2.convexHull(cntr_f)
            cv2.drawContours(maskf, [hull_f], 0, (255,255,255),-1)
            M_f = cv2.moments(cntr_f)

        contours_b = cv2.findContours(maskb.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(contours_b) > 0:
            cntr_b = max(contours_b, key=cv2.contourArea)
            Kordinat_b  = cv2.minEnclosingCircle(cntr_b)
            
        try:
                final=cv2.bitwise_and(maskf,maskb.copy())
                contours = cv2.findContours(final, cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)[-2]
                center = None
                if len(contours_f) > 0:
                    cntr = max(contours, key=cv2.contourArea)
                    Kordinat  = cv2.minEnclosingCircle(cntr)
                    ((x, y), radius) = cv2.minEnclosingCircle(cntr)
                    hull = cv2.convexHull(cntr)
                    cv2.drawContours(final, [hull], 0, (255,255,255),-1)
                    M = cv2.moments(cntr)
                    detect_final=True
                
                
        except:
                final= np.zeros(img.shape, dtype = "uint8")
                detect_final=False
                
        if mode==0:                
                try:
                        if detect_final==True:
                                cv2.drawContours(frame, [hull_f], 0, (0,255,0) ,2)
                                center_f = (int(M_f["m10"] / M_f["m00"]), int(M_f["m01"] / M_f["m00"]))
                                cv2.circle(frame, center_f, 5, (0, 255, 0), -1)
                                try:
                                    kordinat=Kordinat_f
                                except:
                                    kordinat=(('x','x'),'x')
                                    pass
                except:
                        kordinat=(('0','0'),'0')
                        pass
        if mode==1:                
                try:
                        if detect_final==True:
                                cv2.drawContours(frame, [hull], 0, (0,0,0) ,2)
                                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                                
                                #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                                #cv2.circle(frame, center, 3, (0, 0, 255), -1)
                                #cv2.line(frame,(319,480), center,(255,0,0),1)
                                
                                #cv2.putText(frame,"Ball", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 0),1,5)
                                cv2.putText(frame,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255, 0, 0),1)
                                cv2.circle(frame, center, 5, (0, 255, 255), -1)
                                            
                                try:
                                    kordinat=Kordinat
                                except:
                                    kordinat=((0,0),'0')
                                    pass
                except:
                        kordinat=((0,0),'0')
                        pass
                
        try:
                return kordinat[0],maskb,maskf,final
        except:
                kordinat=((0,0),'0')
                return kordinat[0],maskb,maskf,final

def urutContor():
        contours_f = cv2.findContours(maskf.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(contours_f) > 0:
                cnts= sorted(cnts,key=cv2.contourArea, reverse=True)[:10]
                for i in range(0,3):
                        ((x[i],y[i]),radius[i])=cv2.minEnclosingCircle(cnts[i])
                        for i in range(0,3):
                                M= cv2.moments(cnts[i])
                                center[i]=(int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                                
                                           
def combineMask(maskf,mask):
        cont  = ContourMask(mask)
        cont1 = ContourMask(maskf)
        combine = cv2.bitwise_and(cont,cont1)
        return combine

def lineDetection(mask):
    minLineLength = 300
    maxLineGap = 20
    lines = cv2.HoughLinesP(mask,1,np.pi/180,100,minLineLength,maxLineGap)
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
        #print (panjang(x1,y1,x2,y2))
        return x1,y1,x2,y2
        
def parseField(hsv,Lower_val,Upper_val,Esize,Dsize):
    mask = cv2.inRange(hsv, Lower_val, Upper_val)
    try:
        mask = cv2.erode(mask, None, iterations=Esize)
        mask = cv2.dilate(mask, None, iterations=Dsize)
        return mask
    except:
        pass


def map(nilai,OldMin,OldMax,NewMin,NewMax):
    NewValue = (((nilai - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue

################################################################################
##Sub Program
def TrackTanding():
        global Hmax,Hmin,Smax,Smin,Vmax,Vmin,Esize,Dsize,EFinalsize,DFinalsize
        global Hbmax,Hbmin,Sbmax,Sbmin,Vbmax,Vbmin,Ebsize,Dbsize,b_blur
        global Hfmax,Hfmin,Sfmax,Sfmin,Vfmax,Vfmin,Efsize,Dfsize,f_blur
        global Hgmax,Hgmin,Sgmax,Sgmin,Vgmax,Vgmin,Egsize,Dgsize,g_blur
        global Hlmax,Hlmin,Slmax,Slmin,Vlmax,Vlmin,Elsize,Dlsize,l_blur
        global frame,count,mode,TextMode,error,nil_blur,SSE_Thresh,Max_Pixel,Min_Pixel,Max_Radius,Min_Radius,erorx,erory,nilaiKotak
        
        gatherThresh()
        while(1):
                #(grabbed,frame) = camera.read()
                frame = camera.read()
                frame = imutils.resize(frame, width=im_widht)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                #blur_ball = cv2.GaussianBlur(hsv.copy(),(b_blur,b_blur),0)
                mask_ball  = parseField(hsv.copy(),b_Lower_val,b_Upper_val,Ebsize,Dbsize) 
                #blur_field = cv2.GaussianBlur(hsv.copy(),(f_blur,f_blur),0)
                mask_field  = parseField(hsv.copy(),f_Lower_val,f_Upper_val,Efsize,Dfsize)
                #blur_goal = cv2.GaussianBlur(hsv.copy(),(g_blur,g_blur),0)
                mask_goal  = parseField(hsv.copy(),g_Lower_val,g_Upper_val,Egsize,Dgsize)
                #blur_cyan = cv2.GaussianBlur(hsv.copy(),(c_blur,c_blur),0)
                mask_cyan  = parseField(hsv.copy(),c_Lower_val,c_Upper_val,Ecsize,Dcsize)
                #blur_magenta = cv2.GaussianBlur(hsv.copy(),(m_blur,m_blur),0)
                mask_magenta  = parseField(hsv.copy(),m_Lower_val,m_Upper_val,Emsize,Dmsize)
                kosong = parseField(hsv.copy(),(0,0,0),(0,0,0),0,0)

                try:
                        maskMagen= ContourMask(mask_magenta)
                        center= maskMagen[0]
                        cx=int(center[0])
                        cy=int(center[1])
                             

                except:
                        cx=0
                        cy=0
                        pass

                
                ##########################################################

                #Koordinat_ball  
                kordinat_ball=MaskFieldBall(mask_field,mask_ball,1)
                x_ball,y_ball=kordinat_ball[0]
                try:
                        #cv2.line(frame,(319,480), (x_ball,y_ball),(255,0,0),1)
                        degree_ball=sudut(x_ball,y_ball,center_im[0],center_im[1],center_im[0],0)
                        #putText(frame,'Ball',x_ball,y_ball,0,0,255,0.5,2)
                except:
                        degree_ball='500'
               
                #Koordinat_Goal_Cyan
                cyan_Goal=MaskFieldBall(mask_field,mask_cyan,1)
                x_Goal_cyan,y_Goal_cyan=cyan_Goal[0]
                try:
                    degree_Goal_cyan=sudut(x_Goal_cyan,y_Goal_cyan,center_im[0],center_im[1],center_im[0],0)
                    #putText(frame,'Goal_cyan',x_Goal_cyan,y_Goal_cyan,255,255,0,0.5,2)
                except:
                    degree_Goal_cyan='500'
                    
                    
                #Koordinat_Goal_magenta
                #magenta_Goal=MaskFieldBall(kosong,mask_goal,0)
                magenta_Goal= MaskFieldBall(mask_field,mask_magenta,1)
                x_Goal_magenta,y_Goal_magenta=magenta_Goal[0]
                try:
                        
                    degree_Goal_magenta=sudut(x_Goal_magenta,y_Goal_magenta,center_im[0],center_im[1],center_im[0],0)
                    #putText(frame,'Goal_magenta',x_Goal_magenta,y_Goal_magenta,255,0,255,0.5,2) 
                    
                except:
                        
                        degree_Goal_magenta='500'

                #Jarak_Goal_cyan
                titik_kordinat_cyan= cyan_Goal[0]
                t_x_cyan= titik_kordinat_cyan[0]
                t_y_cyan= titik_kordinat_cyan[1]
                try:
                        jarak_goal_cyan=int(jarak(t_x_cyan,t_y_cyan))
                except:
                        jarak_goal_cyan=999

                #jarak_Goal_magenta
                titik_kordinat_magenta= magenta_Goal[0]
                t_x_magenta= titik_kordinat_magenta[0]
                t_y_magenta= titik_kordinat_magenta[1]
                try:
                        jarak_goal_magenta=int(jarak(t_x_magenta,t_y_magenta))
                except:
                        jarak_goal_magenta=999
                
                text1 = 'Sudut_Ball=  %s'%(str(degree_ball))
                putText(frame,text1,10,20,3,165,252,0.5,2)
                text2 = 'Cyan_Goal=  %s'%(str(degree_Goal_cyan))
                putText(frame,text2,10,40,255,255,0,0.5,2)
                text3 = 'Magenta_Goal=  %s'%(str(degree_Goal_magenta))
                putText(frame,text3,10,60,255,0,255,0.5,2)
                text4 = 'Jarak_Cyan=  %s'%(str(int(jarak_goal_cyan)))
                putText(frame,text4,10,80,255,255,0,0.5,2)
                text5 = 'Jarak_Magenta=  %s'%(str(int(jarak_goal_magenta)))
                putText(frame,text5,10,100,255,0,255,0.5,2)
                

            
                data=[databall,datafield,datacyan,datamagenta]
                xball_eror = int(x_ball)
                yball_eror = int(y_ball)
                xcyan_eror = int(x_Goal_cyan)
                ycyan_eror = int(y_Goal_cyan)
                xmagenta_eror = int(x_Goal_magenta)
                ymagenta_eror = int(y_Goal_magenta)
                # xcyan_eror = int(320-int(x_Goal_cyan))
                # ycyan_eror = int(240-int(y_Goal_cyan))
                # xmagenta_eror = int(320-int(x_Goal_magenta))
                # ymagenta_eror = int(240-int(y_Goal_magenta))

                erorx =[xball_eror,xcyan_eror,xmagenta_eror]
                erory =[yball_eror,ycyan_eror,ymagenta_eror]              
                Display_Kotak(3,4)  
                cv2.imshow('asli',frame)
                
                print(f"{xball_eror} | {yball_eror}")
               
                koor_bola_penuh()
                
                                  
                
                
                k=cv2.waitKey(1)& 0xFF
                if k==ord('q'):
                        cv2.destroyAllWindows()
                        # for i in range(7):
                             # GPIO.output(pin[i], 0) 
                        # break
                fps.update()

def koor_bola_penuh():
        global asd,ads,dataFrame,tengah_bola_x,tengah_bola_y,tengah_magenta_x,tengah_magenta_y,frame
        gatherThresh()
        frame = camera.read()
        frame = imutils.resize(frame, width=im_widht)

        # cv2.line(frame,(213,0), (213,480),(255,255,255),2) 
        # cv2.line(frame,(426,0), (426,480),(255,255,252),2) 
        # cv2.line(frame,(0,240), (640,240),(255,255,252),2) 

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_ball  = parseField(hsv.copy(),b_Lower_val,b_Upper_val,Ebsize,Dbsize) 
        mask_field  = parseField(hsv.copy(),f_Lower_val,f_Upper_val,Efsize,Dfsize) 
        mask_magenta  = parseField(hsv.copy(),m_Lower_val,m_Upper_val,Emsize,Dmsize) 
        mask_cyan  = parseField(hsv.copy(),c_Lower_val,c_Upper_val,Ecsize,Dcsize) 
        
       
        adudufont = cv2.FONT_HERSHEY_SIMPLEX
      
        _, aduduthreshold = cv2.threshold(mask_ball, 110, 255, cv2.THRESH_BINARY)
        _, aduduthreshold_field = cv2.threshold(mask_field, 110, 255, cv2.THRESH_BINARY)
        _, aduduthreshold_magenta = cv2.threshold(mask_magenta, 110, 255, cv2.THRESH_BINARY)
        
        contours, _= cv2.findContours(aduduthreshold, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
        contours_field, _= cv2.findContours(aduduthreshold_field, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
        contours_magenta, _= cv2.findContours(aduduthreshold_magenta, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
        
        asd = []
        ads = []
        for cnt in contours :
                
                approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
                
                cv2.drawContours(frame, [approx], 0, (0, 0, 255), 3) 
                
                n = approx.ravel() 
                i = 0
                
                
                for j in n :
                        if(i % 2 == 0):
                                x_bola_penuh = int(n[i])
                                y_bola_penuh = int(n[i + 1])
                        
                                string = str(x_bola_penuh) + " " + str(y_bola_penuh) 
                        
                                # if(i == 0):
                                #     # text on topmost co-ordinate.
                                #     cv2.putText(aduduimg2, "=", (x, y),
                                #                     adudufont, 0.5, (255, 0, 0)) 
                                # if(i != 0):
                                        
                                #         cv2.putText(frame, string, (x_bola_penuh, y_bola_penuh), 
                                #                 adudufont, 0.5, (0, 255, 0))
                                asd.append(x_bola_penuh)
                                ads.append(y_bola_penuh)
            
                        i = i + 1
        asd_field = []
        ads_field = []
        for cnt in contours_field :
                
                approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
                
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3) 
                
                n = approx.ravel() 
                i = 0
                
                
                for j in n :
                        if(i % 2 == 0):
                                x_field_penuh = int(n[i])
                                y_field_penuh = int(n[i + 1])
                        
                                string = str(x_field_penuh) + " " + str(y_field_penuh) 
                        
                                # if(i == 0):
                                #     # text on topmost co-ordinate.
                                #     cv2.putText(aduduimg2, "=", (x, y),
                                #                     adudufont, 0.5, (255, 0, 0)) 
                                # if(i != 0):
                                        
                                #         cv2.putText(frame, string, (x_field_penuh, y_field_penuh), 
                                #                 adudufont, 0.5, (0, 255, 0))
                                asd_field.append(x_field_penuh)
                                ads_field.append(y_field_penuh)
            
                        i = i + 1
        asd_magenta = []
        ads_magenta = []
        for cnt in contours_magenta :
                
                approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
                
                cv2.drawContours(frame, [approx], 0, (255,0,255), 3) 
                
                n = approx.ravel() 
                i = 0
                
                
                for j in n :
                        if(i % 2 == 0):
                                x_magenta_penuh = int(n[i])
                                y_magenta_penuh = int(n[i + 1])
                        
                                string = str(x_magenta_penuh) + " " + str(y_magenta_penuh) 
                        
                                asd_magenta.append(x_magenta_penuh)
                                ads_magenta.append(y_magenta_penuh)
            
                        i = i + 1

        if ((len(asd)>0) and (len(ads)>0)):
                tengah_bola_x = min(asd) + ((max(asd) - min(asd)) // 2)
                tengah_bola_y = min(ads) + ((max(ads) - min(ads)) // 2)
                if((len(asd_field)>0) and (len(ads_field)>0) and (len(asd)>0) and (len(ads)>0)):
                        if (min(asd_field) < tengah_bola_x < max(asd_field)  and  min(ads_field) < tengah_bola_y < max(ads_field)):
                                #  Display_Kotak(3,2)
                                cv2.line(frame,(90,0), (90,480),(255,255,255),2) 
                                cv2.line(frame,(330,0), (330,480),(255,255,252),2) 
                                cv2.line(frame,(0,240), (640,240),(255,255,252),2) 
                                if(tengah_bola_x>0 and tengah_bola_x<180 and tengah_bola_y<360):
                                      dataFrame = int(1)
                                      kirim_ke_opencm_2023(dataFrame)
                                elif(tengah_bola_x>180 and tengah_bola_x<240 and tengah_bola_y<360):
                                      dataFrame = int(2)
                                      kirim_ke_opencm_2023(dataFrame)
                                elif(tengah_bola_x>240 and tengah_bola_y<360):
                                      dataFrame = int(3)
                                      kirim_ke_opencm_2023(dataFrame)
                                elif(tengah_bola_x>0 and tengah_bola_x<180 and tengah_bola_y>360):
                                      dataFrame = int(4)
                                      kirim_ke_opencm_2023(dataFrame)
                                elif(tengah_bola_x>180 and tengah_bola_x<240 and tengah_bola_y>360):
                                      dataFrame = int(5)
                                      kirim_ke_opencm_2023(dataFrame)
                                elif(tengah_bola_x>240 and tengah_bola_y>360):
                                      dataFrame = int(6)
                                      kirim_ke_opencm_2023(dataFrame)
                                                                            
                        else:
                                kirim_ke_opencm_2023(0)
                              

                elif((len(asd_field)==0) and (len(ads_field)==0) and (len(asd)>0) and (len(ads)>0)):
                        kirim_ke_opencm_2023(0)
              
        elif((len(asd_field)==0) and (len(ads_field)==0) and (len(asd)>0) and (len(ads)>0)):
                kirim_ke_opencm_2023(0)
        elif((len(asd_field)>0) and (len(ads_field)>0) and (len(asd)==0) and (len(ads)==0)):
                kirim_ke_opencm_2023(0)
        elif((len(asd_field)==0) and (len(ads_field)==0) and (len(asd)==0) and (len(ads)==0)):
                kirim_ke_opencm_2023(0)  


        if ((len(asd_magenta)>0) and (len(ads_magenta)>0)):
                tengah_magenta_x = min(asd_magenta) + ((max(asd_magenta) - min(asd_magenta)) // 2)
                tengah_magenta_y = min(ads_magenta) + ((max(ads_magenta) - min(ads_magenta)) // 2)
                Display_Kotak_cyan(3,1)

        elif ((len(asd_magenta)==0) and (len(ads_magenta)==0)):
              kirim_ke_opencm_cyan(0)
                            
                        
        cv2.imshow('tepi bola', frame) 
      
def sudut_bola_penuh(x_bola_penuh,y_bola_penuh):
      kotakX = 20
      kotakY = 24
      xx = 640 // kotakX 
      yy = 480 // kotakY 
      nomorKotak = np.arange(1,(kotakX*kotakY)+1) 
      apa = 0
      for eroryMin in range(kotakY): 
                for erorxMin in range(kotakX): 
                        if ((x_bola_penuh  > (xx*erorxMin) and x_bola_penuh < (xx*(erorxMin+1))) and (y_bola_penuh > (yy*eroryMin) and y_bola_penuh < (yy*(eroryMin+1)))):
                                dataFrame = nomorKotak[apa] 
                                # kirim_ke_opencm_2023(dataFrame)
                        elif (x_bola_penuh  == 0 and y_bola_penuh == 0):
                                dataFrame = 0
                                # kirim_ke_opencm_2023(dataFrame)
                        apa = apa + 1
#       print(f"{x_bola_penuh} | {y_bola_penuh}")
      

def Display_Kotak_cyan(kotakX, kotakY):
      global erorx,erory,tengah_magenta_x,tengah_magenta_y,frame,nilaiKotak
      xYangAkanDiIsi = tengah_magenta_x
      yYangAkanDiIsi = tengah_magenta_y
      xx = 640 // kotakX 
      yy = 480 // kotakY 
      
      for  xxx in range(1,kotakX):
            cv2.line(frame,((xx * xxx),0), ((xx * xxx),480),(255,0,255),2) 
      for yyy in range(1,kotakY):
            cv2.line(frame,(0,(yy * yyy)), (640,(yy * yyy)),(255,0,255),2)

      nomorKotak = np.arange(1,(kotakX*kotakY)+1) 
      apa = 0

      for eroryMin in range(kotakY): 
        for erorxMin in range(kotakX): 
                if ((xYangAkanDiIsi > (xx*erorxMin) and xYangAkanDiIsi< (xx*(erorxMin+1))) and (yYangAkanDiIsi> (yy*eroryMin) and yYangAkanDiIsi< (yy*(eroryMin+1)))):
                        dataFrame = nomorKotak[apa] 
                        kirim_ke_opencm_cyan(dataFrame)
                elif (xYangAkanDiIsi == 0 and yYangAkanDiIsi== 0):
                        dataFrame = 0
                        kirim_ke_opencm_cyan(dataFrame)
                apa = apa + 1

def Display_Kotak(kotakX, kotakY):
      global erorx,erory,tengah_bola_x,tengah_bola_y,frame,nilaiKotak
      xYangAkanDiIsi = erorx[0]
      yYangAkanDiIsi = erory[0]
      xx = 640 // kotakX 
      yy = 480 // kotakY 
      
      for  xxx in range(1,kotakX):
             (frame,((xx * xxx),0), ((xx * xxx),480),(0,0,255),2) 
      for yyy in range(1,kotakY):
            cv2.line(frame,(0,(yy * yyy)), (640,(yy * yyy)),(0,0,255),2)

      nomorKotak = np.arange(1,(kotakX*kotakY)+1) 
      apa = 0
      for eroryMin in range(kotakY): 
                for erorxMin in range(kotakX): 
                        if ((xYangAkanDiIsi  > (xx*erorxMin) and xYangAkanDiIsi < (xx*(erorxMin+1))) and (yYangAkanDiIsi > (yy*eroryMin) and yYangAkanDiIsi < (yy*(eroryMin+1)))):
                                dataFrame = nomorKotak[apa] 
                                kirim_ke_opencm_2023(dataFrame)
                        elif (xYangAkanDiIsi  == 0 and yYangAkanDiIsi == 0):
                                dataFrame = 0
                                kirim_ke_opencm_2023(dataFrame)
                        apa = apa + 1

                        


def kirim_ke_opencm_2023(dataFrame):
      print(f"dataFrame : {dataFrame}")
      for pin_bolaa in pin_bola:
            if dataFrame % 2 == 1:
                  GPIO.output(pin_bolaa,True)
                  #print(f"{pin_bolaa} : true")
            else:
                  GPIO.output(pin_bolaa,False)
                  #print(f"{pin_bolaa} : false  | ")
            dataFrame = dataFrame//2

def kirim_ke_opencm_cyan(dataFrame):
      #print(f"cyan : {dataFrame}")
      for pin_cyann in pin_cyan:
            if dataFrame % 2 == 1:
                  GPIO.output(pin_cyann,True)
                  #print(f"{pin_cyann} : true")
            else:
                  GPIO.output(pin_cyann,False)
                  #print(f"{pin_cyann} : false  | ")
            dataFrame = dataFrame//2


def SSE(Buffer,rata2):
     Deviasi={}
     Deviasi2={}
     rata2=0
     SSE=0
     for i in range(5):
             Deviasi[i] = (Buffer[i]-rata2)/10
             Deviasi2[i]=pow(Deviasi[i],2)
             SSE=SSE+Deviasi2[i]
     return SSE

def StandarDeviasi(SSE):
        return math.sqrt(SSE/(10-1))

################################################################################
##Main Program  
loadConfig()    
while True:
    progHelp()
    img = camera.read()
    img = imutils.resize(img, width=640)
    cv2.imshow('image',img)
    f=cv2.waitKey(0)& 0xFF
    if f==ord('q'):
        cv2.destroyAllWindows()
        VideoStream(src=cameranum).stop()
        break
    elif f==ord('='):
        cv2.destroyAllWindows()
        settThreshold()
    elif f==ord('m'):
        cv2.destroyAllWindows()
        TrackTanding()
fps.stop()
VideoStream(src=cameranum).stop()
cv2.destroyAllWindows()
