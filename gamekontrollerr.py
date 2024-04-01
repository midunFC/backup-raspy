import socket
import RPi.GPIO as GPIO 
from time import sleep 
GPIO.setwarnings(False) 
GPIO.setmode(

GPIO.BCM)
pinSockets=13
pinSocketsReady=19
GPIO.setup(pinSockets,GPIO.OUT)
GPIO.setup(pinSocketsReady,GPIO.OUT)



client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

#client.bind(('192.168.66.230', 3838))
client.bind(('', 3838))

print("test...")
while True:
    data, address = client.recvfrom(1024)
    wrx = data.strip()
    #print (wrx)
    #print(wrx[0],wrx[1],wrx[2],wrx[3],wrx[4],wrx[5],wrx[6],wrx[7],wrx[8],wrx[9],wrx[10],wrx[11],wrx[12],wrx[13],wrx[14],wrx[15],wrx[16],wrx[17],wrx[18],wrx[19],wrx[20],wrx[21],wrx[22],wrx[23],wrx[24],wrx[25],wrx[26],wrx[27],wrx[28],wrx[29],wrx[30],wrx[31],wrx[32],wrx[33],wrx[34],wrx[35],wrx[36],wrx[37],wrx[38],wrx[39],wrx[40],wrx[41],wrx[42],wrx[43],wrx[44],wrx[45],wrx[46],wrx[47],wrx[48],wrx[49],wrx[50],wrx[51],wrx[52],wrx[53],wrx[54],wrx[55],wrx[56],wrx[57],wrx[58],wrx[59],wrx[60],wrx[61],wrx[62],wrx[63],wrx[64],wrx[65],wrx[66],wrx[67],wrx[68],wrx[69],wrx[70],wrx[71],wrx[72],wrx[73],wrx[74],wrx[75],wrx[76],wrx[77],wrx[78],wrx[79])
    #print(len(wrx))
    if (len(wrx)==158):
        #print(wrx[0],wrx[1],wrx[2],wrx[3],wrx[4],wrx[5],wrx[6],wrx[7],wrx[8],wrx[9],wrx[10],wrx[11],wrx[12],wrx[13],wrx[14],wrx[15],wrx[16],wrx[17],wrx[18],wrx[19],wrx[20],wrx[21],wrx[22],wrx[23],wrx[24],wrx[25],wrx[26],wrx[27],wrx[28],wrx[29],wrx[30],wrx[31],wrx[32],wrx[33],wrx[34],wrx[35],wrx[36],wrx[37],wrx[38],wrx[39],wrx[40],wrx[41],wrx[42],wrx[43],wrx[44],wrx[45],wrx[46],wrx[47],wrx[48],wrx[49],wrx[50],wrx[51],wrx[52],wrx[53],wrx[54],wrx[55],wrx[56],wrx[57],wrx[58],wrx[59],wrx[60],wrx[61],wrx[62],wrx[63],wrx[64],wrx[65],wrx[66],wrx[67],wrx[68],wrx[69],wrx[70],wrx[71],wrx[72],wrx[73],wrx[74],wrx[75],wrx[76],wrx[77],wrx[78],wrx[79],print(wrx[0],wrx[1],wrx[2],wrx[3],wrx[4],wrx[5],wrx[6],wrx[7],wrx[8],wrx[9],wrx[10],wrx[11],wrx[12],wrx[13],wrx[14],wrx[15],wrx[16],wrx[17],wrx[18],wrx[19],wrx[20],wrx[21],wrx[22],wrx[23],wrx[24],wrx[25],wrx[26],wrx[27],wrx[28],wrx[29],wrx[30],wrx[31],wrx[32],wrx[33],wrx[34],wrx[35],wrx[36],wrx[37],wrx[38],wrx[39],wrx[40],wrx[41],wrx[42],wrx[43],wrx[44],wrx[45],wrx[46],wrx[47],wrx[48],wrx[49],wrx[50],wrx[51],wrx[52],wrx[53],wrx[54],wrx[55],wrx[56],wrx[57],wrx[58],wrx[59],wrx[60],wrx[61],wrx[62],wrx[63],wrx[64],wrx[65],wrx[66],wrx[67],wrx[68],wrx[69],wrx[80],wrx[81],wrx[82],wrx[83],wrx[84],wrx[85],wrx[86],wrx[87],wrx[88],wrx[89]))
        #for i in range (0,len(wrx)):
        #    if (wrx[i]==1):
        #        print(i)
        '''
        if (wrx[6]==6 and wrx[7]==0):
            print('initialize')
        elif (wrx[6]==6 and wrx[7]==1):
            print('ready')
        elif (wrx[6]==6 and wrx[7]==2):
            print('set')
        elif (wrx[6]==6 and wrx[7]==3):
            print('play kiri = ', wrx[20], 'kanan = ' , wrx[90])
      
        elif (wrx[6]==6 and wrx[7]==4):
            print('finish')
        
        '''
        if (wrx[6]==6 and wrx[7]==0):
            print('initialize')
            GPIO.output(pinSockets,False)
            GPIO.output(pinSocketsReady,False)
            
        elif (wrx[6]==6 and wrx[7]==1):
            print('ready')
            GPIO.output(pinSockets,True)
            GPIO.output(pinSocketsReady,False)
            
        elif (wrx[6]==6 and wrx[7]==3):
            print('play kiri = ', wrx[20], 'kanan = ' , wrx[90])
            GPIO.output(pinSockets,False)
            GPIO.output(pinSocketsReady,True)
            
        elif (wrx[6]==6 and wrx[7]==2):
            print('set')
            
            GPIO.output(pinSockets,False)
            GPIO.output(pinSocketsReady,False)
            
        elif (wrx[6]==6 and wrx[7]==4):
            print('finish')
            GPIO.output(pinSockets,False)
            GPIO.output(pinSocketsReady,False)
            
#         elif (wrx[6]==6 and wrx[7]==3):
#             print('play kiri = ', wrx[20], 'kanan = ' , wrx[90])
#             GPIO.output(pinSockets,1)
#         else:
#             GPIO.output(pinSockets,0)
#             print('stop')
        
