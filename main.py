import cv2
import numpy as np
import serial
import time

# Serial connection
serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

cam_index = str(input("Kamera indeksi giriniz (/dev/video0 || 0/1): "))
last_xt, last_yt, last_wt, last_ht = 0,0,0,0
xt,yt,wt,ht=0,0,0,0
#cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
def readVideo(cam_index):
    pipeline = " ! ".join(["v4l2src device=/dev/video0",
                       "video/x-raw, width=640, height=480, framerate=30/1",
                       "videoconvert",
                       "video/x-raw, format=(string)BGR",
                       "appsink"
                       ])
    h264_pipeline = " ! ".join(["v4l2src device=/dev/video0",
								"video/x-h264, width=1280, height=720, framerate=30/1, format=H264",
								"avdec_h264",
								"videoconvert",
								"video/x-raw, format=(string)BGR",
								"appsink sync=false"
								])
    #cap_yayagecidi = cv2.VideoCapture(2)                            
    #cap_yol = cv2.VideoCapture(1) # 1 --> yol  2 --> yaya geçidi
    # **MAIN** cap_yol = cv2.VideoCapture(0) # 1 --> yol  2 --> yaya geçidi
    cap_yol = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    #return cap_yol, cap_yayagecidi
    return cap_yol
def changeColorSpace(inputImage):
	gray = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	return gray, hsv, blurred
def warpImage(frame):
    pts1=[50,240]
    pts2=[400,240]
    pts3=[300,480]
    pts4=[0,480]
    
    #point_matrix = np.float32([pts1,pts2,pts3,pts4])
    point_matrix = np.float32([pts1,pts2,pts3,pts4])
    width,height = 300,300
    
    converted_pts1=[0,0]
    converted_pts2=[320,0]
    converted_pts3=[500,500]
    converted_pts4=[0,500]
    
    converted_points=np.float32([converted_pts1,converted_pts2,converted_pts3,converted_pts4])
    perspective_transform=cv2.getPerspectiveTransform(point_matrix,converted_points)
    img_Output=cv2.warpPerspective(frame, perspective_transform,(width,height))
    outputx=img_Output.shape[1]/2
    outputy=img_Output.shape[0]/2
    return img_Output, outputx, outputy
	
def findContours(thresh):
	_,thresh = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)
	contours,b = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	return thresh, contours
def mainProcess(img_Output, contours, outputx, outputy, frame_yol):
    if(len(contours)>0):
        c=max(contours,key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        print(x,y,w,h)
        """for cnt in contours:
            last_xt, last_yt, last_wt, last_ht = xt,yt,wt,ht
            xt,yt,wt,ht = cv2.boundingRect(cnt)
            if(xt<last_xt):
                xt=last_xt
            if(yt<last_yt):
                yt=last_yt
            if(wt<last_wt):
                wt=last_wt
            if(ht<last_ht):
                ht=last_ht"""
        #print("C : ",c)
        print("T VALUES: ",xt,yt,wt,ht)
        M=cv2.moments(c)
        if(M["m00"]!=0):
            cx=int(M['m10']/M['m00'])
            cy=int(M['m01']/M['m00'])
        else:
            cx,cy = 0,0
        a = (img_Output.shape[1]/2)-cx
        cv2.drawContours(img_Output, contours, -1, (0,0,255),1)
        cv2.circle(img_Output, (cx,img_Output.shape[0]-5),2,(255,0,0),6)
        #print("X Center of img_Output: ",a)
        
        
        # -- OTONOM SÜRÜŞ --
        if(cx<outputx-40):
            print("Sola dönünüz")
            b=0
        elif(cx>outputx+50):
            print("Sağa dönünüz")
            b=0
        elif(cx<outputx+50 and cx>outputx-40):
            print("Yoldasınız")
            b=0
        # -- YAYA / HEMZEMİN GEÇİDİ
        if(x<5 and x+w>img_Output.shape[1]-10):
            print("YAYA GECIDI / HEMZEMIN GECIDI\n DURUNUZ!")
            a = "x"
            #time.sleep(5)
        serial_port.write((str(a)+"\n").encode())
        time.sleep(0.010)
        
        
        # -- CAR DETECTION --
        hsvOfYol = cv2.cvtColor(frame_yol, cv2.COLOR_BGR2HSV)
        orangeMask = cv2.inRange(hsvOfYol, (10,100,20), (25, 255, 255))
        orangeContours,s = cv2.findContours(orangeMask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if(len(orangeContours)>0):
            for cnts in orangeContours:
                orangeC = max(orangeContours, key = cv2.contourArea)
                orangeX,orangeY,orangeW,orangeH = cv2.boundingRect(orangeC)
                print("W size of Orange: ",orangeW)
                if(orangeW>75 and orangeH>75):
                    frame_yol = cv2.rectangle(frame_yol, (orangeX,orangeY),(orangeX+orangeW,orangeY+orangeH),(0,0,255,),2)
                    print("Orange Light detected")
                    a="y"
                    
                    break
                else:
                    print("Orange Light is not detected")
                    break
    else:
        print("Şerit Kaybedildi.")
        b=0



#cap_yol, cap_yayagecidi = readVideo(cam_index)
#cap_yol = readVideo(cam_index)
cap_yol = readVideo(2)
while True:
	ret, frame_yol = cap_yol.read()
	img_Output,outputx,outputy = warpImage(frame_yol)
	gray, hsv, blurred = changeColorSpace(img_Output)
	thresh, contours = findContours(gray)
	mainProcess(img_Output,contours,outputx,outputy, frame_yol)
	cv2.imshow("Output Image",img_Output)
	cv2.imshow("frame",frame_yol)
	if(cv2.waitKey(1) & 0xFF==ord("q")):
		break

cap_yol.release()
cv2.destroyAllWindows()
