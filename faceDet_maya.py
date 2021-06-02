#TURN ON IRIUM WEBCAME AND ANACONDA tf2

import socket
import random 
from unicodedata import *
import cv2
import dlib
import telnetlib
import time
import threading


HOST = '192.168.0.201' # The remote host
PORT = 1235 # The same port as used by the server
ADDR=(HOST,PORT)
def rf(pos):
    # x,y,z = pos[0],pos[1],pos[2]
    for dim in pos:
        dim = str(dim+random.uniform(-3,3))
    st = pos[0]+","+pos[1]+","+pos[2]
    return st

print("xx")
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

def start():
    # START CUBES
    cm = "import maya.cmds as cmds;  import cubez1; cubez1.makecubez(68); dad = cmds.select('dad'); cubz = cmds.select('cube_*'); sel = cmds.ls(sl=True); cmds.select( clear=True ); selectedMeshes = [x for x in sel if cmds.objectType(x) == 'transform']; print(selectedMeshes);[cmds.parent(i,'dad') for i in selectedMeshes];"    
    client.send(bytes(cm, 'utf-8'))
    data = client.recv(1024) #receive the result info
    print("Data",data)

def SendCommand(lz):
    print("update cubes")
    # lz = [[4,4],[10,20],[30,40]]
    d = ["["+str(i[0])+","+str(i[1])+"]" for i in lz]
    z = "["+", ".join(d)+"]"
    # print("zzz",z)
    cm = "import maya.cmds as cmds;  import aniface; aniface.main("+z+");"
    client.send(bytes(cm, 'utf-8'))
    data = client.recv(1024) #receive the result info
    print("The Result is",data)

    # saveKeyAdvance = "x = cmds.select('dad'); c = cmds.listRelatives(ad=True); cmds.setKeyframe(c); time = cmds.currentTime( query=True ); cmds.currentTime( time+1, edit=True )"
    # client.send(bytes(saveKeyAdvance, 'utf-8'))
    # data = client.recv(1024) #receive the result info
    # print("The Result is",data)

# CREATE CUBES
start()
lastTime = time.time()

# Load the detector
detector = dlib.get_frontal_face_detector()

# Load the predictor
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

cap = cv2.VideoCapture("Week8_Housekeeping.mov")

# read the image
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_POS_FRAMES, 10)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

# TO KEYFRAME AND ADVANCE
# val saveKeyframe = "import maya.cmds as cmds; x = cmds.select('spine'); c = cmds.listRelatives(ad=True); cmds.setKeyframe(c);time = cmds.currentTime( query=True ); print(time); cmds.currentTime( time+1, edit=True );"



# x = threading.Thread(target=thread_function, args=(1,))
# x.start()

# if (time.time()-lastTime)>.08:
#     time.sleep(.03)
# else:
#     lastTime = time.time()
#     print(lastTime)

while True:
    _, frame = cap.read()
    # Convert image into grayscale
    gray = cv2.cvtColor(src=frame, code=cv2.COLOR_BGR2GRAY)

    # Use detector to find landmarks
    faces = detector(gray)

    for face in faces:
        x1 = face.left()  # left point
        y1 = face.top()  # top point
        x2 = face.right()  # right point
        y2 = face.bottom()  # bottom point

        # Create landmark object
        landmarks = predictor(image=gray, box=face)
        list_of_face_points = []
        for n in range(0, 68):
            x = landmarks.part(n).x
            y = landmarks.part(n).y
            list_of_face_points.append([x,y])

            # Draw a circle
            cv2.circle(img=frame, center=(x, y), radius=3, color=(0, 255, 0), thickness=-1)
        
        # print(list_of_face_points)
        # x = threading.Thread(target=thread_function, args=(1,))
        # x.start()
        SendCommand(list_of_face_points)

    # show the image
    cv2.imshow(winname="Face", mat=frame)

    # Exit when escape is pressed
    if cv2.waitKey(delay=1) == 27:
        break

# When everything done, release the video capture and video write objects
cap.release()

# Close all windows
cv2.destroyAllWindows()

client.close()




'''
#FROM MAYA BACKUP
import maya.cmds as cmds
import maya.mel as mel

def clearKeyframes():
	x = cmds.select('dad')
	c = cmds.listRelatives(ad=True)
	cmds.cutKey(c, s=True)#delete key command
	time = 0
	print(time)
	cmds.currentTime( time+1, edit=True )

with open ('C:/Users/TENZE-SUPER/Documents/ADozenLonely/Scripts/PyWebcamToMobu/outfile', 'rb') as fp:
	pointHistory = pickle.load(fp)
def playAnimationFromFile(count):
	print("playAnimationFile")
	x = cmds.select('dad')
	sel = cmds.listRelatives(ad=True)
	cmds.select( clear=True )
	c = [x for x in sel if cmds.objectType(x) == 'transform']
	for i,element in enumerate(c):
		qq = 0
		#time = cmds.currentTime( query=True )
		#print(time)
		cmds.currentTime(pointHistory[count][0], edit=True )
		#print(pointHistory[count][1][i])
		print("lennn point hist ",len(pointHistory[count][1]))
		print("lennn c ",len(c))
		pz = [pointHistory[count][1][i][0],pointHistory[count][1][i][1],0]
		cmds.xform(element, t=pz)
		cmds.setKeyframe(element)

clearKeyframes()
#change to do whole outfile
for i in range(30):
	print(i)
	playAnimationFromFile(i)
'''
