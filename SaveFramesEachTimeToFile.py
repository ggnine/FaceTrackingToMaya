#TURN ON IRIUM WEBCAME AND ANACONDA tf2
# cd Documents\ADozenLonely\Scripts\PyWebcamToMobu
# conda activate tf2-gpu

# USAGE python SaveFramesEachTimeToFile.py
import cv2, queue, threading, time


import socket
import random 
from unicodedata import *
import cv2
import dlib
import telnetlib
import time
import threading
import pickle
import numpy as np
NUM_POINTS = 64 # 68/5
sendEveryXPoints = 1

pointHistory = []

ready= True

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



# bufferless VideoCapture
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture("Week8_Housekeeping.mov")

    # self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

def start():
    # START CUBES BAKCUP
    # cm = "import maya.cmds as cmds; import cubez1; cubez1.makecubez(68); dad = cmds.select('dad'); cubz = cmds.select('cube_*'); sel = cmds.ls(sl=True); cmds.select( clear=True ); selectedMeshes = [x for x in sel if cmds.objectType(x) == 'transform']; print(selectedMeshes);[cmds.parent(i,'dad') for i in selectedMeshes];"    

    cm = "import maya.cmds as cmds; [cmds.instance('plat',n='cube_'+str(i)) for i in range("+str(NUM_POINTS)+")]; dad = cmds.select('dad'); cubz = cmds.select('cube_*'); sel = cmds.ls(sl=True); cmds.select( clear=True ); selectedMeshes = [x for x in sel if cmds.objectType(x) == 'transform']; print(selectedMeshes);[cmds.parent(i,'dad') for i in selectedMeshes];"    
    client.send(bytes(cm, 'utf-8'))
    data = client.recv(1024) #receive the result info
    print("Data",data)

def SendCommand(lz):
    global ready
    print("update cubes")
    # lz = [[4,4],[10,20],[30,40]]
    d = ["["+str(i[0])+","+str(i[1])+"]" for i in lz]
    z = "["+", ".join(d)+"]"
    # print("zzz",z)
    cm = "import maya.cmds as cmds;  import aniface; aniface.main("+z+");"
    client.send(bytes(cm, 'utf-8'))
    data = client.recv(1024) #receive the result info
    # print("The Result is",data)
    
    # newTime = time.time() - startTime
    # newTime = str(int(newTime* 24))
    # print("newtime",newTime)

    # BACKUP
    # saveKeyAdvance = "x = cmds.select('dad'); c = cmds.listRelatives(ad=True); cmds.setKeyframe(c); time = cmds.currentTime( query=True ); cmds.currentTime( time+1, edit=True )"

    # SKIP FOR NOW
    # saveKeyAdvance = "x = cmds.select('dad'); c = cmds.listRelatives(ad=True); cmds.setKeyframe(c); cmds.currentTime("+str(newTime)+", edit=True )"
    # client.send(bytes(saveKeyAdvance, 'utf-8'))
    # data = client.recv(1024) #receive the result info

    return data

# CREATE CUBES
start()


# Load the detector
detector = dlib.get_frontal_face_detector()

# Load the predictor
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

cap = VideoCapture(0)
startTime = time.time()
count = 0
lastTime = 0
buffa = []
gray = None
while True:
	# faketest
	# if 1==1 or int((time.time() - lastTime)*24)>4:
		count += 1
		#   try:
		print("here",count,"isrady?",ready)
		
		frame = cap.read()
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
			for n in range(0, 68,sendEveryXPoints):
				x = landmarks.part(n).x
				y = landmarks.part(n).y
				list_of_face_points.append([x,y])

				# Draw a circle
				# cv2.circle(img=frame, center=(x, y), radius=3, color=(0, 255, 0), thickness=-1)
			

			nutime = int((time.time() - startTime)*24)
			print("nutime",time.time() - startTime) 
			print("nutime",nutime) 

			buffa.append(list_of_face_points)

			if len(buffa) == 2:
				# print(list_of_face_points)
				a = np.asarray(buffa[0])
				b = np.asarray(buffa[1])
				z = (a+b)/2
				pointHistory.append([nutime-1,z.tolist()])
				buffa = []


		# show the image
		# cv2.imshow(winname="Face", mat=frame)
		lastTime = time.time()
		# Exit when escape is pressed
		if count > 3480:
			break
		if cv2.waitKey(delay=1) == 27:
			break

print("lennn",len(pointHistory))

# f = open("points.txt", "x")
# f.write(pointHistory)
# f.close()

# WRITE
with open('outfile', 'wb') as fp:
    pickle.dump(pointHistory, fp,protocol=2)

# READ
# with open ('outfile', 'rb') as fp:
#     pointHistory = pickle.load(fp)

# cap.release()

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
