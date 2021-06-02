import cv2
import dlib


import telnetlib

class MBPipeline:
    def __init__(self, host="127.0.0.1", port="4242"):
        self.tn = telnetlib.Telnet(host, port)
        self.tn.read_until(b'>>> ')

    def call(self, command):
        self.tn.write(command + b'\n')
        r = self.tn.read_until(b'>>> ')[:-6]
        # try:
        #     return eval(r)
        # except:
        #     return str(r)
mbp = MBPipeline()

mbp.call("from pyfbsdk import *\n".encode("ascii"))

mbp.call("def des():\n    codefile = open(r'C:\PythonFolder\deselect.py')\n    import imp\n    module = imp.new_module('child')\n    exec codefile in module.__dict__\n    del(codefile,imp)".encode("ascii"))
mbp.call("des()\n".encode("ascii"))
mbp.call("def make(val):\n    c = FBModelCube('cube_'+str(val))\n    c.Show = True\n".encode("ascii"))
mbp.call("for i in range(68):\n    make(i)".encode("ascii"))

# mbp.call("c = FBModelCube('aaaaaa')\n".encode("ascii"))
# mbp.call("c.Show = True\n".encode("ascii"))

# Load the detector
detector = dlib.get_frontal_face_detector()

# Load the predictor
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# read the image
cap = cv2.VideoCapture(0)

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

        # Loop through all the points
        # x = landmarks.part(33).x
        # y = landmarks.part(33).y
        # message = "c.Translation = FBVector3d("+str(-x)+","+str(-y)+", 10)\n"
        # mbp.call(message.encode("ascii"))

        

        for n in range(0, 68):
            x = landmarks.part(n).x
            y = landmarks.part(n).y
            m1 = "mymodel = FBFindModelByLabelName('cube_"+str(n)+"')\n"
            print("m1",m1)
            mbp.call(m1.encode("ascii"))
            mbp.call("mymodel.Selected = True\n".encode("ascii"))
            message = "mymodel.Translation = FBVector3d("+str(x)+","+str(-y)+", 0)\n"
            mbp.call(message.encode("ascii"))
            mbp.call("mymodel.Selected = False\n".encode("ascii"))
            # mbp.call("des()\n".encode("ascii"))

            # Draw a circle
            # cv2.circle(img=frame, center=(x, y), radius=3, color=(0, 255, 0), thickness=-1)

    # show the image
    cv2.imshow(winname="Face", mat=frame)

    # Exit when escape is pressed
    if cv2.waitKey(delay=1) == 27:
        break

# When everything done, release the video capture and video write objects
cap.release()

# Close all windows
cv2.destroyAllWindows()