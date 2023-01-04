#!/usr/bin/env python3
from faceNet.utils import FPSmetric
from faceNet.engine import Engine
from faceNet.faceDetection import MPFaceDetection
from faceNet.faceNet import FaceNet
import roslib

DIR = roslib.packages.get_pkg_dir("person_follower", required=True)
person_face_path = DIR + "/src/faces/Elon/" # Change this path to your folder path containing the cropped image of the person

if __name__ == '__main__':
    facenet = FaceNet(
        detector = MPFaceDetection(),
        onnx_model_path = DIR + "/src/models/faceNet.onnx", 
        anchors = person_face_path,
        force_cpu = True,
        person_name="elon.png", # Change this to the name of the cropped image of the person
    )

    engine = Engine(turtlebot_video_path="/camera/rgb/image_raw", show=True, custom_objects=[facenet, FPSmetric()])

    engine.run()