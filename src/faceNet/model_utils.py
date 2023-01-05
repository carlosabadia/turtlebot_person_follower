#!/usr/bin/env python3
from faceNet.utils import FPSmetric
from faceNet.engine import Engine
from faceNet.faceDetection import MPFaceDetection
from faceNet.faceNet import FaceNet
import os
import numpy as np
import roslib

DIR = roslib.packages.get_pkg_dir("person_follower", required=True) # Get the path of the package

models = {} # Dictionary to store the models

# Function to load a model
def load_model(name_path: str = "Person") -> None:
    """
    Load a model with the specified name.
    
    Parameters:
        name_path (str): The name of the model to load. Default is "Person".
        
    Returns:
        None
    """
    # Initialize the FaceNet model with the specified parameters
    facenet = FaceNet(
        detector=MPFaceDetection(),
        onnx_model_path=os.path.join(DIR, "src/models/faceNet.onnx"),
        anchors=os.path.join(DIR, "src/faces/", name_path),
		person_name=name_path,
        force_cpu=True,
    )

    # Check if the model was successfully initialized
    if facenet is None:
        raise Exception("Model is not loaded")

    # Print a message indicating that the model was successfully loaded
    print(f"Model loaded: {name_path}")

    # Add the model to the models dictionary
    models[name_path] = facenet

    # Print the keys of the models dictionary
    print(f"Models loaded: {list(models.keys())}")

# Function to run a model
def run_model(name: str = "Person") -> None:
    """
    Run a model with the given name on the main thread.
    
    Parameters:
    - name (str): The name of the person to use as the key for the model in the models dictionary.
    """
    
    # Set the main thread's run function to the run_model_mainthread function with the given name
    # If a model with the specified name does not exist, create a new model
    if name not in models.keys():
        load_model(name)
        
    # Initialize the engine with the specified parameters
    engine = Engine(webcam_id=0, show=True, turtlebot_video_path="/camera/rgb/image_raw/compressed", custom_objects=[models[name], FPSmetric()])
    
    # Run the engine
    engine.run()


# Function to add an image to the dataset
def add_image_to_dataset(img: np.array, name: str = "Person") -> str:
    """
    Add a new image to the dataset of images labeled with the given name.
    
    Parameters:
    - img (np.array): The image to add to the dataset.
    - name (str, optional): The name to label the image with. Default is "Person".
    
    Returns:
    - str: The path to the saved image.
    """
    
    # Create a directory for the faces if it does not exist
    faces_dir = os.path.join(DIR, "src/faces/")
    if not os.path.exists(faces_dir):
        os.makedirs(faces_dir)
        print("Created faces directory")
        
    # Create a subdirectory within the faces directory with the given name if it does not exist
    name_dir = os.path.join(faces_dir, name)
    if not os.path.exists(name_dir):
        os.makedirs(name_dir)
        print(f"Created {name} directory")
        
    # Find the next available number to use as the file name for the new image
    i = 0
    while os.path.exists(os.path.join(name_dir, f"{i}.png")):
        i += 1
        
    # If a model with the key of the given name does not exist, create a new model
    if name not in models.keys():
        load_model(name)
        
    # Detect and save the faces in the image using the model with the key of the given name
    models[name].detect_save_faces(img, output_dir=name_dir, name=name)
    
    # Return the path to the saved image to display in the gradio app
    return os.path.join(name_dir, f"{name}_{i}.png")
