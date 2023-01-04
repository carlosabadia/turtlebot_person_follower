#!/usr/bin/env python3
from faceNet.model_utils import add_image_to_dataset, run_model
import gradio as gr
import cv2

# Create a list of strings to store the names of the people in the dataset to track
names = []

# Function to update the names list and add the face image to the dataset
def update_names(face_image, name) -> tuple:
    """
    Update the names list with a unique name and add the face image to the dataset.

    Parameters:
    - face_image (np.array): The image of the face to add to the dataset.
    - name (str): The name to label the image with.

    Returns:
    - tuple: A tuple containing the updated Dropdown widget and Image widget.
    """

    # If no name is given, generate a unique name in the form "Person{i}"
    if name == "":
        i = 0
        while f"Person{i}" in names:
            i += 1
        name = f"Person{i}"
    # If a name is given, generate a unique name in the form "{name}{i}" if the name already exists in the list
    else:
        if name in names:
            i = 0
            while f"{name}{i}" in names:
                i += 1
            name = f"{name}{i}"
    names.append(name)

    # Convert the image to a cv2 image and add it to the dataset
    face_detected = add_image_to_dataset(
        cv2.cvtColor(face_image, cv2.COLOR_RGB2BGR), name)

    # Return the updated Dropdown widget and Image widget
    return gr.Dropdown.update(choices=names, value=name), gr.Image.update(value=face_detected)

# Function to run the model with the given name
def track_face(person_name: str = "Person") -> None:
    """
    Track a face in the given image using a model with the given name.

    Parameters:
    - person_name (str): The name of the person to use as the key for the model in the models dictionary.
    """

    # Run the model with the given name
    run_model(person_name)

# Function to run the app
def interface() -> None:
    """
    Create and launch the graphical user interface for the Turtlebot person follower app.
    """

    # Create the blocks for the interface
    with gr.Blocks() as demo:
        # Add a title and opening HTML element
        gr.HTML(
            """
            <div style="text-align: center; max-width: 650px; margin: 0 auto; padding-top: 7px;">
              <div
                style="
                  display: inline-flex;
                  align-items: center;
                  gap: 0.8rem;
                  font-size: 1.85rem;
                "
              >
                <h1 style="font-weight: 900; margin-bottom: 7px;">
                  TurtleBot Person Follower 🤖
                </h1>
              </div>
            </div>
        """
        )
        # Add a tabbed interface
        with gr.Tabs():
            # Add the "Face Recognition" tab
            with gr.TabItem("Face Recognition 👤"):
                # Add a group for the face recognition input and output widgets
                with gr.Group():
                    with gr.Row():
                        with gr.Column():
                            # Add a textbox and button for saving a new person's face and name
                            with gr.Row():
                                new_person_name = gr.Textbox(label="Person Name", show_label=False, max_lines=1,
                                                             placeholder="Enter person name after taking a picture of his/her face", interactive=True)
                                new_person_button = gr.Button(value="Save person's face and name").style(
                                    rounded=(False, True, True, False))
                            # Add a webcam and image widget for displaying the input and detected face
                            with gr.Row():
                                webcam_image_in = gr.Webcam(
                                    label="Webcam input")
                                face_detected_image_in = gr.Image(
                                    label="Face detected", interactive=False)

                            with gr.Row():
                            # Add a text widget for displaying the detected face's name
                                gr.Text(
									label="⚠️ Reminder ", value="Do not forget to click the camera button to freeze and get the webcam image 📷!", interactive=False)

                        with gr.Column():
                            # Add a dropdown widget for selecting the person to track
                            with gr.Row():
                                person_name_dropdown = gr.Dropdown(label="Select the person you wanna track 🔎", choices=[
                                                                   name for name in names], interactive=True)
                            # Add a button for initializing the Turtlebot and starting the face tracking
                            track_button = gr.Button(
                                value="🚀 Init Face Recognition and Start Tracking!")

                        # Bind the track_face function to the track button's click event
                        track_button.click(fn=track_face, inputs=[
                                           person_name_dropdown], outputs=None)

                        # Bind the update_names function to the new_person_button's click event
                        new_person_button.click(fn=update_names, inputs=[webcam_image_in, new_person_name], outputs=[
                                                person_name_dropdown, face_detected_image_in])
    # Launch the interface
    demo.launch(share=False)

if __name__ == '__main__':
    interface() # Run the interface
