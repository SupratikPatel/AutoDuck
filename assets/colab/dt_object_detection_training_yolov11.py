#!/usr/bin/env python
# coding: utf-8

# # Object Detection Training with Ultralytics YOLO
#
# This script trains an object detection model using the Ultralytics YOLO framework.
# It is designed to be run in a Colab-like environment.

import os
import contextlib
import subprocess
import tempfile
import shutil

# Helper functions (from the original notebook)
@contextlib.contextmanager
def directory(name):
  ret = os.getcwd()
  os.chdir(name)
  yield None
  os.chdir(ret)

def run(command, exception_on_failure=False):
  try:
    program_output = subprocess.check_output(command, shell=True, universal_newlines=True, stderr=subprocess.STDOUT)
  except Exception as e:
    if exception_on_failure:
      raise e
    program_output = e.output
  return program_output

def prun(command, exception_on_failure=False):
  x = run(command, exception_on_failure)
  print(x)
  return x

def main():
    print("# Object Detection Training Script Started")

    # --- Setup Dependencies ---
    print("
# Setting up dependencies...")
    print("# Don't forget to switch to a GPU-enabled colab runtime if not already!")

    # --- Create a temporary workspace ---
    print("
# Creating a temporary workspace...")
    SESSION_WORKSPACE = tempfile.mkdtemp()
    print(f"Session workspace created at: {SESSION_WORKSPACE}")

    # --- Mount the drive (Colab specific) ---
    print("
# Mounting Google Drive (Colab specific)...")
    # This part is for Colab. If running locally, you might need to adjust dataset paths.
    try:
        from google.colab import drive
        drive.mount('/content/drive')
        DRIVE_PATH = "/content/drive/My Drive"
        print("Google Drive mounted.")
    except ImportError:
        print("Google Colab 'drive' not available. Assuming local execution or pre-mounted drive.")
        print("Please ensure your dataset is accessible at the specified DRIVE_PATH or modify script.")
        DRIVE_PATH = os.path.expanduser("~/GoogleDrive/My Drive") # Example for local
        if not os.path.exists(DRIVE_PATH):
             print(f"Warning: Default DRIVE_PATH {DRIVE_PATH} does not exist.")
             # Allow user to specify if not found, or use a default local path
             DRIVE_PATH = input(f"Please enter the path to your dataset zip file's directory (e.g., /path/to/drive/My Drive): ")


    # --- Unzip the dataset ---
    print("
# Unzipping the dataset...")
    DATASET_DIR_NAME = "duckietown_object_detection_dataset"
    DATASET_ZIP_NAME = f"{DATASET_DIR_NAME}.zip"
    DATASET_DIR_PATH = os.path.join(SESSION_WORKSPACE, DATASET_DIR_NAME)
    TRAIN_DIR = "train"
    VALIDATION_DIR = "val"
    IMAGES_DIR = "images"
    LABELS_DIR = "labels"

    def show_info(base_path: str):
      for l1 in [TRAIN_DIR, VALIDATION_DIR]:
        for l2 in [IMAGES_DIR, LABELS_DIR]:
          p = os.path.join(base_path, l1, l2)
          if os.path.exists(p):
            print(f"#Files in {l1}/{l2}: {len(os.listdir(p))}")
          else:
            print(f"#Path not found: {p}")

    def unzip_dataset():
      zip_path = os.path.join(DRIVE_PATH, DATASET_ZIP_NAME)
      if not os.path.exists(zip_path):
          print(f"No zipped dataset found at {zip_path}! Please check the path and filename.")
          alt_zip_path = input(f"Enter the full path to '{DATASET_ZIP_NAME}' if it's elsewhere, or press Enter to abort: ")
          if alt_zip_path and os.path.exists(alt_zip_path):
              zip_path = alt_zip_path
          else:
              print("Dataset zip file not found. Aborting.")
              return False

      print("Unpacking zipped data...")
      shutil.unpack_archive(zip_path, DATASET_DIR_PATH)
      print(f"Zipped dataset unpacked to {DATASET_DIR_PATH}")
      show_info(DATASET_DIR_PATH)
      return True

    if not unzip_dataset():
        return # Exit if dataset unzip failed

    # --- Change working directory to the session workspace ---
    print(f"
# Changing working directory to: {SESSION_WORKSPACE}")
    os.chdir(SESSION_WORKSPACE)
    print(f"PWD: {os.getcwd()}")

    # --- Install PyTorch and torchvision (if needed, ultralytics might handle this) ---
    print("
# Installing PyTorch and torchvision...")
    prun("pip3 install torch==1.13.0 torchvision==0.14.0") # As per original notebook

    # --- Install Ultralytics YOLO ---
    print("
# Installing Ultralytics YOLO...")
    prun("pip3 install ultralytics")

    # --- Create dataset YAML file ---
    print("
# Creating dataset YAML file (duckietown.yaml)...")
    yaml_content = f"""
# train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
train: {DATASET_DIR_NAME}/train # Path relative to this YAML file
val: {DATASET_DIR_NAME}/val   # Path relative to this YAML file

# number of classes
nc: 4

# class names
names: [ 'duckie', 'cone', 'truck', 'bus' ]
"""
    with open("duckietown.yaml", "w") as f:
        f.write(yaml_content)
    print("duckietown.yaml created.")

    # --- Train the model ---
    print("
# Training the model...")
    print("# This step will take about 5 minutes. Notice we're only training for 10 epochs.")

    from ultralytics import YOLO # Import here after installation

    BEST_MODEL_PATH = None
    try:
        model = YOLO('yolo11n.pt') # Or specific model like 'yolov8n.pt'
        results = model.train(
            data='duckietown.yaml',
            epochs=10,
            imgsz=416,
            batch=32,
            project='runs/train',
            name='exp',
        )
        BEST_MODEL_PATH = os.path.join(results.save_dir, 'weights/best.pt')
        print(f"Best model saved to: {BEST_MODEL_PATH}")
    except Exception as e:
        print(f"An error occurred during training: {e}")
        BEST_MODEL_PATH = None

    if BEST_MODEL_PATH and os.path.exists(BEST_MODEL_PATH):
        print(f"The best model path is: {BEST_MODEL_PATH}")
    else:
        print("Training did not complete successfully, or the best model path was not found.")

    # --- Upload model to Duckietown's cloud ---
    print("
# Preparing to upload model to Duckietown's cloud...")
    print("# We will need our token to access our personal cloud space.")

    YOUR_DT_TOKEN = os.getenv("YOUR_DT_TOKEN") # Try to get from env var
    if not YOUR_DT_TOKEN:
        YOUR_DT_TOKEN = input("Please enter your Duckietown Token (or set YOUR_DT_TOKEN env var): ")
    
    if not YOUR_DT_TOKEN or YOUR_DT_TOKEN == "YOUR_TOKEN_HERE":
        print("Duckietown token not provided. Skipping model upload.")
    else:
        print("
# Configuring model for upload...")
        model_name = "yolo11n" # Or your chosen model variant name
        model_local_path = BEST_MODEL_PATH
        model_remote_path = f"courses/mooc/objdet/data/nn_models/{model_name}.pt"

        print("# Installing DCSS client...")
        prun("pip3 install dt-data-api")

        print("
# Uploading model...")
        if model_local_path and os.path.exists(model_local_path):
            try:
                from dt_data_api import DataClient # Import here after installation
                client = DataClient(YOUR_DT_TOKEN)
                storage = client.storage("user")
                print(f"Uploading {model_local_path} to {model_remote_path}")
                upload = storage.upload(model_local_path, model_remote_path)
                upload.join()
                print("Upload complete.")
            except Exception as e:
                print(f"An error occurred during upload: {e}")
        else:
            print(f"Skipping upload: model_local_path is not valid ('{model_local_path}'). Training might have failed or model file not found.")

    print("
# Script finished!")
    print("# If training was successful, you can now use the trained model.")

if __name__ == '__main__':
    main()
