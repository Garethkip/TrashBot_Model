import os
from PIL import Image, ImageDraw
import cv2
from ultralytics import YOLO
from pathlib import Path

# Define paths
input_folder = Path(r'E:\Comp_Stuff\Glenn_s Shtuff\TrashBot\Data')  # Folder containing water body images
output_folder = Path(r'E:\Comp_Stuff\Glenn_s Shtuff\TrashBot\out')  # Folder to save annotated images
output_folder.mkdir(parents=True, exist_ok=True)

# Load YOLO model trained for trash detection
model = YOLO(r'E:\Comp_Stuff\Glenn_s Shtuff\Falgari\best.pt')  # Full path to model file

# Define trash classes
TRASH_CLASSES = ['Fishing wastes', 'Glass wastes', 'Metal wastes', 'Natural wastes', 'Plastic wastes']  # Ensure all classes match model training

# Annotate and save function
def annotate_and_save(image_path, detections, output_path):
    image = Image.open(image_path)
    draw = ImageDraw.Draw(image)

    for detection in detections:
        box = detection.xyxy[0].cpu().numpy()
        class_id = int(detection.cls[0].item())
        label = TRASH_CLASSES[class_id]
        confidence = detection.conf[0].item()

        x1, y1, x2, y2 = box[:4]
        draw.rectangle([x1, y1, x2, y2], outline='red', width=3)
        draw.text((x1, y1 - 10), f'{label} {confidence:.2f}', fill='red')

    image.save(output_path)
    print(f"Annotated image saved: {output_path}")

# Function to process images, detect trash, and annotate
def detect_and_annotate_trash(input_folder, output_folder):
    for img_file in input_folder.rglob('*.*'):  # Recursively find all image files
        relative_path = img_file.relative_to(input_folder)
        output_subfolder = output_folder / relative_path.parent
        output_subfolder.mkdir(parents=True, exist_ok=True)

        img_name = img_file.stem
        image_path = str(img_file)
        
        # Load image with OpenCV for YOLO inference
        img = cv2.imread(image_path)
        results = model(img)

        # Extract detections
        detections = results[0].boxes

        # Annotate and save only if trash is detected
        if any(detection.cls[0].item() in range(len(TRASH_CLASSES)) for detection in detections):
            annotated_output_path = output_subfolder / f"{img_name}_annotated.jpg"
            annotate_and_save(image_path, detections, str(annotated_output_path))
        else:
            print(f"No trash detected in {img_name}")

# Run the detection and annotation process
detect_and_annotate_trash(input_folder, output_folder)
