# TrashBot_Model

This repository contains a simple yet effective model that detects and categorizes pollutants found in oceans, rivers, lakes, and open seas using computer vision. The project leverages YOLO for object detection and classification to address environmental challenges related to water pollution.

---

## Features
- Detects and categorizes various types of trash and pollutants, including:
  - **Fishing wastes**
  - **Glass wastes**
  - **Metal wastes**
  - **Natural wastes**
  - **Plastic wastes**
- Custom-trained on a labeled dataset for high accuracy in diverse water environments.
- Scalable and adaptable to new trash categories with additional training.

---

## Dataset Structure
The dataset should be structured as follows:

```
New_data_set/
├── train/
│   ├── images/
│   ├── labels/
├── valid/
│   ├── images/
│   ├── labels/
├── test/
│   ├── images/
│   ├── labels/
└── data.yaml
```

- **`images/`**: Contains the images for training, validation, and testing.
- **`labels/`**: Contains the corresponding YOLO annotation files for each image.
- **`data.yaml`**: Config file specifying paths, number of classes, and class names.

---

## Installation
To run this project, follow the steps below:

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/TrashBot_Model.git
   cd TrashBot_Model
   ```

2. Install dependencies:
   - Ensure you have Python 3.8 or later installed.
   - Install required Python libraries:
     ```bash
     pip install -r requirements.txt
     ```

3. Install PyTorch (ensure compatibility with your GPU/CPU):
   ```bash
   pip install torch torchvision
   ```

4. Install YOLOv5 (if not already included):
   ```bash
   pip install yolov5
   ```

---

## Training the Model
To train the model using the dataset:

1. Upload your dataset (`New_data_set.zip`) to Google Drive.

2. Open the provided Google Colab notebook provided in this roboflow account: [YOLO_Model_Training_]([https://universe.roboflow.com/avibot/trashy2]).

3. Follow these steps in the notebook:
   - Mount Google Drive.
   - Unzip and prepare the dataset.
   - Update the `.yaml` file with correct paths.
   - Train the model using:
     ```bash
     python train.py --img 640 --batch 16 --epochs 50 --data /content/New_data_set/data.yaml --weights yolov5s.pt
     ```

---

## Testing the Model
After training, test the model's performance:

1. Use the saved weights from the training output.
2. Run inference on test images:
   ```bash
   python detect.py --weights runs/train/exp/weights/best.pt --img 640 --conf 0.5 --source /content/New_data_set/test/images
   ```
3. View the results in the `runs/detect/exp/` folder.

---

## Results
- The model achieves high precision and recall for detecting water pollutants.
- Example detections:
  - Fishing nets
  - Plastic bottles
  - Metal cans

---

## Future Work
- Integrate real-time detection for deployment in drones or robots.
- Expand the dataset to include more pollutant types.
- Use transfer learning to enhance detection in low-resource environments.

---

## License
This project is licensed under the [MIT License](LICENSE).

---

## Acknowledgments
Special thanks to:
- [YOLOv5](https://github.com/ultralytics/yolov5) for the object detection framework.
- Dataset contributors and environmental organizations for making this project possible.
```
