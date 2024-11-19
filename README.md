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

# MoveIt Tutorial: Pick and Place with ROS 2 Humble

This README provides a step-by-step guide to setting up and executing a pick-and-place robotic task using MoveIt on ROS 2 Humble.

---

## Prerequisites
Before starting, ensure the following are installed and configured:

1. **ROS 2 Humble**: Install it following the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

2. **MoveIt**: Install MoveIt from binary or source by following the [MoveIt installation guide](https://moveit.picknik.ai/humble/doc/install.html).

3. **Workspace Setup**: Create and source a workspace:
   ```bash
   mkdir -p ~/moveit_ws/src
   cd ~/moveit_ws
   colcon build
   source install/setup.bash
   ```

---

## Steps

### 1. Getting Started with MoveIt
Launch MoveIt in RViz to visualize and interact with your robot's motion:
```bash
ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz
```
This command initializes RViz, displaying the robot and allowing interactive planning.

---

### 2. Planning Around Objects
Add collision objects to the robot's planning environment to ensure safe motion planning.

#### Code Example:
```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

moveit_msgs::msg::CollisionObject collision_object;
collision_object.id = "box1";
collision_object.header.frame_id = "world";

// Create a box
shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions = {0.5, 0.5, 0.5};

// Define its pose in the world frame
geometry_msgs::msg::Pose box_pose;
box_pose.position.x = 0.4;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;
collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

// Add the object to the planning scene
planning_scene_interface.applyCollisionObject(collision_object);
```

---

### 3. Pick and Place with MoveIt Task Constructor (MTC)
Use MoveIt Task Constructor (MTC) to create a pick-and-place sequence. This approach defines modular stages for each task step (e.g., picking, transporting, and placing the object).

1. **Configure the Task**: Define the task, stages, and robot properties.
   ```cpp
   mtc::Task task;
   task.stages()->setName("demo task");
   task.loadRobotModel(node_);

   const auto& arm_group_name = "panda_arm";
   const auto& hand_group_name = "hand";
   const auto& hand_frame = "panda_hand";

   task.setProperty("group", arm_group_name);
   task.setProperty("eef", hand_group_name);
   task.setProperty("ik_frame", hand_frame);
   ```

2. **Add Motion Stages**: Implement stages such as pre-grasp pose, grasping, lifting, and placing.
   ```cpp
   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
   task.add(std::move(stage_state_current));
   ```

3. **Select a Motion Planner**: Choose planners like CartesianPath or PipelinePlanner for more complex motions.
   ```cpp
   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
   cartesian_planner->setMaxVelocityScalingFactor(1.0);
   ```

4. **Plan and Execute**: Plan the task and execute it.
   ```cpp
   task.init();
   if (!task.plan(5))
   {
       RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
       return;
   }

   auto result = task.execute(*task.solutions().front());
   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
   {
       RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
   }
   ```

---

### 4. Running the Complete Program
To visualize and execute your pick-and-place task, run the following:
```bash
ros2 run your_package your_pick_and_place_node
```

---

## Key Features
- **Collision Awareness**: Ensures safe motion around obstacles.
- **Task Modularity**: Easily customize each stage of the pick-and-place task.
- **Real-Time Feedback**: Visualize the task in RViz for real-time monitoring.

For more details and further tutorials, refer to the [MoveIt Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html).
