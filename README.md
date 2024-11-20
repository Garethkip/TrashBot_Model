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

# MoveIt Tutorial: Pick and Place with ROS 2 Humble
# ROS2 Humble MoveIt

## Getting Started

### Steps:

1. **Install ROS2** by following [this link](https://docs.ros.org/en/rolling/Installation.html).
2. **Source the ROS2 install** itself:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. **Install `rosdep`** to install system dependencies:
   ```bash
   sudo apt install python3-rosdep
   ```
4. **Update packages** after having ROS2 installed:
   ```bash
   sudo rosdep init
   rosdep update
   sudo apt update
   sudo apt dist-upgrade
   ```
5. **Install Colcon**, the ROS 2 build system, with mixin:
   ```bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default
   ```
6. **Install vcstool**:
   ```bash
   sudo apt install python3-vcstool
   ```
7. **Create workspace**:
   ```bash
   mkdir -p ~/ws_moveit2/src
   ```
8. **Move into your Colcon workspace** and pull the MoveIt tutorials source:
   ```bash
   cd ~/ws_moveit2/src
   git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
   ```
9. **Download the source code** for the rest of MoveIt:
   ```bash
   vcs import < moveit2_tutorials/moveit2_tutorials.repos
   ```
10. **Install dependencies** not already in your workspace:
    ```bash
    sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```
11. **Configure your Colcon workspace**:
    ```bash
    cd ~/ws_moveit2
    colcon build --mixin release
    ```
    This build command will likely take a long time (20+ minutes) depending on your computer speed and amount of RAM available (we recommend 32 GB). If you are short on computer memory or generally your build is struggling to complete on your computer, you can append the argument `--parallel-workers 1` to the `colcon` command above.
12. **Source the Colcon workspace**:
    ```bash
    source ~/ws_moveit2/install/setup.bash
    ```
13. **Add the previous command to your bashrc**:
    ```bash
    echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
    ```
14. **Switch to Cyclone DDS**:
    ```bash
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    # You may want to add this to ~/.bashrc to source it automatically
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
##MoveIt Quickstart in RViz

### Steps:

1. **Launch the Demo and Configure the Plugin**:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz
   ```
   In the RViz Displays Tab, press Add then from the moveit_ros_visualization folder, choose “MotionPlanning” as the DisplayType. Press “Ok”. You should now see the Panda demo robot.

   - Once you have the Motion Planning Plugin loaded, we can configure it. In the “Global Options” tab of the “Displays” subwindow, set the Fixed Frame field to /panda_link0.
   - Now, you can start configuring the Plugin for your robot (the Panda in this case). Click on “MotionPlanning” within “Displays”.
       - Make sure the Robot Description field is set to robot_description.
       - Make sure the Planning Scene Topic field is set to /monitored_planning_scene. Click on topic name to expose topic-name drop-down.
       - Make sure the Trajectory Topic under Planned Path is set to /display_planned_path.
       - In Planning Request, change the Planning Group to panda_arm. You can also see this in the MotionPlanning panel in the bottom left.

2. **Play with the Visualized Robots**: There are four different overlapping visualizations:
    - The robot’s configuration in the /monitored_planning_scene planning environment (active by default).
    - The planned path for the robot (active by default).
    - **Green**: The start state for motion planning (disabled by default).
    - **Orange**: The goal state for motion planning (active by default).

3. **Interact with the Panda**: [Step 3 - Interact with the Panda](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#step-3-interact-with-the-panda).

4. **Use Motion Planning with the Panda**: [Step 4 - Use Motion Planning with the Panda](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#step-4-use-motion-planning-with-the-panda).

5. **RViz Visual Tools**: Enable the RvizVisualToolsGui in the 'add panels' sections.

6. **Save your configuration**:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=your_rviz_config.rviz
   ```
## Your First C++ MoveIt Project

### Steps:

1. **Create a package within the src directory**:
   ```bash
   ros2 pkg create \
    --build-type ament_cmake \
    --dependencies moveit_ros_planning_interface rclcpp \
    --node-name hello_moveit hello_moveit
   ```
2. **Create a ROS Node and Executor**:
   ```cpp
   #include <memory>

   #include <rclcpp/rclcpp.hpp>
   #include <moveit/move_group_interface/move_group_interface.h>

   int main(int argc, char * argv[])
   {
     // Initialize ROS and create the Node
     rclcpp::init(argc, argv);
     auto const node = std::make_shared<rclcpp::Node>(
       "hello_moveit",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
     );

     // Create a ROS logger
     auto const logger = rclcpp::get_logger("hello_moveit");

     // Next step goes here

     // Shutdown ROS
     rclcpp::shutdown();
     return 0;
   }
   ```
   - **Build and run**:
     ```bash
     colcon build --mixin debug
     ```
   - On a new terminal:
     ```bash
     cd ~/ws_moveit2
     source install/setup.bash
     ```
   - **Run your program**:
     ```bash
     ros2 run hello_moveit hello_moveit
     ```

3. **Plan and Execute using MoveGroupInterface**:
   - After the comment that says "//next step goes here", add this:
     ```cpp
     // Create the MoveIt MoveGroup Interface
     using moveit::planning_interface::MoveGroupInterface;
     auto move_group_interface = MoveGroupInterface(node, "panda_arm");

     // Set a target Pose
     auto const target_pose = []{
       geometry_msgs::msg::Pose msg;
       msg.orientation.w = 1.0;
       msg.position.x = 0.28;
       msg.position.y = -0.2;
       msg.position.z = 0.5;
       return msg;
     }();
     move_group_interface.setPoseTarget(target_pose);

     // Create a plan to that target pose
     auto const [success, plan] = [&move_group_interface]{
       moveit::planning_interface::MoveGroupInterface::Plan msg;
       auto const ok = static_cast<bool>(move_group_interface.plan(msg));
       return std::make_pair(ok, msg);
     }();

     // Execute the plan
     if(success) {
       move_group_interface.execute(plan);
     } else {
       RCLCPP_ERROR(logger, "Planning failed!");
     }
     ```
   - **Build and Run**:
     ```bash
     colcon build --mixin debug
     ```
   - In a separate terminal:
     ```bash
     ros2 launch moveit2_tutorials demo.launch.py
     ```
   - In a separate terminal:
     ```bash
     ros2 run hello_moveit hello_moveit
     ```
   - **Explanation of the code**: [Explanation](https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html#id2)

## Visualizing In RViz

### Steps:

1. **Add the dependency moveit_visual_tools**:
   ```xml
   <depend>moveit_visual_tools</depend>
   ```

   - Then in your CMakeLists.txt, add this line to the section of `find_package` statements:
     ```cmake
     find_package(moveit_visual_tools REQUIRED)
     ```
   - Further down in the file, extend the `ament_target_dependencies` macro call to include the new dependency like this:
     ```cmake
     ament_target_dependencies(
       hello_moveit
       "moveit_ros_planning_interface"
       "moveit_visual_tools"
       "rclcpp"
     )
     ```
   - Add the required include to your source file `hello_moveit.cpp`:
     ```cpp
     #include <moveit_visual_tools/moveit_visual_tools.h>
     ```
   - **Build**:
     ```bash
     cd ~/ws_moveit2
     colcon build --mixin debug
     ```

2. **Create a ROS executor and spin the node on a thread**:
   - Make the following changes:
     ```cpp
     #include <thread>  // <---- add this to the set of includes at the top

     ...

     // Create a ROS logger
     auto const logger = rclcpp::get_logger("hello_moveit");

     // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
     rclcpp::executors::SingleThreadedExecutor executor;
     executor.add_node(node);
     auto spinner = std::thread([&executor]() { executor.spin(); });

     // Create the MoveIt MoveGroup Interface
     ...

     // Shutdown ROS
     rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
     spinner.join();  // <--- Join the thread before exiting
     return 0;
     ```
3. **Create and Initialize MoveItVisualTools**:
   ```cpp
   // Create the MoveIt MoveGroup Interface
   using moveit::planning_interface::MoveGroupInterface;
   auto move_group_interface = MoveGroupInterface(node, "panda_arm");

   // Construct and initialize MoveItVisualTools
   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
       node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
       move_group_interface.getRobotModel()};
   moveit_visual_tools.deleteAllMarkers();
   moveit_visual_tools.loadRemoteControl();
   ```

4. **Write closures for visualizations**:
   ```cpp
   // Create closures for visualization
   auto const draw_title = [&moveit_visual_tools](auto text) {
     auto const text_pose = [] {
       auto msg = Eigen::Isometry3d::Identity();
       msg.translation().z() = 1.0;
       return msg;
     }();
     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                     rviz_visual_tools::XLARGE);
   };
   auto const prompt = [&moveit_visual_tools](auto text) {
     moveit_visual_tools.prompt(text);
   };
   auto const draw_trajectory_tool_path =
       [&moveit_visual_tools,
        jmg = move_group_interface.getRobotModel()->getJointModelGroup(
            "panda_arm")](auto const trajectory) {
         moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
       };
   ```

5. **Visualize the steps of your program**:
   ```cpp
   // Set a target Pose
   auto const target_pose = [] {
     geometry_msgs::msg::Pose msg;
     msg.orientation.w = 1.0;
     msg.position.x = 0.28;
     msg.position.y = -0.2;
     msg.position.z = 0.5;
     return msg;
   }();
   move_group_interface.setPoseTarget(target_pose);

   // Create a plan to that target pose
   prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
   draw_title("Planning");
   moveit_visual_tools.trigger();
   auto const [success, plan] = [&move_group_interface] {
     moveit::planning_interface::MoveGroupInterface::Plan msg;
     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
     return std::make_pair(ok, msg);
   }();

   // Execute the plan
   if (success) {
     draw_trajectory_tool_path(plan.trajectory_);
     moveit_visual_tools.trigger();
     prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
     draw_title("Executing");
     moveit_visual_tools.trigger();
     move_group_interface.execute(plan);
   } else {
     draw_title("Planning Failed!");
     moveit_visual_tools.trigger();
     RCLCPP_ERROR(logger, "Planning failed!");
   }
   ```

   - **Build to see if everything is working fine**:
     ```bash
     cd ~/ws_moveit2
     source /opt/ros/humble/setup.bash
     colcon build --mixin debug
     ```

6. **Enable visualization in RViz**:
   ```bash
   cd ~/ws_moveit2
   source install/setup.bash
   ros2 launch moveit2_tutorials demo.launch.py
   ```

7. **In a new terminal**:
   ```bash
   cd ~/ws_moveit2
   source install/setup.bash
   ros2 run hello_moveit hello_moveit
   ```

8. **Here's the copy of the hello_moveit.cpp file**: [hello_moveit.cpp](https://github.com/moveit/moveit2_tutorials/blob/main/doc/tutorials/visualizing_in_rviz/hello_moveit.cpp)

## Planning Around Objects

### Steps:

1. **Add include for Planning Scene Interface** at the top of your source file:
   ```cpp
   #include <moveit/planning_scene_interface/planning_scene_interface.h>
   ```

2. **Change the Target Pose**:
   ```cpp
   // Set a target Pose
   auto const target_pose = [] {
     geometry_msgs::msg::Pose msg;
     msg.orientation.w = 1.0;
     msg.position.x = 0.28;
     msg.position.y = 0.4;  // <---- This value was changed
     msg.position.z = 0.5;
     return msg;
   }();
   move_group_interface.setPoseTarget(target_pose);
   ```

3. **Create a Collision Object**:
   ```cpp
   // Create collision object for the robot to avoid
   auto const collision_object = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
     moveit_msgs::msg::CollisionObject collision_object;
     collision_object.header.frame_id = frame_id;
     collision_object.id = "box1";
     shape_msgs::msg::SolidPrimitive primitive;

     // Define the size of the box in meters
     primitive.type = primitive.BOX;
     primitive.dimensions.resize(3);
     primitive.dimensions[primitive.BOX_X] = 0.5;
     primitive.dimensions[primitive.BOX_Y] = 0.1;
     primitive.dimensions[primitive.BOX_Z] = 0.5;

     // Define the pose of the box (relative to the frame_id)
     geometry_msgs::msg::Pose box_pose;
     box_pose.orientation.w = 1.0;
     box_pose.position.x = 0.2;
     box_pose.position.y = 0.2;
     box_pose.position.z = 0.25;

     collision_object.primitives.push_back(primitive);
     collision_object.primitive_poses.push_back(box_pose);
     collision_object.operation = collision_object.ADD;

     return collision_object;
   }();
   ```

4. **Add the Object to the Planning Scene**:
   ```cpp
   // Add the collision object to the scene
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   planning_scene_interface.applyCollisionObject(collision_object);
   ```

5. **Run the Program and Observe the Change**:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz
   ```

6. **A copy of the full hello_moveit.cpp file**:
   [hello_moveit.cpp](ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz)

## Pick and Place with MoveIt Task Constructor

### Steps:

1. **Download MoveIt Task Constructor**:
   ```bash
   cd ~/ws_moveit2/src
   git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2
   ```

2. **Create a New Package**:
   ```bash
   ros2 pkg create --build-type ament_cmake --node-name mtc_tutorial mtc_tutorial
   ```

3. **Add the dependencies to package.xml**:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
   <name>mtc_tutorial</name>
   <version>0.0.0</version>
   <description>TODO: Package description</description>
   <maintainer email="youremail@domain.com">user</maintainer>
   <license>TODO: License declaration</license>

   <buildtool_depend>ament_cmake</buildtool_depend>

   <depend>moveit_task_constructor_core</depend>
   <depend>rclcpp</depend>

   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>

   <export>
       <build_type>ament_cmake</build_type>
   </export>
   </package>
   ```

4. **Add dependencies to CMakeLists.txt**:
   ```cmake
        cmake_minimum_required(VERSION 3.8)
        project(mtc_tutorial)

        if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
        endif()

        # find dependencies
        find_package(ament_cmake REQUIRED)
        find_package(moveit_task_constructor_core REQUIRED)
        find_package(rclcpp REQUIRED)
        # uncomment the following section in order to fill in
        # further dependencies manually.
        # find_package(<dependency> REQUIRED)

        add_executable(mtc_tutorial src/mtc_tutorial.cpp)
        ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
        target_include_directories(mtc_tutorial PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
        target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

        install(TARGETS mtc_tutorial
        DESTINATION lib/${PROJECT_NAME})

        if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # uncomment the line when a copyright and license is not present in all source files
        #set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # uncomment the line when this package is not in a git repo
        #set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
        endif()

        ament_package()
   ```

5. **Setting up the project with MoveIt Task Constructor** in the mtc_tutorial.cpp:
   ```cpp
        #include <rclcpp/rclcpp.hpp>
        #include <moveit/planning_scene/planning_scene.h>
        #include <moveit/planning_scene_interface/planning_scene_interface.h>
        #include <moveit/task_constructor/task.h>
        #include <moveit/task_constructor/solvers.h>
        #include <moveit/task_constructor/stages.h>
        #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
        #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
        #else
        #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
        #endif
        #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
        #include <tf2_eigen/tf2_eigen.hpp>
        #else
        #include <tf2_eigen/tf2_eigen.h>
        #endif

        static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
        namespace mtc = moveit::task_constructor;

        class MTCTaskNode
        {
        public:
        MTCTaskNode(const rclcpp::NodeOptions& options);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        void doTask();

        void setupPlanningScene();

        private:
        // Compose an MTC task from a series of stages.
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        };

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
        {
        return node_->get_node_base_interface();
        }

        MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
        : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
        {
        }

        void MTCTaskNode::setupPlanningScene()
        {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "world";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 };

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = -0.25;
        pose.orientation.w = 1.0;
        object.pose = pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);
        }

        void MTCTaskNode::doTask()
        {
        task_ = createTask();

        try
        {
            task_.init();
        }
        catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return;
        }

        if (!task_.plan(5))
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return;
        }

        return;
        }

        mtc::Task MTCTaskNode::createTask()
        {
        mtc::Task task;
        task.stages()->setName("demo task");
        task.loadRobotModel(node_);

        const auto& arm_group_name = "panda_arm";
        const auto& hand_group_name = "hand";
        const auto& hand_frame = "panda_hand";

        // Set task properties
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // Disable warnings for this line, as it's a variable that's set but not used in this example
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
        mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
        #pragma GCC diagnostic pop

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);

        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage_open_hand->setGroup(hand_group_name);
        stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));

        return task;
        }

        int main(int argc, char** argv)
        {
        rclcpp::init(argc, argv);

        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);

        auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
        rclcpp::executors::MultiThreadedExecutor executor;

        auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
            executor.add_node(mtc_task_node->getNodeBaseInterface());
            executor.spin();
            executor.remove_node(mtc_task_node->getNodeBaseInterface());
        });

        mtc_task_node->setupPlanningScene();
        mtc_task_node->doTask();

        spin_thread->join();
        rclcpp::shutdown();
        return 0;
        }   
   ```

6. **Running the demo**:
   - **Launch file**: Save it as `pick_place_demo.launch.py`.
     ```python
     from launch import LaunchDescription
     from launch_ros.actions import Node
     from moveit_configs_utils import MoveItConfigsBuilder

     def generate_launch_description():
         moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

         # MTC Demo node
         pick_place_demo = Node(
             package="mtc_tutorial",
             executable="mtc_tutorial",
             output="screen",
             parameters=[
                 moveit_config,
             ],
         )

         return LaunchDescription([pick_place_demo])
     ```
   - **Add this line to your CMakeLists.txt**:
     ```cmake
     install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
     ```

7. **Build and source**:
   ```bash
   cd ~/ws_moveit2
   colcon build --mixin release
   source ~/ws_moveit2/install/setup.bash
   ```

8. **Launch your first launch file**:
   ```bash
   ros2 launch moveit2_tutorials mtc_demo.launch.py
   ```

9. **Configure your RViz**: [RViz Configuration](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html#rviz-configuration)

10. **Launch the demo**:
    ```bash
    ros2 launch mtc_tutorial pick_place_demo.launch.py
    ```
    - Here, your arm will appear with an open hand and an object in front of it.

11. **Adding more stages to enable the arm to pick and place the object**:
    ```cpp
    To be added soon.
    ```

12. **Visualizing with RViz**:
    ```bash
    ros2 launch moveit2_tutorials mtc_demo.launch.py
    ```

13. **In a second terminal**:
    ```bash
    ros2 launch moveit2_tutorials pick_place_demo.launch.py
    ```

