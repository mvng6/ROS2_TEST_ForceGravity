# ROS2 Gravity Compensation Simulation

This ROS2 package simulates and visualizes the effect of gravity on a tool attached to a 6-DOF robot's end-effector. It is designed to run with the Doosan Robot M-series model in RViz on ROS2 Humble.

---

## ⚙️ Key Features

-   Calculates the gravity wrench (force and torque) in real-time based on the robot's orientation.
-   Visualizes the calculated force and torque vectors in RViz.
-   Provides a clean, reusable C++ library for gravity compensation logic.

---

## 🔧 Dependencies

To run this simulation, the following dependencies must be installed:

-   **ROS2 Humble Hawksbill**
-   **Doosan Robot ROS2 Package:**
    ```bash
    git clone -b humble [https://github.com/doosan-robotics/doosan-robot2.git](https://github.com/doosan-robotics/doosan-robot2.git)
    ```
-   **Gazebo ROS2 Packages (Required by Doosan package):**
    ```bash
    sudo apt update && sudo apt install ros-humble-gazebo-ros-pkgs
    ```
-   **POCO C++ Libraries (Required by Doosan package):**
    ```bash
    sudo apt install libpoco-dev
    ```

---

## 🚀 Installation & Build

1.  **Create and navigate to your ROS2 workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone this repository and the Doosan robot package:**
    ```bash
    git clone [https://github.com/mvng6/ROS2_TEST_ForceGravity.git](https://github.com/mvng6/ROS2_TEST_ForceGravity.git) # 이 주소는 예시입니다. 본인 저장소 주소로 변경하세요.
    git clone -b humble [https://github.com/doosan-robotics/doosan-robot2.git](https://github.com/doosan-robotics/doosan-robot2.git)
    ```

3.  **Install dependencies:**
    ```bash
    cd ~/ros2_ws
    sudo apt update
    sudo apt install ros-humble-gazebo-ros-pkgs libpoco-dev
    ```

4.  **Build the workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

---

## ▶️ How to Run

1.  **Open Terminal 1 and source the workspace, then launch RViz:**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch dsr_description2 dsr_description.launch.py
    ```

2.  **Open Terminal 2 and source the workspace, then run the gravity compensation node:**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run gravity_compensation_pkg gravity_comp_node --ros-args -p use_sim_time:=true
    ```

3.  **In RViz:**
    -   Add the **TF** display to visualize coordinate frames.
    -   Add the **Wrench** display and subscribe to the `/gravity_comp_node/gravity_wrench` topic to visualize the force vector.
    -   Use the **Joint State Publisher** GUI to move the robot and observe the changes.
