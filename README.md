# ROS2 중력 보상 시뮬레이션

이 ROS2 패키지는 6축 로봇의 엔드 이펙터에 부착된 툴에 작용하는 중력의 영향을 시뮬레이션하고 시각화합니다. ROS2 Humble 환경에서 두산 로보틱스 M-시리즈 모델과 함께 RViz를 통해 구동되도록 설계되었습니다.

---

## ⚙️ 주요 기능

-   로봇의 자세에 따라 실시간으로 중력 렌치(힘과 토크)를 계산
-   계산된 중력 벡터를 RViz에서 시각화
-   재사용 가능한 순수 C++ 기반의 중력 보상 로직 라이브러리 제공

---

## 🔧 의존성 (Dependencies)

이 시뮬레이션을 실행하기 위해서는 아래의 의존성 패키지들이 반드시 설치되어 있어야 합니다.

-   **ROS2 Humble Hawksbill**
-   **두산 로봇 ROS2 패키지 (`doosan-robot2`):**
    ```bash
    git clone -b humble [https://github.com/doosan-robotics/doosan-robot2.git](https://github.com/doosan-robotics/doosan-robot2.git)
    ```
-   **Gazebo ROS2 패키지 (두산 패키지 필요):**
    ```bash
    sudo apt update && sudo apt install ros-humble-gazebo-ros-pkgs
    ```
-   **POCO C++ 라이브러리 (두산 패키지 필요):**
    ```bash
    sudo apt install libpoco-dev
    ```

---

## 🚀 설치 및 빌드

1.  **ROS2 워크스페이스 생성 및 이동:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **이 저장소와 두산 로봇 패키지 클론:**
    ```bash
    git clone [https://github.com/mvng6/ROS2_TEST_ForceGravity.git](https://github.com/mvng6/ROS2_TEST_ForceGravity.git) 
    git clone -b humble [https://github.com/doosan-robotics/doosan-robot2.git](https://github.com/doosan-robotics/doosan-robot2.git)
    ```

3.  **의존성 패키지 설치:**
    ```bash
    cd ~/ros2_ws
    sudo apt update
    sudo apt install ros-humble-gazebo-ros-pkgs libpoco-dev
    ```

4.  **워크스페이스 빌드:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

---

## ▶️ 실행 방법

1.  **터미널 1: 워크스페이스 소싱 후 RViz 실행**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch dsr_description2 dsr_description.launch.py
    ```

2.  **터미널 2: 워크스페이스 소싱 후 중력 보상 노드 실행**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run gravity_compensation_pkg gravity_comp_node --ros-args -p use_sim_time:=true
    ```

3.  **RViz 설정:**
    -   **TF** 디스플레이를 추가하여 좌표계 시각화
    -   **Wrench** 디스플레이를 추가하고 `/gravity_comp_node/gravity_wrench` 토픽을 구독하여 힘 벡터 시각화
    -   **Joint State Publisher** GUI를 사용하여 로봇을 움직이며 변화 관찰
