FROM osrf/ros:jazzy-desktop

# ติดตั้งเครื่องมือที่ต้องใช้บ่อยๆ
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git nano \
    && rm -rf /var/lib/apt/lists/*

# ติดตั้ง Library สำหรับ Mobile Robot (ตัวอย่าง)
RUN pip3 install --no-cache-dir numpy scipy matplotlib --break-system-packages

# Auto-source ROS 2 ทุกครั้งที่เปิด Terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /workspace

# ติดตั้ง Foxglove Bridge สำหรับส่งข้อมูล
RUN apt-get update && apt-get install -y \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*