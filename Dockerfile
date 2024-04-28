# Start from the Yahboom ROS Foxy base image
FROM yahboomtechnology/ros-foxy:4.0.0

# Install Python pip and other dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-matplotlib

# Install Python packages
RUN pip3 install Flask flask-socketio

# Clear the cache created by apt install
RUN rm -rf /var/lib/apt/lists/*
