# Use a base image with ROS installed
FROM ros:humble

# Install the dependencies
RUN apt-get update \
    && apt-get install -y ros-humble-xacro \
        python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /diff_drive_bot

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add the user to the dialout group to enable serial port access
RUN usermod -aG dialout ${USERNAME}

# Copy the entire ROS workspace into the container
COPY src/ src/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Copy the entrypoint script file
COPY entrypoint.sh /entrypoint.sh

# Copy the bashrc file to enable autocopletion
COPY bashrc /home/${USERNAME}/.bashrc

# Setup entrypoint and default command
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
CMD [ "bash" ]