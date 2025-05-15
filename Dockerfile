# FROM osrf/ros:jazzy-desktop
FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


RUN apt-get update \
    && apt-get install -y ros-jazzy-joint-state-publisher
#     && apt-get install -y curl lsb-release gnupg \
#     && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
#     && apt-get update \
#     && apt-get install -y gz-harmonic

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

ENV SHELL /bin/bash

USER $USERNAME
CMD ["/bin/bash"]