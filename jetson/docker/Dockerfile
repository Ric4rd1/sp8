FROM dustynv/ros:humble-desktop-l4t-r32.7.1

ARG USERNAME=sp8
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# USER ${USERNAME}
WORKDIR /home/${USERNAME}
