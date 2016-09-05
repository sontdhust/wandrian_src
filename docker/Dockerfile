FROM sontdhust/ros:indigo-gazebo-kobuki

# Arguments
ARG user
ARG uid
ARG shell
ARG workspace

# Add new sudo user
RUN useradd -m ${user} && \
  echo "${user}:${user}" | chpasswd && \
  usermod --shell ${shell} ${user} && \
  usermod -aG sudo ${user} && \
  echo "${user} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${user} && \
  chmod 0440 /etc/sudoers.d/${user} && \
  usermod --uid ${uid} ${user} && \
  groupmod --gid ${uid} ${user}

# Switch to user
USER "${user}"

# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1

# Switch to the workspace
WORKDIR ${workspace}
