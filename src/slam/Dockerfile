FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-cartographer-ros \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /slam
COPY . /slam
CMD ["python3", "/slam/bridge.py"]
