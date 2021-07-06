🔽 Instructions to run the sandbox locally
==========================================

To run the sandbox locally using [Docker](https://docs.docker.com/get-docker), go through the following steps:

1. Build the docker image:
   ```console
   cd dockerfiles
   docker build -f Dockerfile . --build-arg ROBOTOLOGY_SUPERBUILD_RELEASE=releases/2021.05 --build-arg BUILD_TYPE=Release
   ```
2. Launch the container:
    ```console
    docker run -it -p 6080:6080 --user gitpod <container friendly name>
    ```
3. From within the container shell, launch the following scripts:
    ```console
    start-vnc-session.sh
    ```
4. Clone and install the project:
    ```console
    git clone https://github.com/robotology/icub-gazebo-grasping-sandbox.git /workspace/icub-gazebo-grasping-sandbox
    cd /workspace/icub-gazebo-grasping-sandbox 
    mkdir build && cd build
    cmake ../
    sudo make install
    ```
5. Open up the browser and connect to **`localhost:6080`** to get to the workspace desktop GUI.
6.  In the desktop GUI, open a terminal and run the grasping experiment:
   ```console
   icub-grasp.sh
   ```
11. Once done, from the container shell press **CTRL+D**.

