🔽 Instructions to run the sandbox locally
==========================================

To run the sandbox locally using [Docker](https://docs.docker.com/get-docker), go through the following steps:

1. Build the docker image:
   ```console
   cd dockerfiles
   docker build . --build-arg ROBOTOLOGY_SUPERBUILD_RELEASE=releases/2021.05 \
                  --build-arg BUILD_TYPE=Release \
                  --tag icub-fkin-inspection
   ```
2. Launch the container:
    ```console
    docker run -it -p 6080:6080 --name <container_name> icub-fkin-inspection
    ```
3. From within the container shell, launch the following scripts:
    ```console
    start-vnc-session.sh
    ```
4. Open up the browser and connect to **`localhost:6080`** to get to the workspace desktop GUI.

5. On a terminal, either in the desktop GUI or the container shell, navigate to the project folder by typing `cd /workspace/icub-forward-kinematics-inspection` . The project should already be compiled and ready to run.

5.  Once done, from the container shell press **CTRL+D** to exit.

