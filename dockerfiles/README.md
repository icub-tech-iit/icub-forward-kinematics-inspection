🐳 Instructions to run the docker container
============================================

Make sure you have [Docker](https://docs.docker.com/get-docker) available on your sytem; then, go through the following steps:

1. Build the docker image:
   ```console
   cd dockerfiles
   docker build . --build-arg ROBOTOLOGY_SUPERBUILD_RELEASE=releases/2021.08 --build-arg ICUB_MAIN_FORK=mfussi66/icub-main --tag icub-fkin-inspection
   ```
2. Launch the container:
    ```console
    docker run -it -p 6080:6080 icub-fkin-inspection
    ```
3. From within the container shell, launch the following scripts:
    ```console
    start-vnc-session.sh
    ```
4. Open up the browser and connect to **`localhost:6080`** to get to the workspace desktop GUI.
5. You can now run the `fkin` executable.
6. To launch the red-ball test, make sure that `yarpserver` is running and then do:
   ```console
   cd /robotology-superbuild/src/icub-tests/suites
   robottestingframework-testrunner --verbose --suite demoRedBall-icubSim.xml
   ```
7.  Once done, from the container shell press **CTRL+D** to exit.
