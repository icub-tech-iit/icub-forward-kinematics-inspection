# icub-forward-kinematics-inspection
A study on the computation of the forward kinematics of iCub's arm, to inspect potential issues.

### ☁ Launch the cloud workspace
We make use of [Gitpod Cloud IDE](https://gitpod.io) as infrastructure:
- Use your GitHub credentials to sign up for Gitpod. You will have [**50 hours/month**](https://www.gitpod.io/pricing) free on public repositories.
- Learn how to deal with our [Gitpod-enabled repositories](https://github.com/robotology/community/discussions/459).
- Finally, click on the badge below to launch the Gitpod workspace!
  
  [![](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/mfussi66/icub-forward-kinematics-inspection)

### ⚙ Build the project
Move to the project folder, then on a terminal, type
```console
mkdir build && cd build
cmake ..
make && make install
```

### 🔘 Run the project
Running the project requires loading the URDF model of iCub 2.0, 2.5, or 2.7, all provided in the repository.

To run the project, you can use the script `test.sh`, which provides 4 joint configurations. 

To use it, run on a terminal `./test.sh <joint_cfg>`, where `joint_cfg` can be either 1, 2, 3, 4.

For more information, type `fkin --help`.