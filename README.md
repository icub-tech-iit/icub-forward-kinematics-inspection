# icub-forward-kinematics-inspection
A study on the computation of the forward kinematics of iCub's arm, to inspect potential issues.

### ☁ Launch the cloud workspace
We make use of [Gitpod Cloud IDE](https://gitpod.io) as infrastructure:
- Use your GitHub credentials to sign up for Gitpod. You will have [**50 hours/month**](https://www.gitpod.io/pricing) free on public repositories.
- Learn how to deal with our [Gitpod-enabled repositories](https://github.com/robotology/community/discussions/459).
- Finally, click on the badge below to launch the Gitpod workspace!
  
  [![](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/mfussi66/icub-forward-kinematics-inspection)

### ⚙ Build the project
```console
cd /workspace/icub/forward-kinematics-inspection
mkdir build && cd build
cmake ..
make && make install
```

### 🔘 Run the project
If you ran `make install`, launch the project with `fkin` in a terminal.
