/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "fkinModule.h"

#include <string>
#include <vector>

int main(int argc, char *argv[]) {
  yarp::os::ResourceFinder rf;

  rf.setDefaultContext("fkin");
  rf.configure(argc, argv);

  std::vector<std::string> axesList;
  // Torso
  axesList.push_back("torso_pitch");
  axesList.push_back("torso_yaw");
  axesList.push_back("torso_roll");

  // Arm
  axesList.push_back("r_shoulder_pitch");
  axesList.push_back("r_shoulder_roll");
  axesList.push_back("r_shoulder_yaw");
  axesList.push_back("r_elbow");
  axesList.push_back("r_wrist_prosup");
  axesList.push_back("r_wrist_pitch");
  axesList.push_back("r_wrist_yaw");

  KinModule mod("right_v2.5", axesList);

  if (!mod.configure(rf)) {
    return EXIT_FAILURE;
  }

  mod.evaluateKinematics("root_link", "r_hand_dh_frame");
  return EXIT_SUCCESS;
}
