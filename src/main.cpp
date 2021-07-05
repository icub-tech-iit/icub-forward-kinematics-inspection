/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "fkinModule.h"

int main(int argc, char *argv[]) {
  yarp::os::ResourceFinder rf;

  rf.setDefaultContext("fkin");
  rf.configure(argc, argv);

  KinModule mod("left_v2.5");

  mod.configure(rf);

  mod.evaluateKinematics("root_link", "l_hand_dh_frame");
  return 0;
}
