/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include <cstdlib>

#include "fkinModule.h"

int main(int argc, char *argv[]) {
  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "YARP does not seem to be available";
    return EXIT_FAILURE;
  }

  yarp::os::ResourceFinder rf;

  rf.setDefaultContext("fkin");
  rf.configure(argc, argv);

  KinModule mod("left_v2.5");

  mod.configure(rf);

  return mod.evaluateKinematics("root_link", "l_hand_dh_frame");
}
