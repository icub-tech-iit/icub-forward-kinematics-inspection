/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cstdlib>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include "fkinModule.h"

int main(int argc, char *argv[]) {
  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "YARP doesn't seem to be available";
    return EXIT_FAILURE;
  }
  
  yarp::os::ResourceFinder rf;
  
  rf.setDefaultContext("fkin");
  rf.configure(argc, argv);

  KinModule mod;
  return mod.runModule(rf);
}
