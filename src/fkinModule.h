/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <iCub/iKin/iKinFwd.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Link.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>


class KinModule {
 public:
  explicit KinModule(const std::string& armType);
  ~KinModule();

  bool configure(const yarp::os::ResourceFinder& rf);
  bool loadIDynTreeKinematicsFromUrdf(const std::string& modelPath,
                                      const std::vector<std::string>& axesList);
  void evaluateKinematics(const std::string& rootFrame, const std::string& endFrame);

 private:
  iCub::iKin::iCubArm arm;
  iCub::iKin::iKinChain* armChain;
  yarp::os::Property armProperties;
  iDynTree::KinDynComputations kinDynCompute;
  yarp::sig::Vector jointsValues;
};
