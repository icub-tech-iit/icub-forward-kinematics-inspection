/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "fkinModule.h"

#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Matrix.h>

#include <utility>

/**
 * Definitions of KinModule functions
 */
KinModule::KinModule(const std::string& armType,
                     const std::vector<std::string>& axesListValues)
    : arm(armType),
      armChain(),
      armProperties(),
      kinDynCompute(),
      jointsValues(),
      axesList(axesListValues) {}

KinModule::~KinModule() {}

bool KinModule::configure(const yarp::os::ResourceFinder& rf) {
  if (!rf.check("model")) {
    std::cout << "URDF robot model not provided." << std::endl;
    return false;
  }
  auto modelPath = rf.find("model").asString();

  bool urdfLoaded = loadIDynTreeKinematicsFromUrdf(modelPath);

  if (!urdfLoaded) {
    return false;
  }

  if (!rf.check("joints")) {
    std::cout << "Joints values not provided." << std::endl;
    return false;
  } else {
    const auto* jointsList = rf.find("joints").asList();
    for (size_t i = 0; i < jointsList->size(); ++i) {
      jointsValues.push_back(jointsList->get(i).asDouble());
    }
  }

  jointsValues *= iCub::ctrl::CTRL_DEG2RAD;
  std::swap(jointsValues[0], jointsValues[2]);

  // Enable positioning of torso joints
  arm.releaseLink(0);
  arm.releaseLink(1);
  arm.releaseLink(2);

  arm.toLinksProperties(armProperties);

  // Remove constraints from joints
  arm.setAllConstraints(false);

  return true;
}

bool KinModule::loadIDynTreeKinematicsFromUrdf(
    const std::string& modelPath) {
  bool ok = true;
  iDynTree::ModelLoader mdlLoader;

  ok = mdlLoader.loadReducedModelFromFile(modelPath, axesList);
  ok = ok && kinDynCompute.loadRobotModel(mdlLoader.model());

  if (!ok) {
    std::cout << "Unable to open model " << modelPath << std::endl;
  } else {
    std::cout << "URDF model loaded successfully." << std::endl;
  }

  return ok;
}

void KinModule::evaluateKinematics(const std::string& rootFrame,
                                   const std::string& endFrame) {
  kinDynCompute.setJointPos(jointsValues);
  armChain = arm.asChain();

  auto DynH = kinDynCompute.getRelativeTransform(rootFrame, endFrame);
  auto KinH = arm.getH(jointsValues);

  std::cout << "-------------------------" << std::endl
            << "------- iKin Data -------" << std::endl
            << "-------------------------" << std::endl;

  std::cout << "Transform: " << std::endl << KinH.toString(5, 3) << std::endl;

  std::cout << "Properties: " << std::endl
            << armProperties.toString() << std::endl;

  std::cout << "H0: " << std::endl
            << armProperties.find("H0").toString() << std::endl;

  std::cout << "HN: " << std::endl
            << armProperties.find("HN").toString() << std::endl;

  std::cout << "-------------------------" << std::endl
            << "----- iDynTree Data -----" << std::endl
            << "-------------------------" << std::endl;

  std::cout << "Transform:" << std::endl
            << DynH.getRotation().toString()
            << "pos: " << DynH.getPosition().toString() << std::endl;
}
