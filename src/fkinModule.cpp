/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "fkinModule.h"

#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Matrix.h>

#include <utility>

/**
 * Definitions of KinModule functions
 */
KinModule::KinModule(const std::string& armType)
    : arm(armType), kinDynCompute() {}

KinModule::~KinModule() {}

bool KinModule::configure(const yarp::os::ResourceFinder& rf) {
  if (!rf.check("model")) {
    yError() << "URDF robot model not provided.";
    return false;
  }
  auto modelPath = rf.find("model").asString();

  std::vector<std::string> axesListValues;
  // Torso
  axesListValues.push_back("torso_pitch");
  axesListValues.push_back("torso_yaw");
  axesListValues.push_back("torso_roll");

  // Left arm
  axesListValues.push_back("l_shoulder_pitch");
  axesListValues.push_back("l_shoulder_roll");
  axesListValues.push_back("l_shoulder_yaw");
  axesListValues.push_back("l_elbow");
  axesListValues.push_back("l_wrist_prosup");
  axesListValues.push_back("l_wrist_pitch");
  axesListValues.push_back("l_wrist_yaw");

  bool urdfLoaded = loadIDynTreeKinematicsFromUrdf(modelPath, axesListValues);

  if (!urdfLoaded) {
    return false;
  }

  if (!rf.check("joints")) {
    yError() << "Joints values not provided.";
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
    const std::string& modelPath, const std::vector<std::string>& axesList) {
  bool ok = true;
  iDynTree::ModelLoader mdlLoader;

  ok = mdlLoader.loadReducedModelFromFile(modelPath, axesList);
  ok = ok && kinDynCompute.loadRobotModel(mdlLoader.model());

  if (!ok) {
    yError() << "Unable to open model " << modelPath;
  } else {
    yInfo() << "URDF model loaded successfully.";
  }

  return ok;
}

bool KinModule::evaluateKinematics(const std::string& rootFrame, const std::string& endFrame) {
  yInfo() << "evaluateKinematics is running correctly...";

  yInfo() << "iDynTree data: n_dofs: "
          << kinDynCompute.getNrOfDegreesOfFreedom()
          << " n_frames: " << kinDynCompute.getNrOfFrames()
          << " n_links: " << kinDynCompute.getNrOfLinks()
          << " n_pos_coords: " << kinDynCompute.model().getNrOfPosCoords();

  kinDynCompute.setJointPos(jointsValues);
  armChain = arm.asChain();

  auto DynH = kinDynCompute.getRelativeTransform(rootFrame, endFrame);
  auto KinH = arm.getH(jointsValues);

  yInfo() << "----- iKin H Transform -----\n" << KinH.toString(5, 3);

  yInfo() << "properties: " << armProperties.toString();
  yInfo() << "H0: " << armProperties.find("H0").toString();

  yInfo() << "HN: " << armProperties.find("HN").toString();

  yInfo() << "----- iDyn H Transform -----\n"
          << DynH.getRotation().toString()
          << "pos: " << DynH.getPosition().toString();
  yInfo() << "-------------------------";

  return true;
}