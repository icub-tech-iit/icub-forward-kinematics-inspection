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
 * Definitions of KinThread functions
 */
KinThread::KinThread(double period, const std::string& modelPath,
                     const yarp::sig::Vector& joints)
    : yarp::os::PeriodicThread(period),
      armEncValues(joints.subVector(3, 9)),
      torsoEncValues(joints.subVector(0, 2)),
      arm("left_v2.5"),
      kinDynCompute(),
      modelPath(modelPath) {
  yInfo() << "KinThread constructor";


  torsoEncValues *= iCub::ctrl::CTRL_DEG2RAD;
  armEncValues *= iCub::ctrl::CTRL_DEG2RAD;
  }

KinThread::~KinThread() {}

bool KinThread::threadInit() {
  yarp::os::Property optArm;
  yarp::os::Property optTorso;

  yInfo() << "Port configuration in progress...";

  arm.toLinksProperties(armProperties);

  return true;
}

void KinThread::run() {
  yInfo() << "KinThread is running correctly ...";

 

void KinThread::threadRelease() {
  yInfo() << "KinThread is shutting down...";

  driverArm.close();
  driverTorso.close();
}

/**
 * Definitions of KinModule functions
 */
KinModule::KinModule() : RFModule() {}
KinModule::~KinModule() {}

bool KinModule::configure(yarp::os::ResourceFinder& rf) {
  if (!rf.check("model")) {
    yError() << "URDF robot model not provided.";
    return false;
  }

  auto modelPath = rf.find("model").asString();

  yarp::sig::Vector jointsValues;

  if (rf.check("joints")) {
    const auto* jointsList = rf.find("joints").asList();

    if (jointsList->size() < 10) {
      yError () << "joints argument requires 10 elements (in degrees),but"
      "only" << joints->size() << " were provided.";
    }
    for (size_t i = 0; i < jointsList->size(); ++i) {
      jointsValues.push_back(jointsList->get(i).asDouble());
    }
  }

  arm.releaseLink(0);
  arm.releaseLink(1);
  arm.releaseLink(2);
  arm.setAllConstraints(false);

  std::vector<std::string> axesList;
  axesList.push_back("torso_pitch");
  axesList.push_back("torso_yaw");
  axesList.push_back("torso_roll");

  // Left arm
  axesList.push_back("l_shoulder_pitch");
  axesList.push_back("l_shoulder_roll");
  axesList.push_back("l_shoulder_yaw");
  axesList.push_back("l_elbow");
  axesList.push_back("l_wrist_prosup");
  axesList.push_back("l_wrist_pitch");
  axesList.push_back("l_wrist_yaw");

  loadIDynTreeKinematicsFromUrdf(modelPath, axesList);


  jointsList = jointsList * iCub::ctrl::CTRL_DEG2RAD;

  std::swap(torsoEncValues[0], torsoEncValues[2]);
  auto ang = yarp::math::cat(torsoEncValues, armEncValues);
  //thr = std::make_unique<KinThread>(1, modelPath, jointsValues);

  //return thr->start();
}

bool KinModule::loadIDynTreeKinematicsFromUrdf(const std::string& modelPath, 
const std::vector<std::string>& axesList) {
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

bool KinModule::close() {
  yInfo() << "Stopping the encoder reading thread...";

  thr->stop();
  return true;
}

double KinModule::getPeriod() { return 1.; }

bool KinModule::updateModule() {
  yInfo() << "KinModule is running correctly...";

  yInfo() << "iDynTree data: n_dofs: "
          << kinDynCompute.getNrOfDegreesOfFreedom()
          << " n_frames: " << kinDynCompute.getNrOfFrames()
          << " n_links: " << kinDynCompute.getNrOfLinks()
          << " n_pos_coords: " << kinDynCompute.model().getNrOfPosCoords();

  kinDynCompute.setJointPos(ang.subVector(0, 9));
  armChain = arm.asChain();
  auto DynH = kinDynCompute.getRelativeTransform("root_link", 
"l_hand_dh_frame");
  auto KinH = arm.getH(ang);

  yInfo() << "----- iKin H Transform -----\n" << KinH.toString(5, 3);

  yInfo() << "properties: " << armProperties.toString();
  yInfo() << "H0: " << armProperties.find("H0").toString();

  yInfo() <<  "HN: " << armProperties.find("HN").toString();

  yInfo() << "----- iDyn H Transform -----\n"
          << DynH.getRotation().toString()
          << "pos: " << DynH.getPosition().toString();
  yInfo() << "-------------------------";
}


  return true;
}
