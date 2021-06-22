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
                     const std::vector<double>& joints)
    : yarp::os::PeriodicThread(period),
      arm("left_2.5"),
      armEncValues(),
      torsoEncValues(),
      kinDynCompute(),
      model(),
      modelPath(modelPath) {
  yInfo() << "KinThread constructor";
  arm.releaseLink(0);
  arm.releaseLink(1);
  arm.releaseLink(2);
  arm.setAllConstraints(false);
}

KinThread::~KinThread() {}

bool KinThread::threadInit() {
  yarp::os::Property optArm;
  yarp::os::Property optTorso;

  yInfo() << "Port configuration in progress...";

  optArm.put("device", "remote_controlboard");
  optArm.put("remote", "/icubSim/left_arm");
  optArm.put("local", "/logger/left_arm");

  optTorso.put("device", "remote_controlboard");
  optTorso.put("remote", "/icubSim/torso");
  optTorso.put("local", "/logger/torso");

  if (!driverArm.open(optArm)) {
    yError() << "Unable to connect to /icubSim/left_arm";
    return false;
  }

  if (!driverTorso.open(optTorso)) {
    yError() << "Unable to connect to /icubSim/torso";
    driverArm.close();
    return false;
  }

  // open views
  bool ok = true;
  ok = ok && driverTorso.view<yarp::dev::IEncoders>(iTorsoEnc);
  ok = ok && driverArm.view<yarp::dev::IEncoders>(iArmEnc);

  if (!ok) {
    yError() << "Unable to open views";
    driverArm.close();
    driverTorso.close();
    return false;
  }

  int nAxes;
  iTorsoEnc->getAxes(&nAxes);
  torsoEncValues.resize(nAxes);

  iArmEnc->getAxes(&nAxes);
  armEncValues.resize(nAxes);

  arm.toLinksProperties(armProperties);

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

  iDynTree::ModelLoader mdlLoader;
  ok = mdlLoader.loadReducedModelFromFile(modelPath, axesList);
  ok = ok && kinDynCompute.loadRobotModel(mdlLoader.model());
  model = kinDynCompute.model();

  if (!ok) {
    yError() << "Unable to open model " << modelPath;
    return ok;
  }

  return true;
}

void KinThread::run() {
  yInfo() << "KinThread is running correctly ...";

  iTorsoEnc->getEncoders(torsoEncValues.data());
  iArmEnc->getEncoders(armEncValues.data());

  torsoEncValues *= iCub::ctrl::CTRL_DEG2RAD;
  armEncValues *= iCub::ctrl::CTRL_DEG2RAD;

  std::swap(torsoEncValues[0], torsoEncValues[2]);
  auto ang = yarp::math::cat(torsoEncValues, armEncValues);

  yInfo() << "iDynTree data:: n_dofs: "
          << kinDynCompute.getNrOfDegreesOfFreedom()
          << " n_frames: " << kinDynCompute.getNrOfFrames()
          << " n_links: " << kinDynCompute.getNrOfLinks()
          << " n_pos_coords: " << kinDynCompute.model().getNrOfPosCoords();

  dynEncValues = ang.subVector(0, 9);

  kinDynCompute.setJointPos(dynEncValues);

  auto DynH =
      kinDynCompute.getRelativeTransform("root_link", "l_hand_dh_frame");
  auto KinH = arm.getH(ang);

  yInfo() << "----- iKin H Transform -----\n" << KinH.toString(5, 3);

  yInfo() << "H0: " << armProperties.find("H0").toString();

  yInfo() <<  "HN: " << armProperties.find("HN").toString();

  yInfo() << "----- iDyn H Transform -----\n"
          << DynH.getRotation().toString()
          << "pos: " << DynH.getPosition().toString();
  yInfo() << "-------------------------";
}

void KinThread::threadRelease() {
  yInfo() << "KinThread is shutting down...";

  driverArm.close();
  driverTorso.close();
}

bool KinThread::loadIDynModelFromUrdf(const std::string& filename,
                                      iDynTree::Model& model) {
  bool result = true;

  return result;
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

  std::vector<double> jointsValues;

  if (rf.check("joints")) {
    const auto* joints = rf.find("joints").asList();
    for (size_t i = 0; i < joints->size(); ++i) {
      jointsValues.push_back(joints->get(i).asDouble());
    }
  } else {
    yInfo() << "Joint positions not provided, using iCubSIM defaults.";
  }

  thr = std::make_unique<KinThread>(1., modelPath, jointsValues);

  return thr->start();
}

bool KinModule::close() {
  yInfo() << "Stopping the encoder reading thread...";

  thr->stop();
  return true;
}

double KinModule::getPeriod() { return 1.; }

bool KinModule::updateModule() {
  yInfo() << "KinModule is running correctly...";

  return true;
}
