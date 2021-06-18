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
KinThread::KinThread(double period)
    : yarp::os::PeriodicThread(period),
      arm("left"),
      armEncValues(),
      torsoEncValues(),
      kinDynCompute(),
      model() {
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

  std::vector<std::string> axesList;
  axesList.push_back("torso_pitch");
  axesList.push_back("torso_roll");
  axesList.push_back("torso_yaw");

  // Left arm
  axesList.push_back("l_shoulder_pitch");
  axesList.push_back("l_shoulder_roll");
  axesList.push_back("l_shoulder_yaw");
  axesList.push_back("l_elbow");
  axesList.push_back("l_wrist_prosup");
  axesList.push_back("l_wrist_pitch");
  axesList.push_back("l_wrist_yaw");

  iDynTree::ModelLoader mdlLoader;
  std::string modelPath = "/home/mfussi/.local/share/iCub/robots/iCubGenova02/model.urdf";
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

  dynEncValues = ang.subVector(0, 9);

  kinDynCompute.setJointPos(dynEncValues);

  auto DynH = kinDynCompute.getRelativeTransform("torso_1", "l_hand");
  auto KinH = arm.getH(ang);

  yInfo() << "----- iKin H Matrix -----";
  yInfo() << "\n" << KinH.toString(3, 3);

  yInfo() << "----- iDyn H Matrix -----";
  yInfo() << "\n" << DynH.toString();

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
  yInfo() << "Port configuration in progress...";

  thr = std::make_unique<KinThread>(0.01);

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
