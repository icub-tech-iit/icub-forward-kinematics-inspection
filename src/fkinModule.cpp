/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <utility>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include "fkinModule.h"

/**
 * Definitions of KinThread functions
 */
KinThread::KinThread(double period)
  : yarp::os::PeriodicThread(period),
    arm("left"),
    armEncValues(),
    torsoEncValues() {
  yInfo() << "KinThread constructor";

  arm.releaseLink(0);
  arm.releaseLink(1);
  arm.releaseLink(2);
  arm.setAllConstraints(false);
  arm.setAng(yarp::math::zeros(arm.getDOF()));
}

KinThread::~KinThread() {}

bool KinThread::threadInit() {
  yarp::os::Property optArm;
  yarp::os::Property optTorso;

  optArm.put("device", "remote_controlboard");
  optArm.put("remote", "/icubSim/left_arm");
  optArm.put("local", "/logger");

  optTorso.put("device", "remote_controlboard");
  optTorso.put("remote", "/icubSim/torso");
  optTorso.put("local", "/logger");
  
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
  auto H = arm.getH(ang);

  yInfo() << H.toString(3, 3);
}

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

bool KinModule::configure(yarp::os::ResourceFinder &rf) {
  yInfo() << "Port configuration in progress...";

  thr = std::make_unique<KinThread>(0.01);

  return thr->start();
}

bool KinModule::close() {
  yInfo() << "Stopping the encoder reading thread...";

  thr->stop();
  return true;
}

double KinModule::getPeriod() {
  return 1.;
}

bool KinModule::updateModule() {
  yInfo() << "KinModule is running correctly...";

  return true;
}