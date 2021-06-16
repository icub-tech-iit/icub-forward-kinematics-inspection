/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "fkinModule.h"

/**
 * Definitions of KinThread functions
 */
KinThread::KinThread(double period)
  : PeriodicThread(period),
    arm("left"),
    armEncValues(),
    armH(yarp::math::eye(4, 4)) {
  yInfo() << "KinThread constructor";

  arm.setAllConstraints(false);
  arm.setAng(yarp::math::zeros(arm.getDOF()));
}

KinThread::~KinThread() {}

bool KinThread::threadInit() {
  Property optArm;
  optArm.put("device", "remote_controlboard");
  optArm.put("remote", "/icubSim/left_arm");
  optArm.put("local", "/logger");

  if (!jointsClient.open(optArm)) {
    yError() << "Unable to connect to /icubSim/left_arm";
    return false;
  }

  // open views
  bool ok = true;
  ok = ok && jointsClient.view<IEncoders>(iArmEnc);

  if (!ok) {
    yError() << "Unable to open views";
    return false;
  }

  return true;
}

void KinThread::run() {
  yInfo() << "KinThread is running correctly ...";

 
  torsoH = getHfromEncoders<iCubTorso>(iTorsoEnc, torso);

  armH = getHfromEncoders<iCubArm>(iArmEnc, arm);


  // Read Torso Encoders
  iTorsoEnc->getEncoders(torsoEncValues.data());

  for (auto &e : torsoEncValues) {
    e = iCub::ctrl::CTRL_DEG2RAD * e;
  }

  torsoActualValues = arm.setAng(torsoEncValues);
  torsoH = arm.getH(torsoActualValues);

}
template <class T>
yarp::sig::Matrix getHfromEncoders(const IEncoders *encs, const T& limb) {
  yarp::sig::Vector encValues;
  yarp::sig::Vector actualValues;

 // Read encoders
  encs->getEncoders(encValues.data());

  // convert each element in radians
  for (auto &e : encValues) {
    e = iCub::ctrl::CTRL_DEG2RAD * e;
  }

  actualValues = limb.setAng(encValues);
  return arm.getH(actualValues);
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

bool KinModule::updateModule() {
  yInfo() << "KinModule is running correctly...";

  return true;
}