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

KinThread::KinThread() : PeriodicThread(0.01), arm("left"), encodersValues() {
  yInfo() << "KinThread constructor";
}

KinThread::~KinThread() {}

bool KinThread::threadInit() { return true; }

void KinThread::run() {
  yInfo() << "KinThread is running correctly ...";

  iEnc->getEncoders(encodersValues.data());
}

KinModule::KinModule() : RFModule() {}
KinModule::~KinModule() {}

bool KinModule::configure(yarp::os::ResourceFinder &rf) {
  yInfo() << "KinModule configure";

  thr = std::make_unique<KinThread>();
  thr->setPeriod(0.01);

  Property optArm;
  optArm.put("device", "remote_controlboard");
  optArm.put("remote", "/icubSim/left_arm");
  optArm.put("local", "/logger");

  if (!thr->jointsClient.open(optArm)) {
    yError() << "Unable to connect to /icubSim/left_arm";
    return false;
  }

  // open views
/*   bool ok = true;
  ok = ok && thr->jointsClient.view(ienc);

  if (!ok) {
    yError() << "Unable to open views";
    return false;
  } */

  return thr->start();
}

bool KinModule::close() {
  thr->stop();
  return true;
}

double KinModule::getPeriod() { return 0.01; }

bool KinModule::updateModule() {
  yInfo() << "KinModule is running correctly ...";
  return true;
}