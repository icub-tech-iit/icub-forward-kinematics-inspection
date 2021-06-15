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
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <iostream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace iCub::iKin;

class KinThread : public PeriodicThread {
 protected:
  PolyDriver jointsClient;
  IEncoders *iEnc;

 public:
  KinThread() : PeriodicThread(0.01) {}
  bool threadInit() { 
            return true; }
  void run() { }
};

class KinModule : public RFModule {
 protected:
  KinThread thr;

 public:
  bool configure(yarp::os::ResourceFinder &rf) {
    thr.setPeriod(0.01);

    return thr.start();
  }

  bool close() {
    thr.stop();
    return true;
  }

  double getPeriod() { return 0.01; }

  bool updateModule() {
    yInfo() << "KinModule is running correctly ...";
    return true;
  }
};

int main(int argc, char *argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "YARP doesn't seem to be available";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.configure(argc, argv);

  KinModule mod;

  return mod.runModule();
}
