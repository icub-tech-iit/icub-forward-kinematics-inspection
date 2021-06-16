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

 public:
  KinThread(double period);
  ~KinThread();

  bool threadInit();
  void run();

  template <class T>
  yarp::sig::Matrix getHfromEncoders(IEncoders *encs, T& limb) {
    yarp::sig::Vector encValues;
    yarp::sig::Vector actualValues;

    // Read encoders
    encs->getEncoders(encValues.data());

    // convert each element in radians
    for (auto &e : encValues) {
      e = iCub::ctrl::CTRL_DEG2RAD * e;
    }

    actualValues = limb.setAng(encValues);
    return limb.getH(actualValues);
 }

  IEncoders *iArmEnc;
  IEncoders *iTorsoEnc;
  PolyDriver driverArm;
  PolyDriver driverTorso;

 protected:
  iCubArm arm;
  yarp::sig::Vector armEncValues;
  yarp::sig::Vector armActualValues;
  yarp::sig::Matrix armH;

  iCubTorso torso;
  yarp::sig::Vector torsoEncValues;
  yarp::sig::Vector torsoActualValues;
  yarp::sig::Matrix torsoH;
};


class KinModule : public RFModule {
 public:
  KinModule();
  ~KinModule();

  bool configure(yarp::os::ResourceFinder &rf);
  bool close();
  bool updateModule();

 private:
  std::unique_ptr<KinThread> thr;
};