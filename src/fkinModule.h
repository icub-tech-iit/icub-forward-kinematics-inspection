/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <iostream>

class KinThread : public yarp::os::PeriodicThread {

 public:
  KinThread(double period);
  ~KinThread();

  bool threadInit() override;
  void run() override;
  void threadRelease() override;

  yarp::dev::IEncoders *iArmEnc;
  yarp::dev::IEncoders *iTorsoEnc;
  yarp::dev::PolyDriver driverArm;
  yarp::dev::PolyDriver driverTorso;

 protected:
  iCub::iKin::iCubArm arm;
  yarp::sig::Vector armEncValues;
  yarp::sig::Vector torsoEncValues;
};


class KinModule : public yarp::os::RFModule {
 public:
  KinModule();
  ~KinModule();

  bool configure(yarp::os::ResourceFinder &rf) override;
  bool close() override;
  double getPeriod() override;
  bool updateModule() override;

 private:
  std::unique_ptr<KinThread> thr;
};