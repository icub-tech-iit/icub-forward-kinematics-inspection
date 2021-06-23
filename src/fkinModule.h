/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Link.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <iostream>
#include <vector>

/**
 * @brief 
 * 
 */
class KinThread : public yarp::os::PeriodicThread {

 public:
 /**
  * @brief Construct a new Kin Thread object. Releases the torso links and sets
  * all the constraints to false.
  * 
  * @param period The thread spinning period.
  */
  KinThread(double period, const std::string& modelPath, const yarp::sig::Vector& joints);

  /**
   * @brief Default destructor of the KinThread object.
   */
  ~KinThread();

/**
 * @brief Initialises the KinThread by opening the ports 
 * associated to torso and limb. Upon opening, it resizes the encoder vectors.
 * @return true on success, false otherwise.
 */
  bool threadInit() override;
  void run() override;
  void threadRelease() override;

 protected:
  yarp::dev::IEncoders *iArmEnc;
  yarp::dev::IEncoders *iTorsoEnc;
  yarp::dev::PolyDriver driverArm;
  yarp::dev::PolyDriver driverTorso;
  yarp::sig::Vector armEncValues;
  yarp::sig::Vector torsoEncValues;
  iCub::iKin::iCubArm arm;
  iCub::iKin::iKinChain *armChain;
  yarp::os::Property armProperties;
  iDynTree::KinDynComputations kinDynCompute;
  iDynTree::Model model;
  iDynTree::VectorDynSize dynEncValues;

  std::string modelPath;
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