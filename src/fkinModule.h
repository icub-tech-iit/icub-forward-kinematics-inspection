/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef FKINMODULE_H_
#define FKINMODULE_H_

#include <iCub/iKin/iKinFwd.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Link.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

class KinModule {
 public:
  /**
   * @brief Construct a new KinModule object
   *
   * @param axesListValues List of axes used by iDynTree to construct the
   *                       kinematic chain.
   */
  explicit KinModule(const std::vector<std::string>& axesListValues);

  /**
   * @brief Destroy the Kin Module object
   *
   */
  ~KinModule();

  /**
   * @brief Configures the data structures used by the class, by loading the
   *        command line arguments via ResourceFinder.
   *
   * @param rf Object used to parse command line arguments.
   * @return true if successful, false otherwise.
   */
  bool configure(const yarp::os::ResourceFinder& rf);
  void evaluateKinematics(const std::string& rootFrame,
                          const std::string& endFrame);

 private:
  /**
   * @brief Loads the iDynTree kinematics model from the input urdf file, by
   *        populating the class member \c kinDynCompute.
   *
   * @param modelPath file path of the desired urdf model
   * @return true if successful, false otherwise.
   */
  bool loadIDynTreeKinematicsFromUrdf(const std::string& modelPath);

  /** @brief iKin arm object */
  iCub::iKin::iCubArm arm;
  /** @brief Pointer to the generic kinematic chain of the arm. Used to retrieve
   *         the arm kinematic properties.
   */
  iCub::iKin::iKinChain* armChain;
  /** @brief Container for the arm properties. */
  yarp::os::Property armProperties;
  /** @brief iDynTree object used to compute forward kinematics */
  iDynTree::KinDynComputations kinDynCompute;
  /** @brief Joints values expressed in [deg].  */
  yarp::sig::Vector jointsValues;
  /** @brief List of axes used by iDynTree to construct the kinematic chain. */
  std::vector<std::string> axesList;
};

#endif  // FKINMODULE_H_
