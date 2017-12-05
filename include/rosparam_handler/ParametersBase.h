/**
 * \file ParametersBase.h
 *
 *  Created on: Oct 28, 2017
 *
 *  \authors: Claudio Bandera
 *            Sascha Wirges
 *            Niels Ole Salscheider
 *            Jeremie Deray
 */

#ifndef _ROSPARAM_HANDLER_PARAMETERS_BASE_H_
#define _ROSPARAM_HANDLER_PARAMETERS_BASE_H_

#include <ros/param.h>
#include <ros/node_handle.h>
#include <rosparam_handler/utilities.hpp>
#include <rosparam_handler/pointer.hpp>

namespace rosparam_handler {

/// \brief ParametersBase An abstract base struct for struct generated by rosparam_handler.
struct ParametersBase {

  ParametersBase(const ros::NodeHandle& private_node_handle)
  : globalNamespace{"/"},
    privateNamespace{private_node_handle.getNamespace() + "/"},
    nodeName{rosparam_handler::getNodeName(private_node_handle)} {}

  virtual ~ParametersBase() = default;

  /// \brief Get values from parameter server
  ///
  /// Will fail if a value can not be found and no default value is given.
  inline void fromParamServer(){
    if(!fromParamServerImpl()){
      missingParamsWarning();
      rosparam_handler::exit("RosparamHandler: GetParam could net retrieve parameter.");
    }
    ROS_DEBUG_STREAM(*this);
  }

  /// \brief Set parameters on ROS parameter server.
  virtual void toParamServer() = 0;

  /// \brief Update configurable parameters.
  ///
  /// \param config  dynamic reconfigure struct
  /// \level ?
  template <typename T>
  inline void fromConfig(const T& config, const uint32_t level = 0){
#ifdef DYNAMIC_RECONFIGURE_FOUND
  fromConfigImpl<T>(config, level);
#else
  ROS_FATAL_STREAM("dynamic_reconfigure was not found during compilation. So fromConfig() is not available. Please recompile with dynamic_reconfigure.");
  rosparam_handler::exit("dynamic_reconfigure was not found during compilation. So fromConfig() is not available. Please recompile with dynamic_reconfigure.");
#endif
  }

  /// \brief Stream operator for printing parameter struct
  inline friend std::ostream& operator<<(std::ostream& os, const ParametersBase& p)
  {
    return p.print(os);
  }

protected:

  /// \brief The actual implementation of 'fromParamServer'
  /// overriden by the derived class
  virtual bool fromParamServerImpl() = 0;

  /// \brief The actual implementation os 'fromConfig' specialized
  /// for the 'DerivedConfig' type.
  template <typename T>
  void fromConfigImpl(const T& config, const uint32_t level);

  /// \brief A helper function for ostram operator <<
  virtual std::ostream& print(std::ostream& os) const = 0;

  /// \brief Issue a warning about missing default parameters.
  virtual void missingParamsWarning() = 0;

  std::string globalNamespace = {"/"};
  std::string privateNamespace;
  std::string nodeName;
};

} // namespace rosparam_handler

#endif /* _ROSPARAM_HANDLER_PARAMETERS_BASE_H_ */
