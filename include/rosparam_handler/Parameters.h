#ifndef _ROSPARAM_HANDLER_PARAMETERS_H_
#define _ROSPARAM_HANDLER_PARAMETERS_H_

#include <stdlib.h>
#include <string>
#include <limits>
#include <ros/param.h>
#include <ros/node_handle.h>

namespace rosparam_handler {

inline std::string getNodeName(const ros::NodeHandle& privateNodeHandle){
    std::string name_space = privateNodeHandle.getNamespace();
    std::stringstream tempString(name_space);
    std::string name;
    while(std::getline(tempString, name, '/')){;}
    return name;
}

// Forward declaration
struct ParametersBase;

namespace details
{
  template <typename C>
  void translate(ParametersBase* /*o*/, const C& /*c*/, const uint32_t /*level*/)
  {
    throw std::runtime_error("Not implemented!");
  }
}

struct ParametersBase {

  ParametersBase(const ros::NodeHandle& private_node_handle)
  : privateNamespace{private_node_handle.getNamespace() + "/"},
    nodeName{getNodeName(private_node_handle)},
    globalNamespace{"/"} {}

  virtual void fromParamServer() = 0;

  template <typename T>
  void fromConfig(T&& config, const uint32_t level = 0)
  {
    details::translate(this, std::forward<T>(config), level);
  }

  friend std::ostream& operator<<(std::ostream& os, const ParametersBase& p)
  {
    p.print(os);
    return os;
  }

  virtual void missingParamsWarning() = 0;

  template <typename T>
  void getParam(const std::string key, T& val) {
    if (!ros::param::has(key)) {
      ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
      missingParamsWarning();
      std::exit(EXIT_FAILURE);
    } else if (!ros::param::get(key, val)) {
      ROS_ERROR_STREAM("Could not retrieve parameter'" << key
                       << "'. Does it have a different type?");
      missingParamsWarning();
      std::exit(EXIT_FAILURE);
    }
  }

  template <typename T>
  void getParam(const std::string key, T& val, const T& defaultValue) {
    if (!ros::param::has(key)) {
        val = defaultValue;
    } else if (!ros::param::get(key, val)) {
        ROS_WARN_STREAM("Parameter "<<key
                        <<" is set, but has a different type. Using default value instead.");
        val = defaultValue;
  }
  }

  void testConstParam(const std::string key){
    if (ros::param::has(key)) {
      ROS_WARN_STREAM("Parameter " << key
                      << "' was set on the parameter server eventhough it was defined to be constant.");
    }
  }

  template<typename T>
  void testMin(const std::string key, T& val, T min = std::numeric_limits<T>::min()){
    if (val < min){
      ROS_WARN_STREAM("Value of " << val << " for "
                      << key << " is smaller than minimal allowed value. Correcting value to min=" << min);
      val = min;
    }
  }

  template<typename T>
  void testMax(const std::string key, T& val, T max = std::numeric_limits<T>::max()){
    if (val > max){
      ROS_WARN_STREAM("Value of " << val << " for "
                      << key << " is greater than maximal allowed. Correcting value to max=" << max);
      val = max;
    }
  }

  protected:
    const std::string privateNamespace;
    const std::string nodeName;
    const std::string globalNamespace;

    virtual std::ostream& print(std::ostream& os) const = 0;
};

using ParametersPtr = std::shared_ptr<ParametersBase>;

} // namespace parameter_handler

#endif /* _ROSPARAM_HANDLER_PARAMETERS_H_ */
