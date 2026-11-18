// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__HARDWARE__HARDWARE_PARAMETERS_HPP_
#define ONROBOT_DRIVER__HARDWARE__HARDWARE_PARAMETERS_HPP_

#include <stdexcept>
#include <string>

#include "hardware_interface/hardware_info.hpp"

namespace onrobot_driver
{

/// A <param> the plugin needs is missing from the <ros2_control> block of the URDF, or cannot be
/// read.
class ParameterError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Reads the <param> values of a <ros2_control> block. Every value is required: the plugins hold no
/// fallback, so a block that lacks one stops the hardware with a message that names it.
class HardwareParameters
{
public:
  explicit HardwareParameters(const hardware_interface::HardwareInfo & info)
  : info_(info) {}

  std::string text(const std::string & name) const
  {
    const auto found = info_.hardware_parameters.find(name);
    if (found == info_.hardware_parameters.end()) {
      throw ParameterError(described(name) + " is missing in the <ros2_control> block of the URDF");
    }
    return found->second;
  }

  double number(const std::string & name) const
  {
    const std::string value = text(name);
    try {
      size_t used = 0;
      const double parsed = std::stod(value, &used);
      if (used == value.size()) {
        return parsed;
      }
    } catch (const std::exception &) {
    }
    throw ParameterError(described(name) + " is not a number: '" + value + "'");
  }

  int integer(const std::string & name) const
  {
    const double value = number(name);
    if (value != static_cast<double>(static_cast<int>(value))) {
      throw ParameterError(described(name) + " is not an integer: '" + text(name) + "'");
    }
    return static_cast<int>(value);
  }

  bool flag(const std::string & name) const
  {
    const std::string value = text(name);
    if (value == "true" || value == "True" || value == "1") {
      return true;
    }
    if (value == "false" || value == "False" || value == "0") {
      return false;
    }
    throw ParameterError(described(name) + " is not true or false: '" + value + "'");
  }

private:
  /// Start of every error message: which parameter of which hardware.
  std::string described(const std::string & name) const
  {
    return "[Fail-Fast] Hardware parameter '" + name + "' of '" + info_.name + "'";
  }

  const hardware_interface::HardwareInfo & info_;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__HARDWARE__HARDWARE_PARAMETERS_HPP_
