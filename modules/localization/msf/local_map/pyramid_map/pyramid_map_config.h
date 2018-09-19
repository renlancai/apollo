/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_PYRAMID_MAP_CONFIG_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_PYRAMID_MAP_CONFIG_H

#include "modules/localization/msf/local_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

class PyramidMapConfig : public BaseMapConfig {
 public:
  PyramidMapConfig(const std::string &map_version);
  ~PyramidMapConfig();

  bool has_intensity_ = true;
  bool has_intensity_var_ = true;
  bool has_altitude_ = true;
  bool has_altitude_var_ = true;
  bool has_ground_altitude_ = true;
  bool has_count_ = true;
  bool has_ground_count_ = true;
  unsigned int resolution_num_ = 1; 
  unsigned int resolution_ratio_ = 2;
  /**@brief coordinate type. */
  std::string coordinate_type_;
  /**@brief During the visualization (for example, Call the function get_image()
   * of map node layer), the maximum intensity value in the image. */
  float max_intensity_value_;

  /**@brief During the visualization (for example, Call the function get_image()
   * of map node layer), the maximum intensity value in the image. */
  float max_intensity_var_value_;

 protected:
    /**@brief Create the XML structure. */
    virtual bool CreateXml(boost::property_tree::ptree* config) const;
    /**@brief Load the map options from a XML structure. */
    virtual bool LoadXml(const boost::property_tree::ptree& config); 
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif //MODULES_LOCALIZATION_MSF_LOCAL_MAP_PYRAMID_MAP_CONFIG_H