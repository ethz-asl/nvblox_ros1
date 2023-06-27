// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>

#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>

#include <nvblox_msgs/Mesh.h>

#include "nvblox_rviz_plugin/nvblox_mesh_visual.h"

namespace nvblox_rviz_plugin {

class NvbloxMeshVisual;

class NvbloxMeshDisplay : public rviz::MessageFilterDisplay<nvblox_msgs::Mesh> {
  Q_OBJECT
 public:
  NvbloxMeshDisplay();
  virtual ~NvbloxMeshDisplay();

 public Q_SLOTS:
  virtual void updateCeilingOptions();
 public Q_SLOTS:
  virtual void updateMeshColorOptions();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
 void processMessage(const nvblox_msgs::Mesh::ConstPtr& msg) override;

  rviz::BoolProperty* cut_ceiling_property_;
  rviz::FloatProperty* ceiling_height_property_;
  rviz::EnumProperty* mesh_color_property_;

  std::unique_ptr<NvbloxMeshVisual> visual_;
};

}  // namespace nvblox_rviz_plugin
