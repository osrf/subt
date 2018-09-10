/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef SUBT_GAZEBO_LIGHTVISUALPLUGIN_HH_
#define SUBT_GAZEBO_LIGHTVISUALPLUGIN_HH_

#include <memory>

#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  // forward declaration
  class LightVisualPluginPrivate;

  /// \brief A plugin that toggles light visuals
  class LightVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor
    public: LightVisualPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _parent,
        sdf::ElementPtr _sdf);

    /// \brief PreRender event callback
    public: virtual void PreRender();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<LightVisualPluginPrivate> dataPtr;
  };
}
#endif
