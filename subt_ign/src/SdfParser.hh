/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "ConnectionHelper.hh"

namespace subt
{

class SdfParser
{
  /// \brief Parse contents of an sdf element
  /// \param[in] _key SDF element key
  /// \param[in] _str String content of the sdf element
  /// \param[out] _endPos end position of the sdf element string
  /// \return Value for the input key
  public: static std::string Parse(const std::string &_key, const std::string &_str,
       size_t &_endPos);

  /// \brief Parse contents of an sdf element
  /// \param[in] _key SDF element key
  /// \param[in] _str String content of the sdf element
  /// \return Value for the input key
  public: static std::string Parse(const std::string &_key,
      const std::string &_str);

  /// \brief Fill VertexData from string
  /// \param[in] _includeStr input <include> string
  /// \param[out] _vd Vertex data to be filled
  /// \return True if vertex data is successfully filled, false otherwise
  public: static bool FillVertexData(const std::string &_includeStr,
    VertexData &_vd,
    std::function<bool(const std::string &, const std::string &)> &_filter);
};
}
