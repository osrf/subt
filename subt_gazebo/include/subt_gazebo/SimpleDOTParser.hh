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

#include <istream>
#include <string>
#include <vector>
#include <ignition/math/graph/Graph.hh>

#include "subt_gazebo/CommonTypes.hh"

#ifndef SUBT_GAZEBO_SIMPLEDOTPARSER_HH_
#define SUBT_GAZEBO_SIMPLEDOTPARSER_HH_

namespace subt
{
  /// \brief A simple parser for graphs in DOT format. Note that this parser
  /// doesn't implement the entire DOT spec. It only parses the relevant
  /// features needed for SubT.
  /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
  class SimpleDOTParser
  {
    /// \brief Remove comments, consecutive whitespaces, leading and trailing
    /// whitespaces and remove the trailing semicolon (if any).
    /// \param[in, out] _str The string to be parsed and converted.
    public: void TrimWhitespaces(std::string &_str);

    /// \brief Split the input string using a delimiter.
    /// \param[in] _str The input string.
    /// \param[in] _delim The delimiter.
    /// \return A vector containing the different substrings. If the delimiter
    /// is not found, the result will contain the input string.
    public: std::vector<std::string> Split(const std::string &_str,
                                           const std::string &_delim);

    /// \brief Given an input stream, gets the next real line to be parsed.
    /// A real line is considered when there's something to be parsed.
    /// E.g.: An empy new line is not a real line.
    public: void NextRealLine(std::istream &_input,
                              std::string &_line);
    
    /// \brief Parse DOT attributes from an input string.
    /// \param[in, out] _str The input string. Note that the attributes are
    /// removed from the input string after being parsed.
    /// \param[out] _key If present, the key attribute (e.g.: label).
    /// \param[out] _value If present, the value attribute (e.g.: "my_vertex").
    /// \return True when there was no attribute or the attribute was
    /// succesfully parsed. False when the attribute wasn't parsed.
    public: bool ParseAttribute(std::string &_str,
                                std::string &_key,
                                std::string &_value);
  };

  /// \brief Stream extraction operator.
  /// \param[in] _in An input stream.
  /// \param[out] _g The visibility graph.
  std::istream &operator>>(std::istream &_in,
                           VisibilityGraph &_g);
}  // namespace
#endif
