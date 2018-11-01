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

#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "subt_gazebo/SimpleDOTParser.hh"

using namespace subt;

//////////////////////////////////////////////////
bool SimpleDOTParser::Parse(std::istream &_in, VisibilityGraph &_g)
{
  std::map<std::string, ignition::math::graph::VertexId> verticesLUT;
  std::string lineread;

  this->NextRealLine(_in, lineread);

  const std::string kGraphDelim = "graph";
  if (lineread.compare(0, kGraphDelim.size(), kGraphDelim) != 0)
  {
    std::cerr << "Parsing error: graph not found" << std::endl;
    return false;
  }

  if (lineread.back() != '{')
  {
    std::cerr << "Parsing error: { not found" << std::endl;
    return false;
  }

  do
  {
    this->NextRealLine(_in, lineread);

    // Are there any attributes?
    std::string key;
    std::string value;
    if (!this->ParseAttribute(lineread, key, value))
      return false;

    // Remove leading and trailing whitespaces.
    if (!lineread.empty() && isspace(lineread.front()))
      lineread.erase(0, 1);

    if (!lineread.empty() && isspace(lineread.back()))
      lineread.pop_back();

    // Edges.
    auto edge = this->Split(lineread, "--");
    if (edge.size() > 2u)
    {
      std::cerr << "Parsing error: Only edges with two vertices are supported"
                << std::endl;
      return false;
    }

    if (edge.size() == 2u)
    {
      for (auto i = 0u; i < 2u; ++i)
      {
        // Remove leading and trailing whitespaces.
        if (!edge[i].empty() && isspace(edge[i].front()))
          edge[i].erase(0, 1);

        if (!edge[i].empty() && isspace(edge[i].back()))
          edge[i].pop_back();
      }

      if (verticesLUT.find(edge[0]) == verticesLUT.end() ||
          verticesLUT.find(edge[1]) == verticesLUT.end())
      {
        std::cerr << "Unable to find vertex while trying to create an edge.\n"
                  << " Please, declare all vertices before the edges "
                  << edge[0] << " " << edge[1]
                  << std::endl;
        return false;
      }

      auto vId1 = verticesLUT[edge[0]];
      auto vId2 = verticesLUT[edge[1]];

      _g.AddEdge({vId1, vId2}, 0);
    }

    // Vertices.
    if (edge.size() == 1 && lineread != "}")
    {
      auto newVertex = _g.AddVertex(value, "");
      verticesLUT[lineread] = newVertex.Id();
    }
  }
  while (!lineread.empty() && lineread != "}");

  return true;
}

//////////////////////////////////////////////////
void SimpleDOTParser::TrimWhitespaces(std::string &_str)
{
  // Remove comments.
  auto commentStart = _str.find("/*");
  if (commentStart != std::string::npos)
  {
    auto commentEnd = _str.rfind("*/");
    if (commentEnd != std::string::npos)
      _str.erase(commentStart, commentEnd - commentStart + 2);
  }

  // Remove consecutive whitespaces leaving only one.
  auto new_end = std::unique(_str.begin(), _str.end(),
    [](char lhs, char rhs)
    {
      return isspace(lhs) && isspace(rhs);
    });
  _str.erase(new_end, _str.end());

  // Remove leading and trailing whitespaces.
  if (!_str.empty() && isspace(_str.front()))
    _str.erase(0, 1);

  if (!_str.empty() && isspace(_str.back()))
    _str.pop_back();

  // Replace all spaces with ' '.
  std::replace_if(_str.begin(), _str.end(), ::isspace, ' ');

  // Remove trailing semicolon.
  if (_str.back() == ';')
    _str.pop_back();
}

//////////////////////////////////////////////////
std::vector<std::string> SimpleDOTParser::Split(const std::string &_str,
  const std::string &_delim)
{
  std::vector<std::string> tokens;
  char *saveptr;
  char *str = strdup(_str.c_str());

  auto token = strtok_r(str, _delim.c_str(), &saveptr);

  while (token)
  {
    tokens.push_back(token);
    token = strtok_r(nullptr, _delim.c_str(), &saveptr);
  }

  free(str);
  return tokens;
}

/// \brief ToDo.
void SimpleDOTParser::NextRealLine(std::istream &_input, std::string &_line)
{
  while (std::getline(_input, _line))
  {
    this->TrimWhitespaces(_line);

    // Ignore blank lines.
    if (!_line.empty())
      break;
  }
}

//////////////////////////////////////////////////
bool SimpleDOTParser::ParseAttribute(std::string &_str, std::string &_key,
  std::string &_value)
{
  std::string str = _str;

  // No attribute.
  auto attrStart = str.find("[");
  if (attrStart == std::string::npos)
    return true;

  auto attrEnd = str.find("]");
  if (attrEnd == std::string::npos)
  {
    std::cerr << "Parsing error: Unable to find \"]\"" << std::endl;
    return false;
  }

  str.erase(attrEnd, str.size() - attrEnd);
  str.erase(0, attrStart + 1);

  auto aPair = this->Split(str, "=");
  if (aPair.size() != 2u)
  {
    std::cerr << "Parsing error: Unable to find a single \"=\".\n"
              << "Multiple attributes are not supported." << std::endl;
    return false;
  }

  // Remove leading and trailing whitespaces.
  for (auto i = 0u ; i < aPair.size(); ++i)
  {
    if (!aPair[i].empty() && isspace(aPair[i].front()))
      aPair[i].erase(0, 1);

    if (!aPair[i].empty() && isspace(aPair[i].back()))
      aPair[i].pop_back();
  }

  _key = aPair[0];
  _value = aPair[1];

  // Erase the attribute.
  _str.erase(attrStart, attrEnd - attrStart + 1);

  return true;
}
