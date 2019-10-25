/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "subt_ign/CommonTypes.hh"

#include <ignition/common/Console.hh>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <string>

using namespace boost;
using namespace ignition;
using namespace subt;

struct vertex_label_t { typedef vertex_property_tag kind; };

using vertex_p = property<vertex_name_t, std::string,
                 property<vertex_label_t, std::string>>;

using edge_p = property<edge_name_t, std::string>;

using graph_t = adjacency_list<vecS, vecS, undirectedS, vertex_p, edge_p>;

int main(int argc, char** argv)
{
  auto filename = std::string(argv[1]);

  graph_t graph(0);
  dynamic_properties dp;

  property_map<graph_t, vertex_name_t>::type vname =
    get(vertex_name, graph);
  dp.property("node_id", vname);

  property_map<graph_t, vertex_label_t>::type vlabel =
    get(vertex_label_t(), graph);
  dp.property("label", vlabel);

  property_map<graph_t, edge_name_t>::type elabel =
    get(edge_name, graph);
  dp.property("label", elabel);

  auto ifs = std::ifstream(filename);
  bool status = read_graphviz(ifs, graph, dp, "node_id");
  ignmsg << status << std::endl;

  BOOST_FOREACH(graph_t::vertex_descriptor v, vertices(graph))
  {
    std::cout << get("node_id", dp, v) << " " << get("label", dp, v) << std::endl;
  }
}

