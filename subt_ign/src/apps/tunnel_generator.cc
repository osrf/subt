#include "world_generator_base.hh"
#include "world_generator_utils.hh"

namespace subt {

//////////////////////////////////////////////////
class TunnelGeneratorBase
{
  /// \brief Create world sections from all the individual tiles
  protected: std::vector<WorldSection> CreateWorldSections(std::map<std::string, std::vector<ignition::math::Vector3d>>
      &_tileConnectionPoints);
};

//////////////////////////////////////////////////
class TunnelGenerator : TunnelGeneratorBase, public WorldGenerator
{
  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles() override;
};

//////////////////////////////////////////////////
class TunnelGeneratorDebug : TunnelGeneratorBase, public WorldGeneratorDebug
{
  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles() override;
};
}

//////////////////////////////////////////////////
std::vector<WorldSection> TunnelGeneratorBase::CreateWorldSections(std::map<std::string, std::vector<ignition::math::Vector3d>>
      &_tileConnectionPoints)
{
  static size_t nextId = 0u;
  std::vector<WorldSection> worldSections;
  double tileSize = 10;
  for (const auto &t : _tileConnectionPoints)
  {
    if (t.first.find("Constrained") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, tileSize, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Rough") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("1") != std::string::npos || t.first.find("2") != std::string::npos ||
             t.first.find("3") != std::string::npos || t.first.find("4") != std::string::npos ||
             t.first.find("5") != std::string::npos || t.first.find("6") != std::string::npos ||
             t.first.find("7") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Elevation") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, 0, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Straight") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, 5, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Corner") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, 0, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Bend") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, 0, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }
    else if (t.first.find("Intersection") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, 0, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = NONE;
      s.id = nextId++;
      worldSections.push_back(s);
    }     
  }
  return worldSections;
}

//////////////////////////////////////////////////
void TunnelGenerator::LoadTiles()
{
  WorldGenerator::LoadTiles();
  this->worldSections = this->CreateWorldSections(this->tileConnectionPoints);
}

//////////////////////////////////////////////////
void TunnelGenerator::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond::Identity;
  op.pos += math::Vector3d(10, 0, 0);
  op.tileType = NONE;
  openings.push_back(op);
  int tileCount = this->minTileCount;

  // loop through all openings in the world, starting from the staging area,
  // and add tiles to these connection points until we reach the specified
  // tile count
  while (tileCount > 0 && !openings.empty())
  {
    auto o = openings.front();
    openings.pop_front();

    math::Vector3d connectionPoint = o.pos;
    math::Quaterniond rot = o.rot;
    TileType tileType = o.tileType;

    bool selected = false;
    WorldSection selectedSection;

    // Set max number of attempts at adding a tile to the current connection
    // point before giving up. Failure to add a tile is mostly due to
    // intersection with existing tiles in the world.
    int attempt = 0;
    int maxAttempt = 50;
    while (!selected && attempt++ < maxAttempt)
    {
      // Select the world section generated from the tile
      WorldSection s = std::move(this->SelectWorldSection(tileType));
      // Do nothing if tile count was reached
      if(tileCount == 0)
      {
        continue;
      }

      // transformation from world origin to connection point of the current
      // world section
      math::Pose3d transform(connectionPoint, rot);

      // do bounding box intersection check to prevent overlapping tiles
      if (this->IntersectionCheck(s, transform, this->addedWorldSections))
      {
        continue;
      }

      // make sure we do not add a section that has zero openings
      // if we have not reached the desired tile count yet.
      bool valid = !(openings.empty() &&
          s.connectionPoints.empty() &&
          tileCount - s.tiles.size() > 0);
      if (!valid)
        continue;

      // set the world pose of the new section
      s.pose = transform;

      // add the new section
      selected = true;
      selectedSection = s;
      this->addedWorldSections.push_back(selectedSection);

      // add the openings of the new world section to the list
      // be sure to convert to world frame
      for (auto &cp : s.connectionPoints)
      {
        ConnectionOpening op;
        op.pos = transform.CoordPositionAdd(cp.first);
        op.rot = transform.Rot() * cp.second;
        op.tileType = NONE;
        openings.push_back(op);
      }

      // update number of tiles that still need to be added
      tileCount -= s.tiles.size();
    }

    // if we fail to add a section, add to the unfilled list
    // so we can put a cap there to block the opening
    if (attempt >= maxAttempt)
    {
      unfilledOpenings.push_back(o);
    }
  }

  // add remaining openings to the unfilled list
  for (auto &o : openings)
    unfilledOpenings.push_back(o);

  // begin generating the world sdf
  std::stringstream ss;
  math::AxisAlignedBox worldBBox;
  int tileNo = 0;
  for (const auto &s : this->addedWorldSections)
  {
    for (const auto &t : s.tiles)
    {
      std::string name = "tile_" + std::to_string(++tileNo);

      // convert tile pose to world coordinates
      math::Pose3d pose = t.model.RawPose() + s.pose;
      std::string uri =
          "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/" +
          t.tileType;

      ss << "    <include>\n";
      ss << "      <static>true</static>\n";
      ss << "      <name>" << name << "</name>\n";
      ss << "      <pose>" << pose << "</pose>\n";
      ss << "      <uri>" << uri << "</uri>\n";
      ss << "    </include>\n\n";
    }

    // update world bbox info
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  std::string uri = capUri + "Tunnel Tile Blocker";

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);
    // TODO Verify where the cap needs to be placed
    // Offset tile based on connection tile
    math::Pose3d pose = math::Pose3d(pos + rot*math::Vector3d(2.5, 0, 0), 
                      math::Quaterniond(0, 0, IGN_PI/2)*rot);

    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>" << name << "</name>\n";
    ss << "      <pose>" << pose << "</pose>\n";
    ss << "      <uri>" << uri << "</uri>\n";
    ss << "    </include>\n\n";
  }

  // assemble the sdf string
  std::string worldStr;
  worldStr += WorldTopStr(this->worldType, this->outputFile, this->worldName, this->gui);
  if (this->gui)
    worldStr += this->WorldGUIStr();
  worldStr += ss.str();
  worldStr += this->WorldBottomStr();

  // output to file
  std::ofstream outFile(this->outputFile);
  if (!outFile.is_open())
  {
    std::cerr << "Failed to write to file " << this->outputFile << std::endl;
    return;
  }
  outFile << worldStr;
  outFile.close();

  // print out some stats
  std::cout << "World Generated: " << this->outputFile << std::endl;
  std::cout << "  Total Tile count: " << tileNo << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl;
}

//////////////////////////////////////////////////
void TunnelGeneratorDebug::LoadTiles()
{
  WorldGeneratorDebug::LoadTiles();
  this->worldSections = this->CreateWorldSections(this->tileConnectionPoints);
}

//////////////////////////////////////////////////
void TunnelGeneratorDebug::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;
  std::vector<WorldSection> addedWorldSections;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond::Identity;
  op.pos += math::Vector3d(10, 0, 0);
  op.tileType = NONE;
  openings.push_back(op);
  int tileCount = 1;

  // loop through all openings in the world, starting from the staging area,
  // and add tiles to these connection points until we reach the specified
  // tile count
  while (tileCount > 0 && !openings.empty())
  {
    auto o = openings.front();
    openings.pop_front();

    math::Vector3d connectionPoint = o.pos;
    math::Quaterniond rot = o.rot;
    TileType tileType = o.tileType;

    bool selected = false;
    WorldSection selectedSection;

    // Set max number of attempts at adding a tile to the current connection
    // point before giving up. Failure to add a tile is mostly due to
    // intersection with existing tiles in the world.
    int attempt = 0;
    int maxAttempt = 2;
    while (!selected && attempt++ < maxAttempt)
    {
      // Select the world section generated from the tile
      WorldSection s = std::move(this->SelectWorldSection(tileType));
      // Do nothing if tile count was reached
      if(tileCount == 0)
      {
        continue;
      }

      // transformation from world origin to connection point of the current
      // world section
      math::Pose3d transform(connectionPoint, rot);

      // do bounding box intersection check to prevent overlapping tiles
      if (this->IntersectionCheck(s, transform, addedWorldSections))
      {
        continue;
      }

      // make sure we do not add a section that has zero openings
      // if we have not reached the desired tile count yet.
      bool valid = !(openings.empty() &&
          s.connectionPoints.empty() &&
          tileCount - s.tiles.size() > 0);
      if (!valid)
        continue;

      // set the world pose of the new section
      s.pose = transform;

      // add the new section
      selected = true;
      selectedSection = s;
      addedWorldSections.push_back(selectedSection);

      // add the openings of the new world section to the list
      // be sure to convert to world frame
      for (auto &cp : s.connectionPoints)
      {
        ConnectionOpening op;
        op.pos = transform.CoordPositionAdd(cp.first);
        op.rot = transform.Rot() * cp.second;
        op.tileType = NONE;
        openings.push_back(op);
      }

      // update number of tiles that still need to be added
      tileCount -= s.tiles.size();
    }

    // if we fail to add a section, add to the unfilled list
    // so we can put a cap there to block the opening
    if (attempt >= maxAttempt)
    {
      unfilledOpenings.push_back(o);
    }
  }

  // add remaining openings to the unfilled list
  for (auto &o : openings)
    unfilledOpenings.push_back(o);

  // begin generating the world sdf
  std::stringstream ss;
  math::AxisAlignedBox worldBBox;
  int tileNo = 0;
  for (const auto &s : addedWorldSections)
  {
    for (const auto &t : s.tiles)
    {
      std::string name = "tile_" + std::to_string(++tileNo);

      // convert tile pose to world coordinates
      math::Pose3d pose = t.model.RawPose() + s.pose;
      std::string uri =
          "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/" +
          t.tileType;

      ss << "    <include>\n";
      ss << "      <static>true</static>\n";
      ss << "      <name>" << name << "</name>\n";
      ss << "      <pose>" << pose << "</pose>\n";
      ss << "      <uri>" << uri << "</uri>\n";
      ss << "    </include>\n\n";
    }

    // update world bbox info
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  std::string uri = capUri + "Tunnel Tile Blocker";

  math::Pose3d pose;

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);

    // Adjust cap offset
    if (this->tileName.find("Rough") != std::string::npos)
    {
      pose = math::Pose3d(pos + rot*math::Vector3d(2.5, 0, 0),
           math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    else if (this->tileName.find("Straight") != std::string::npos ||
        this->tileName.find("Constrained") != std::string::npos)
    {
          pose = math::Pose3d(pos + rot*math::Vector3d(2.5, 0, 0),
            math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    else if (this->tileName.find("Bend Right") != std::string::npos)
    {
      pose = math::Pose3d(pos + rot*math::Vector3d(-2.5, 2.5, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    else if (this->tileName.find("Corner") != std::string::npos)
    {
      pose = math::Pose3d(pos + rot*math::Vector3d(2.5, 0, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    else if (this->tileName.find("Elevation") != std::string::npos)
    {
      pose = math::Pose3d(pos + rot*math::Vector3d(2.5, 0, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }
    else
    {
      pose = math::Pose3d(pos, math::Quaterniond(0, 0, IGN_PI/2)*rot);
    }

    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>" << name << "</name>\n";
    ss << "      <pose>" << pose << "</pose>\n";
    ss << "      <uri>" << uri << "</uri>\n";
    ss << "    </include>\n\n";
  }

  // assemble the sdf string
  std::string worldStr;
  worldStr += WorldTopStr(this->worldType, this->outputFile, this->worldName, this->gui);
  if (this->gui)
    worldStr += this->WorldGUIStr();
  worldStr += ss.str();
  worldStr += this->WorldBottomStr();

  // output to file
  std::ofstream outFile(this->outputFile);
  if (!outFile.is_open())
  {
    std::cerr << "Failed to write to file " << this->outputFile << std::endl;
    return;
  }
  outFile << worldStr;
  outFile.close();

  // print out some stats
  std::cout << "World Generated: " << this->outputFile << std::endl;
  std::cout << "  Total Tile count: " << tileNo << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl;
}

//////////////////////////////////////////////////
void printUsage()
{
  std::string usage;
  usage += "Usage: tunnel_generator [options]\n";
  usage += "Example Generate: tunnel_generator -g -c 200 -n tunnel_test -s 11 -o tunnel_test.sdf\n";
  usage += "Example Debug: tunnel_generator -g -d \"Tunnel Straight\" -n tunnel_straight -o tunnel_straight.sdf\n";
  usage += "Options:\n";
  usage += "    -h\t\t Print this help message\n";
  usage += "    -d <tile>\t Generate world from tile for debugging\n";
  usage += "    -o <file>\t Output sdf filename\n";
  usage += "    -s <seed>\t Seed\n";
  usage += "    -c <count>\t Min tile count\n";
  usage += "    -n <name>\t World name\n";
  usage += "    -g\t\t Generate sdf with GUI plugin\n";
  std::cout << usage << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printUsage();
    return -1;
  }

  int opt;
  std::string output;
  std::string worldName;
  std::string tileName;
  std::string worldType = "Tunnel";
  int seed = 0;
  bool gui = false;
  bool debug = false;
  int tileCount = 10;
  while((opt = getopt(argc, argv, "o:s:c:n:d:hg")) != -1)
  {
    switch(opt)
    {
      // debug tile
      case 'd':
      {
        debug = true;
        tileName = optarg;
        std::cout << "Tile name:\t" << tileName << std::endl;
        break;
      }
      // output
      case 'o':
      {
        output = optarg;
        std::cout << "Filename:\t" << output << std::endl;
        break;
      }
      // seed
      case 's':
      {
        seed = std::stoi(optarg);
        break;
      }
      // min tile count
      case 'c':
      {
        tileCount = std::stoi(optarg);
        break;
      }
      // world name
      case 'n':
      {
        worldName = optarg;
        std::cout << "World name:\t" << worldName << std::endl;
        break;
      }
      // help
      case 'h':
      {
        printUsage();
        return -1;
      }
      // enable gui?
      case 'g':
      {
        gui = true;
        break;
      }
      default:
        printUsage();
        return -1;
    }
  }

  srand(seed);

  if(debug)
  {
    TunnelGeneratorDebug tgdb;

    tgdb.SetTileName(tileName);
    tgdb.SetOutputFile(output);
    tgdb.SetEnableGUI(gui);
    tgdb.SetWorldName(worldName);
    tgdb.SetWorldType(worldType);
    
    tgdb.Generate();
  } 
  else
  {
    TunnelGenerator tg;

    tg.SetOutputFile(output);
    tg.SetEnableGUI(gui);
    tg.SetWorldName(worldName);
    tg.SetWorldType(worldType);
    tg.SetSeed(seed);
    tg.SetMinTileCount(tileCount);
    
    tg.Generate();
  }

  return 0;
}