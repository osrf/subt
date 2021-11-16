#include "world_generator_base.hh"
#include "world_generator_utils.hh"

namespace subt {

//////////////////////////////////////////////////
class UrbanGeneratorBase
{
  /// \brief Create world sections from tiles that are for a subway environment
  protected: void CreateUrbanSubwayWorldSections(std::vector<WorldSection> &_worldSections,
      std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints);

  /// \brief Create world sections from tiles that are for a building environment
  protected: void CreateUrbanBuildingWorldSections(std::vector<WorldSection> &_worldSections,
      std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints);

  /// \brief Create world sections that connect building and subway environments
  protected: void CreateUrbanMixedWorldSections(std::vector<WorldSection> &_worldSections,
      std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints);

  /// \brief internal function for determining opening tileType for URBAN_MS tile openings
  /// \param[in] _transform transform from body to world frame
  /// \param[in] _openings list of current world openings
  /// \param[in] _s newly added WorldSection
  protected: void SetConnectionOpeningTypes(math::Pose3d &_transform, std::list<ConnectionOpening> &_openings, WorldSection &_s);
};

//////////////////////////////////////////////////
class UrbanGenerator : UrbanGeneratorBase, public WorldGenerator
{
  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles() override;

  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  private: WorldSection SelectWorldSection(TileType &_tileType) override;

  /// \brief vector of World section with type URBAN_MS 
  private: std::vector<WorldSection> transitionWorldSections;
};

//////////////////////////////////////////////////
class UrbanGeneratorDebug : UrbanGeneratorBase, public WorldGeneratorDebug
{
  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles() override;

  /// \brief Create a world section from transitiion tile
  private: void CreateTransitionWorldSection();

  /// \brief Randomly select a world section from the collection of prefab
  /// world sections
  private: WorldSection SelectWorldSection(TileType &_tileType) override;

  /// \brief World section with type URBAN_MS
  /// The transition tile is fixed to be the "Urban Straight Door Left"
  private: WorldSection transitionWorldSection;

  // \brief determines tileType from tile name
  private: TileType tileTypeFromName();
};
}

//////////////////////////////////////////////////
void UrbanGeneratorBase::CreateUrbanSubwayWorldSections(std::vector<WorldSection> &_worldSections,
    std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints)
{
  static size_t nextId = _worldSections.size();
  double tileSize = 20;
  for (const auto &t : _tileConnectionPoints)
  {
    if (t.first.find("Straight") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Bend") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Superpose") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("3 Way") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Elevation Down") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, tileSize, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Elevation Up") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_S;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
  }
}

void UrbanGeneratorBase::CreateUrbanBuildingWorldSections(std::vector<WorldSection> &_worldSections,
    std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints)
{
  static size_t nextId = _worldSections.size();
  double tileSize = 20;
  for (const auto &t : _tileConnectionPoints)
  {
  // Might have to build these manually
    if (t.first.find("2 Story") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_B;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Large Room") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_B;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Room Centered") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, tileSize, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_B;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Room Straight") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, tileSize, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_B;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Platform Centered") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, -tileSize, 0),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_B;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
  }
}

//////////////////////////////////////////////////
void UrbanGeneratorBase::CreateUrbanMixedWorldSections(std::vector<WorldSection> &_worldSections,
    std::map<std::string, std::vector<ignition::math::Vector3d>> &_tileConnectionPoints)
{
  static size_t nextId = _worldSections.size();
  double tileSize = 20;
  for (const auto &t : _tileConnectionPoints)
  {
    if (t.first.find("Door Right Extension") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(0, tileSize, 0),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Door Right") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(-16.021, 3.94, 0.94),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Door Left") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(-16.021, -3.94, 0.94),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Urban Platform") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(23.979, 3.906, 0.94),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
    else if (t.first.find("Urban Platform Open") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(23.979, 3.906, 0.919),
        math::Quaterniond(0, 0, IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      _worldSections.push_back(s);
    }
  }
}

//////////////////////////////////////////////////
void UrbanGeneratorBase::SetConnectionOpeningTypes(math::Pose3d &_transform, std::list<ConnectionOpening> &_openings, WorldSection &_s)
{
  // add the openings of the new world section to the list
  // be sure to convert to world frame
  std::string tileName = "";
  if (_s.tileType == URBAN_MS)
    tileName = _s.tiles[0].tileName; 
  for (uint i = 0; i < _s.connectionPoints.size() - 1; i++)
  {
    auto pose = _s.connectionPoints[i];
    ConnectionOpening op;
    op.pos = _transform.CoordPositionAdd(pose.first);
    op.rot = _transform.Rot() * pose.second;
    
    // adjust tile types for mix structure tiles
    if (_s.tileType == URBAN_MS)
    {
      if (tileName.find("Door") != std::string::npos)
      {
        if (tileName.find("Extension") != std::string::npos)
        {
          if (pose.first.X() == 0.0 && pose.first.Y() != 0.0 && pose.first.Z() == 0.0)
            op.tileType = URBAN_B;
          else
            op.tileType = URBAN_S;
        }
        if (pose.first.X() != 0.0 && pose.first.Y() != 0.0 && pose.first.Z() != 0.0)
          op.tileType = URBAN_B;
        else
          op.tileType = URBAN_S;  
      }
      else if (tileName.find("Platform") != std::string::npos)
      {
        if (pose.first.Y() != 0.0 || pose.first.Z() != 0.0)
          op.tileType = URBAN_B;
        else
          op.tileType = URBAN_S;
      }
    }
    else
    {
      op.tileType = _s.tileType;
    }

    _openings.push_back(op);
  }
}

//////////////////////////////////////////////////
void UrbanGenerator::LoadTiles()
{
  WorldGenerator::LoadTiles();
  if (this->subWorldType == URBAN_SUBWAY)
  {
    this->CreateUrbanMixedWorldSections(this->transitionWorldSections, this->tileConnectionPoints);
    this->CreateUrbanSubwayWorldSections(this->worldSections, this->tileConnectionPoints);
  }
  else if (this->subWorldType == URBAN_BUILDING)
  {
    this->CreateUrbanBuildingWorldSections(this->worldSections, this->tileConnectionPoints);
  }
  // Must be mixed
  else
  {
    this->CreateUrbanMixedWorldSections(this->transitionWorldSections, this->tileConnectionPoints);
    this->CreateUrbanBuildingWorldSections(this->worldSections, this->tileConnectionPoints);
    this->CreateUrbanSubwayWorldSections(this->worldSections, this->tileConnectionPoints);
  }
}

//////////////////////////////////////////////////
WorldSection UrbanGenerator::SelectWorldSection(TileType &_tileType)
{
  if (this->addedWorldSections.empty())
  {
    // Select random transition tile from start
    int r = rand() % this->transitionWorldSections.size();
    return this->transitionWorldSections[r];
  }
  if (this->subWorldType == URBAN_MIXED_STRUCTURE)
  {
    // have a 20 % chance of choosing a transition world
    if (rand() % 10 + 1 > 8)
    {
      int r = rand() % this->transitionWorldSections.size();
      return this->transitionWorldSections[r];
    }
    return WorldGenerator::SelectWorldSection(_tileType);
  }
  else
  {
    return WorldGenerator::SelectWorldSection(_tileType);
  }
}

//////////////////////////////////////////////////
void UrbanGenerator::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond(0, 0, -IGN_PI/2);
  op.pos += math::Vector3d(-16.021, 3.94, 0.919);
  op.tileType = URBAN_B;
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
    int maxAttempt = 20;
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
      this->SetConnectionOpeningTypes(transform, openings, s);

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
    math::Pose3d pose = math::Pose3d(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);

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
void UrbanGeneratorDebug::CreateTransitionWorldSection()
{
  static size_t nextId = 0u;
  for (const auto &t : this->tileConnectionPoints)
  {
    if (t.first.find("Straight Door Left") != std::string::npos)
    {
      WorldSection s = std::move(
        CreateWorldSectionFromTile(t.first,
        math::Vector3d(-16.021, -3.94, 0.94),
        math::Quaterniond(0, 0, -IGN_PI/2),
        NONE));
      s.tileType = URBAN_MS;
      s.id = nextId++;
      this->transitionWorldSection = s;
    }
  }
}

//////////////////////////////////////////////////
void UrbanGeneratorDebug::LoadTiles()
{
  WorldGeneratorDebug::LoadTiles();
  this->CreateTransitionWorldSection();
  this->CreateUrbanMixedWorldSections(this->worldSections, this->tileConnectionPoints);
  this->CreateUrbanBuildingWorldSections(this->worldSections, this->tileConnectionPoints);
  this->CreateUrbanSubwayWorldSections(this->worldSections, this->tileConnectionPoints);
}

//////////////////////////////////////////////////
TileType UrbanGeneratorDebug::tileTypeFromName()
{

  if (this->tileName.find("Door") != std::string::npos)
    return TileType::URBAN_MS;
  else if (this->tileName.find("Room") != std::string::npos)
    return TileType::URBAN_B;
  else if (this->tileName.find("Stairwell") != std::string::npos)
    return TileType::URBAN_B;
  else if (this->tileName.find("Elevation") != std::string::npos)
    return TileType::URBAN_S;
  else if (this->tileName.find("Bend") != std::string::npos)
    return TileType::URBAN_S;
  else if (this->tileName.find("Urban Straight") != std::string::npos)
    return TileType::URBAN_S;
  else if (this->tileName.find("2 Story") != std::string::npos)
    return TileType::URBAN_B;
  else if (this->tileName.find("Urban Platform") != std::string::npos)
    return TileType::URBAN_MS;
  else if (this->tileName.find("3-Way") != std::string::npos)
    return TileType::URBAN_S;
  return TileType::NONE;
}

//////////////////////////////////////////////////
WorldSection UrbanGeneratorDebug::SelectWorldSection(TileType &_tileType)
{
  TileType typeFromName = this->tileTypeFromName();
  if (typeFromName == URBAN_S)
    return this->transitionWorldSection;
  else
    return WorldGeneratorDebug::SelectWorldSection(_tileType);
}

//////////////////////////////////////////////////
void UrbanGeneratorDebug::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;
  std::vector<WorldSection> addedWorldSections;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond(0, 0, -IGN_PI/2);
  op.pos += math::Vector3d(-16.021, 3.94, 0.919);
  op.tileType = URBAN_B;
  openings.push_back(op);
  int tileCount = 2;

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

      // check if we need to pick transition tile
      if (s.tileType != tileType)
        s = std::move(this->transitionWorldSection);
      else
        tileCount--;
        
      
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

      this->SetConnectionOpeningTypes(transform, openings, s);

      // add the openings of the new world section to the list
      // be sure to convert to world frame
      // for (auto &cp : s.connectionPoints)
      // {
      //   ConnectionOpening op;
      //   // determine the tileType of the opening
      //   op.pos = transform.CoordPositionAdd(cp.first);
      //   op.rot = transform.Rot() * cp.second;
      //   op.tileType = NONE;
      //   openings.push_back(op);
      // }

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

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);
    // TODO Verify where the cap needs to be placed
    math::Pose3d pose = math::Pose3d(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);

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
  usage += "Usage: urban_generator [world_type] [options]\n";
  usage += "Example Generate: urban_generator -g -t s -c 60 -n urban_test -s 25 -o urban_test.sdf\n";
  usage += "Example Debug: urban_generator -g -d \"Urban Straight\" -n urban_straight -o urban_straight.sdf\n";
  usage += "Options:\n";
  usage += "    -h\t\t Print this help message\n";
  usage += "    -d <tile>\t Generate world from tile for debugging\n";
  usage += "    -o <file>\t Output sdf filename\n";
  usage += "    -t <type>\t Urban Type:\n";
  usage += "             \t    'subways' or 's',\n";
  usage += "             \t    'buildings' or 'b',\n";
  usage += "             \t    'mixed' or 'm'\n";
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
  std::string worldType = "Urban";
  std::string urbanType;
  int seed = 0;
  bool gui = false;
  bool debug = false;
  int tileCount = 10;
  while((opt = getopt(argc, argv, "t:o:s:c:n:d:hg")) != -1)
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
      // urban type
      case 't':
      {
        urbanType = optarg;
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
    UrbanGeneratorDebug ugdb;

    ugdb.SetTileName(tileName);
    ugdb.SetOutputFile(output);
    ugdb.SetEnableGUI(gui);
    ugdb.SetWorldName(worldName);
    ugdb.SetWorldType(worldType);
    
    ugdb.Generate();
  } 
  else
  {
    UrbanGenerator ug;

    if (urbanType == "s" || urbanType == "subways")
      ug.SetSubWorldType(SubWorldType::URBAN_SUBWAY);
    else if (urbanType == "b" || urbanType == "buildings")
      ug.SetSubWorldType(SubWorldType::URBAN_BUILDING);
    else if (urbanType == "m" || urbanType == "mixed")
      ug.SetSubWorldType(SubWorldType::URBAN_MIXED_STRUCTURE);
    ug.SetOutputFile(output);
    ug.SetEnableGUI(gui);
    ug.SetWorldName(worldName);
    ug.SetWorldType(worldType);
    ug.SetSeed(seed);
    ug.SetMinTileCount(tileCount);

    ug.Generate();
  }

  return 0;
}