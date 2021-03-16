
#include "world_generator_base.hh"
#include "world_generator_utils.hh"

namespace subt {
//////////////////////////////////////////////////
class CaveGeneratorBase
{
  /// \brief Create a collection of prefab world sections from Type A tiles
  /// The world section are used to create the final generated world
  protected: std::vector<WorldSection> CreateTypeAWorldSections();

  /// \brief Create a collection of prefab world sections from Type B tiles
  /// The world section are used to create the final generated world
  protected: std::vector<WorldSection> CreateTypeBWorldSections(std::map<std::string, std::vector<ignition::math::Vector3d>>
      &_tileConnectionPoints);

  /// \brief Create a world section from transitiion tile
  protected: void CreateTransitionWorldSection();

  /// \brief World section created from transition tile
  protected: WorldSection transitionWorldSection;

};

//////////////////////////////////////////////////
class CaveGenerator : CaveGeneratorBase, public WorldGenerator
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

  /// \brief Correct the transition world sections based on opening type
  private: bool CorrectTransitionWorldPose(WorldSection &_s, TileType &_type) override;

  /// \brief Adjust the opening tile type if transition tiles are considered
  private: void AdjustOpeningTileType(WorldSection &_s, ConnectionOpening &_op, bool adjust);

};

//////////////////////////////////////////////////
class CaveGeneratorDebug : CaveGeneratorBase, public WorldGeneratorDebug
{
  /// \brief Generate the world sdf
  public: void Generate();

  /// \brief Create a collection of prefab world sections from Type A tiles
  private: void CreateTypeAWorldSections();

  /// \brief Preprocess all tiles from the ConnectionHelper class to filter
  /// out only the tiles needed for this world generator class. In addtion,
  /// we also generate bounding box data for each tile.
  protected: void LoadTiles() override;

};
}

//////////////////////////////////////////////////
std::vector<WorldSection> CaveGeneratorBase::CreateTypeAWorldSections()
{
  std::vector<WorldSection> worldSections;
  static size_t nextId = 0u;
  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Vertical Shaft Straight Bottom Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Cantilevered Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Vertical Shaft Straight Top Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, 25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 02 Type A";
    t.model.SetRawPose(math::Pose3d(100, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation 02 Type A";
    t.model.SetRawPose(math::Pose3d(150, 0, 25, 0, 0, -3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Cap Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(200, 0, 100), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 0, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, -25), math::Quaterniond::Identity));

    worldSections.push_back(s);
  }
  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetRawPose(math::Pose3d(75, 75, -25, 0, 0, -3.14159));
    s.tiles.push_back(t);


    t.tileType = "Cave Corner 03 Type A";
    t.model.SetRawPose(math::Pose3d(75, 150, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, -100, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn Elevation Type A";
    t.model.SetRawPose(math::Pose3d(175, -50, 0, 0, 0, 0));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(100, 150, -25), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -125, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -75, 25), math::Quaterniond(0, 0, -IGN_PI/2)));

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, 0, 0, 0, 0));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(75, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(50, -100, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 2 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(100, -125, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 4 Way 01 Type A";
    t.model.SetRawPose(math::Pose3d(125, -100, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 02 Type A";
    t.model.SetRawPose(math::Pose3d(125, -50, 0, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -100, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, 0), math::Quaterniond(0, 0, IGN_PI/2)));

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Elevation Straight Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way Elevation 03 Type A";
    t.model.SetRawPose(math::Pose3d(150, 0, 75, 0, 0, 3.14159));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 75), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, -50, 150), math::Quaterniond(0, 0, -IGN_PI/2)));

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(25, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Cavern Type A";
    t.model.SetRawPose(math::Pose3d(75, 0, -25, 0, 0, 3.14159));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetRawPose(math::Pose3d(125, 0, 0, 0, 0, -1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(175, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(150, 50, 0), math::Quaterniond::Identity));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, 25, -25), math::Quaterniond(0, 0, IGN_PI/2)));
    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(75, -25, -25), math::Quaterniond(0, 0, -IGN_PI/2)));

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, -75, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(125, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave U Turn 01 Type A";
    t.model.SetRawPose(math::Pose3d(175, -25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s;
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;

    VertexData t;
    t.tileType = "Cave 3 Way Elevation 01 Type A";
    t.model.SetRawPose(math::Pose3d(50, 0, 0, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, 25, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Shift Type A";
    t.model.SetRawPose(math::Pose3d(125, -75, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Corner 01 Type A";
    t.model.SetRawPose(math::Pose3d(175, 50, -25, 0, 0, -1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave 3 Way 02 Type A";
    t.model.SetRawPose(math::Pose3d(225, 0, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    t.tileType = "Cave Straight Type A";
    t.model.SetRawPose(math::Pose3d(175, -50, -25, 0, 0, 1.5708));
    s.tiles.push_back(t);

    s.connectionPoints.push_back(std::make_pair(
        math::Vector3d(250, 0, -25), math::Quaterniond::Identity));

    worldSections.push_back(s);
  }

  // --------------------------
  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Split Type A",
        math::Vector3d(0, 50, 0), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Elevation 01 Type A",
        math::Vector3d(0, 50, 75), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Elevation Straight Type A",
        math::Vector3d(0, -50, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Straight Type A",
        math::Vector3d(0, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    worldSections.push_back(s);
  }
  return worldSections;
}

//////////////////////////////////////////////////
std::vector<WorldSection> CaveGeneratorBase::CreateTypeBWorldSections(std::map<std::string, std::vector<ignition::math::Vector3d>>
      &_tileConnectionPoints)
{
  std::vector<WorldSection> worldSections;
  static size_t nextId = 0u;
  double tileSize = 25.0;
  double halfTileSize = tileSize * 0.5;
  for (const auto &t : _tileConnectionPoints)
  {
    // only accept Type B tiles
    if (t.first.find("Type B") == std::string::npos)
      continue;

    // filter out Type B to Type A transition tile
    if (t.first.find("Type A") != std::string::npos)
      continue;

    // ignore starting area
    if (t.first.find("Starting") != std::string::npos)
      continue;

    // build cavern manually
    // Ignore Split 01 -> flight traversable
    if (t.first.find("Cavern Split 02 Type B") != std::string::npos)
    {
      // build cavern world section
        WorldSection s;
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;

        VertexData t;
        t.tileType = "Cave Cavern Split 02 Type B";
        t.model.SetRawPose(math::Pose3d(halfTileSize, 0, 0, 0, 0, 0));
        s.tiles.push_back(t);

        t.tileType = "Cave Cavern Split 01 Type B";
        t.model.SetRawPose(math::Pose3d(halfTileSize + tileSize, 0, 0, 0, 0, 0));
        s.tiles.push_back(t);

        s.connectionPoints.push_back(std::make_pair(
            math::Vector3d(tileSize + tileSize, 0, 0),
            math::Quaterniond::Identity));

        s.connectionPoints.push_back(std::make_pair(
            math::Vector3d(halfTileSize + tileSize, tileSize, 25),
            math::Quaterniond(0, 0, IGN_PI/2)));

        worldSections.push_back(s);
        std::cout << "Created tile:\tCave Cavern Split 01 & 02 Type B" << std::endl;
    }
    else if (t.first.find("Straight") != std::string::npos ||
        t.first.find("Vertical Shaft") != std::string::npos)
    {
      WorldSection s = std::move(
          CreateWorldSectionFromTile(t.first,
          math::Vector3d(0, -halfTileSize, 0), math::Quaterniond(0, 0, -IGN_PI/2),
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      worldSections.push_back(s);
      std::cout << "Created tile:\t" << t.first << std::endl;
    }
    else if (t.first.find("Corner 01") != std::string::npos)
    {
      WorldSection s = std::move(
          CreateWorldSectionFromTile(t.first,
          math::Vector3d(0, halfTileSize, 0), math::Quaterniond(0, 0, IGN_PI/2),
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      worldSections.push_back(s);
      std::cout << "Created tile:\t" << t.first << std::endl;
    }
    else if (t.first.find("Corner 02") != std::string::npos)
    {
      WorldSection s = std::move(
          CreateWorldSectionFromTile(t.first,
          math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
          CAVE_TYPE_B));
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      worldSections.push_back(s);
      std::cout << "Created tile:\t" << t.first << std::endl;
    }
    // TODO Adjust the rotation to account for the 30 degrees
    // Currently ignored in LoadTiles()
    else if (t.first.find("Corner 30") != std::string::npos)
    {
      WorldSection s;
      if (t.first.find("30F D"))
      {
        s = std::move(
            CreateWorldSectionFromTile(t.first,
            math::Vector3d(0, halfTileSize, 0), math::Quaterniond(0, 0, IGN_PI/2),
            CAVE_TYPE_B));
      } else
      {
        s = std::move(
          CreateWorldSectionFromTile(t.first,
          math::Vector3d(0, -halfTileSize, 0), math::Quaterniond(0, 0, -IGN_PI/2),
          CAVE_TYPE_B));
      }
      s.tileType = CAVE_TYPE_B;
      s.id = nextId++;
      worldSections.push_back(s);
      std::cout << "Created tile:\t" << t.first << std::endl;
    }
    else if (t.first.find("3 Way") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            CreateWorldSectionFromTile(t.first,
            math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        worldSections.push_back(s);
        std::cout << "Created tile:\t" << t.first << std::endl;
      }
    }
    else if (t.first.find("Vertical Shaft") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            CreateWorldSectionFromTile(t.first,
            math::Vector3d(-halfTileSize, 0, 0), math::Quaterniond::Identity,
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        worldSections.push_back(s);
        std::cout << "Created tile:\t" << t.first << std::endl;
      }
    }
    else if (t.first.find("Elevation") != std::string::npos)
    {
      {
        WorldSection s = std::move(
            CreateWorldSectionFromTile(t.first,
            math::Vector3d(0, -halfTileSize, 10),
            math::Quaterniond(0, 0, -IGN_PI/2),
            CAVE_TYPE_B));
        s.tileType = CAVE_TYPE_B;
        s.id = nextId++;
        worldSections.push_back(s);
        std::cout << "Created tile:\t" << t.first << std::endl;
      }
    }
  }
  return worldSections;
}

//////////////////////////////////////////////////
void CaveGeneratorBase::CreateTransitionWorldSection()
{
  static size_t nextId = 0u;
  std::string type = "Cave Transition Type A to and from Type B";
  this->transitionWorldSection = std::move(
      CreateWorldSectionFromTile(type,
      math::Vector3d(0, 25, 0), math::Quaterniond(0, 0, IGN_PI/2),
      CAVE_TYPE_TRANSITION));
  this->transitionWorldSection.tileType = CAVE_TYPE_TRANSITION;
  this->transitionWorldSection.id = nextId++;
}

//////////////////////////////////////////////////
void CaveGenerator::AdjustOpeningTileType(WorldSection &_s, ConnectionOpening &_op, bool adjust)
{
  // mark the tile type for the opening of the transition tile
  // based on the tile type the opening connects to.
  if (_s.tileType == CAVE_TYPE_TRANSITION)
  {
    if (adjust)
      _op.tileType = CAVE_TYPE_B;
    else
      _op.tileType = CAVE_TYPE_A;
  }
  else
  {
    _op.tileType = _s.tileType;
  }
}

//////////////////////////////////////////////////
void CaveGenerator::LoadTiles()
{
  WorldGenerator::LoadTiles();
  // create prefab world sections
  if (this->subWorldType == CAVE_ANASTOMOTIC ||
      this->subWorldType == CAVE_CURVILINEAR)
  {
    this->CreateTransitionWorldSection();
    this->worldSections = this->CreateTypeAWorldSections();
  }
  if (this->subWorldType == CAVE_RECTILINEAR ||
      this->subWorldType == CAVE_CURVILINEAR)
  {
    std::vector<WorldSection> typeBWorlds = this->CreateTypeBWorldSections(this->tileConnectionPoints);
    this->worldSections.insert(this->worldSections.end(), typeBWorlds.begin(),typeBWorlds.end());
  }
}

//////////////////////////////////////////////////
WorldSection CaveGenerator::SelectWorldSection(TileType &_tileType)
{
  if (this->subWorldType == CAVE_ANASTOMOTIC)
  {
    if(this->addedWorldSections.empty())
    {
      return this->transitionWorldSection;
    }
    return WorldGenerator::SelectWorldSection(_tileType);
  }
  else if (this->subWorldType == CAVE_CURVILINEAR)
  {
    // set a 20% probablity of including a transition tile for
    // curvilinear worlds
    if (rand() % 10 + 1 > 8)
      return this->transitionWorldSection;
    
    return WorldGenerator::SelectWorldSection(_tileType);
  } 
  // Must be CAVE_RECTILINEAR
  else
    return WorldGenerator::SelectWorldSection(_tileType);  
}

//////////////////////////////////////////
bool CaveGenerator::CorrectTransitionWorldPose(WorldSection &_s, TileType &_tileType)
{
  if (_s.tileType == CAVE_TYPE_TRANSITION)
  {
    // if the tile to connect to is Type A, we need to flip the transition
    // tile by 180 degrees
    if (_tileType == CAVE_TYPE_A)
    {
      auto &transitionTile = _s.tiles[0].model;
      transitionTile.SetRawPose(math::Pose3d(transitionTile.RawPose().Pos(),
          math::Quaterniond(0, 0, IGN_PI) * transitionTile.RawPose().Rot()));
      return true;
    }
    return false;
  }
  return false;
}

//////////////////////////////////////////////////
void CaveGenerator::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond::Identity;
  op.pos += math::Vector3d(12.5, 0, 0);
  op.tileType = CAVE_TYPE_B;
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
      for (auto &cp : s.connectionPoints)
      {
        ConnectionOpening op;
        op.pos = transform.CoordPositionAdd(cp.first);
        op.rot = transform.Rot() * cp.second;

        // mark the tile type for the opening of the transition tile
        // based on the tile type the opening connects to.
        if (s.tileType == CAVE_TYPE_TRANSITION)
        {
            op.tileType = CAVE_TYPE_A;
        }
        else
        {
          op.tileType = s.tileType;
        }
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
  int caveTypeACount = 0;
  int caveTypeBCount = 0;
  int caveTransitionCount = 0;
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

    // keep track of tile count for each tile type
    if (s.tileType == CAVE_TYPE_A)
      caveTypeACount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_B)
      caveTypeBCount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_TRANSITION)
      caveTransitionCount++;

    // update world bbox info
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  std::string capUriTypeA = capUri + "Cave Cap Type A";
  std::string capUriTypeB = capUri + "Cave Cap Type B";

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);
    math::Pose3d pose;
    std::string uri;
    if (uf.tileType == CAVE_TYPE_A)
    {
      uri = capUriTypeA;
      // the cap is placed at opening
      pose = math::Pose3d(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);
    }
    else if (uf.tileType == CAVE_TYPE_B)
    {
      uri = capUriTypeB;
      // the cap origin is at an offset and so it needs to be placed at
      // center of the next tile position instead of where the opening is
      pose = math::Pose3d(pos + rot*math::Vector3d(12.5, 0, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
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
  std::cout << "    Type A Tile count: " << caveTypeACount << std::endl;
  std::cout << "    Type B Tile count: " << caveTypeBCount<< std::endl;
  std::cout << "    Transition Tile count: " << caveTransitionCount << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl;
}

//////////////////////////////////////////////////
void CaveGeneratorDebug::CreateTypeAWorldSections()
{
  static size_t nextId = 0u;
  // --------------------------
  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave U Turn Elevation Type A",
        math::Vector3d(50, 25, 0), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }
  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Split Type A",
        math::Vector3d(0, 50, 0), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Elevation 01 Type A",
        math::Vector3d(0, 50, 75), math::Quaterniond(0, 0, IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Elevation Straight Type A",
        math::Vector3d(0, -50, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }

  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave Straight Type A",
        math::Vector3d(0, -25, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }
  {
    WorldSection s = std::move(
        CreateWorldSectionFromTile("Cave 2 Way 01 Type A",
        math::Vector3d(25, 0, 0), math::Quaterniond(0, 0, -IGN_PI/2),
        CAVE_TYPE_A));
    s.tileType = CAVE_TYPE_A;
    s.id = nextId++;
    this->worldSections.push_back(s);
  }
}

//////////////////////////////////////////////////
void CaveGeneratorDebug::LoadTiles()
{
  WorldGeneratorDebug::LoadTiles();
  // create prefab world sections
  if (this->tileName.find("Type A") != std::string::npos)
  {
    this->CreateTransitionWorldSection();
    this->CreateTypeAWorldSections();
  } 
  else
    this->worldSections = this->CreateTypeBWorldSections(this->tileConnectionPoints);
}

//////////////////////////////////////////////////
void CaveGeneratorDebug::Generate()
{
  this->LoadTiles();

  std::list<ConnectionOpening> openings;
  std::list<ConnectionOpening> unfilledOpenings;

  // first connection opening is at entrance pos in staging area
  ConnectionOpening op;
  op.rot = math::Quaterniond::Identity;
  op.pos += math::Vector3d(12.5, 0, 0);
  op.tileType = CAVE_TYPE_B;
  openings.push_back(op);
  int tileCount;

  // limit worlds to {b_tile} && {transition_tile, a_tile}
  std::vector<WorldSection> addedWorldSections;
  if(this->tileName.find("Type A") != std::string::npos)
  {
    tileCount = 2;
  }
  else if(this->tileName.find("Type B") != std::string::npos)
  {
    tileCount = 1;
  }
  else
  {
    tileCount = 0;
  }

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
      if(s.tileType == CAVE_TYPE_A && tileType == CAVE_TYPE_B)
      {
        // Select transition tile
        s = std::move(this->transitionWorldSection);
      }
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

        // mark the tile type for the opening of the transition tile
        // based on the tile type the opening connects to.
        if (s.tileType == CAVE_TYPE_TRANSITION)
        {
            op.tileType = CAVE_TYPE_A;
        }
        else
        {
          op.tileType = s.tileType;
        }
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
  int caveTypeACount = 0;
  int caveTypeBCount = 0;
  int caveTransitionCount = 0;
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

    // keep track of tile count for each tile type
    if (s.tileType == CAVE_TYPE_A)
      caveTypeACount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_B)
      caveTypeBCount += s.tiles.size();
    else if (s.tileType == CAVE_TYPE_TRANSITION)
      caveTransitionCount++;

    // update world bbox info
    for (const auto &bbox : s.boundingboxes)
      worldBBox.Merge(bbox);
  }

  // fill all openings with caps
  int capNo = 1;
  std::string capUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
  std::string capUriTypeA = capUri + "Cave Cap Type A";
  std::string capUriTypeB = capUri + "Cave Cap Type B";

  for (const auto & uf : unfilledOpenings)
  {
    // convert cap pose to world coordinates
    math::Vector3d pos = uf.pos;
    math::Quaterniond rot = uf.rot;
    std::string name = "cap_" + std::to_string(capNo++);
    math::Pose3d pose;
    std::string uri;
    if (uf.tileType == CAVE_TYPE_A)
    {
      uri = capUriTypeA;
      // the cap is placed at opening
      pose = math::Pose3d(pos, math::Quaterniond(0, 0, -IGN_PI/2)*rot);
    }
    else if (uf.tileType == CAVE_TYPE_B)
    {
      uri = capUriTypeB;
      // the cap origin is at an offset and so it needs to be placed at
      // center of the next tile position instead of where the opening is
      pose = math::Pose3d(pos + rot*math::Vector3d(12.5, 0, 0),
          math::Quaterniond(0, 0, IGN_PI/2)*rot);
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
  std::cout << "    Type A Tile count: " << caveTypeACount << std::endl;
  std::cout << "    Type B Tile count: " << caveTypeBCount<< std::endl;
  std::cout << "    Transition Tile count: " << caveTransitionCount << std::endl;
  std::cout << "  Approx. World Dimension (excl. staging area): "
            << worldBBox.Size() << std::endl; 
}

//////////////////////////////////////////////////
void printUsage()
{
  std::string usage;
  usage += "Usage: cave_generator [world_type] [options]\n";
  usage += "Example Generate: cave_generator -g -t a -c 60 -n cave_test -s 25 -o cave_test.sdf\n";
  usage += "Example Debug: cave_generator -g -d \"Cave Split Type A\" -n cave_split_type_a -o cave_split_type_a.sdf\n";
  usage += "Options:\n";
  usage += "    -h\t\t Print this help message\n";
  usage += "    -d <tile>\t Generate world from tile for debugging\n";
  usage += "    -o <file>\t Output sdf filename\n";
  usage += "    -t <type>\t Cave Type:\n";
  usage += "             \t    'anastomotic' or 'a',\n";
  usage += "             \t    'curvilinear' or 'c',\n";
  usage += "             \t    'rectilinear' or 'r'\n";
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
  std::string caveType;
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
        std::cout << "File name:\t" << output << std::endl;
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
      // cave type
      case 't':
      {
        caveType = optarg;
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
    CaveGeneratorDebug cgdb;
    cgdb.SetTileName(tileName);
    cgdb.SetOutputFile(output);
    cgdb.SetEnableGUI(gui);
    cgdb.SetWorldName(worldName);
    cgdb.SetWorldType("Cave");
    
    cgdb.Generate();
  } 
  else
  {
    CaveGenerator cg;
    if (caveType == "a" || caveType == "anastomotic")
      cg.SetSubWorldType(SubWorldType::CAVE_ANASTOMOTIC);
    else if (caveType == "c" || caveType == "curvilinear")
      cg.SetSubWorldType(SubWorldType::CAVE_CURVILINEAR);
    else if (caveType == "r" || caveType == "rectilinear")
      cg.SetSubWorldType(SubWorldType::CAVE_RECTILINEAR);
    cg.SetOutputFile(output);
    cg.SetEnableGUI(gui);
    cg.SetWorldName(worldName);
    cg.SetWorldType("Cave");
    cg.SetSeed(seed);
    cg.SetMinTileCount(tileCount);
    
    cg.Generate();
  }

  return 0;
}