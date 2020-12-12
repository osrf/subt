#include "world_generator_utils.hh"

//////////////////////////////////////////////////
math::AxisAlignedBox transformAxisAlignedBox(
    const math::AxisAlignedBox &_box, const math::Pose3d &_pose)
{
  if (_box == math::AxisAlignedBox())
    return _box;

  // transform each corner and merge the result
  math::Vector3d oldMin = _box.Min();
  math::Vector3d oldMax = _box.Max();
  math::Vector3d newMin(math::MAX_D, math::MAX_D, math::MAX_D);
  math::Vector3d newMax(math::LOW_D, math::LOW_D, math::LOW_D);

  // min min min
  math::Vector3d corner = oldMin;
  auto v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min min max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max max
  corner.Y() = oldMax.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // min max min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max min
  corner.X() = oldMax.X();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max max max
  corner.Z() = oldMax.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min max
  corner.Y() = oldMin.Y();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  // max min min
  corner.Z() = oldMin.Z();
  v = _pose.Rot() * corner + _pose.Pos();
  newMin.Min(v);
  newMax.Max(v);

  return math::AxisAlignedBox(newMin, newMax);
}

//////////////////////////////////////////////////
std::string WorldTopStr(const std::string &_worldType, std::string outputFile, std::string worldName, bool gui)
{
  if (_worldType == "Tunnel")
  {
    std::stringstream ss;

    ss << "<?xml version=\"1.0\" ?>\n\n";

    ss << "<!--\n";
    ss << "    Generated using cave world generator:\n";
    ss << "        world_generator " << (gui ? "-g" : "");
    ss <<          " -o " << outputFile;
    ss << "-->\n\n";

    ss << "<sdf version=\"1.6\">\n";
    ss << "  <world name=\"" << worldName << "\">\n\n";

    ss << "    <physics name=\"1ms\" type=\"ode\">\n";
    ss << "      <max_step_size>0.004</max_step_size>\n";
    ss << "      <real_time_factor>1.0</real_time_factor>\n";
    ss << "    </physics>\n\n";

    ss << "    <scene>\n";
    ss << "      <ambient>0.1 0.1 0.1 1.0</ambient>\n";
    ss << "      <background>0 0 0 1.0</background>\n";
    ss << "      <grid>false</grid>\n";
    ss << "      <origin_visual>false</origin_visual>\n";
    ss << "    </scene>\n\n";

    ss << "    <!-- The staging area -->\n";
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>staging_area</name>\n";
    ss << "      <pose>0 0 0 0 0 0</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss <<        "subt_tunnel_staging_area</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- Tunnel tiles and artifacts -->\n";
    
    return ss.str();
  }
  else if (_worldType == "Urban")
  {
    std::stringstream ss;

    ss << "<?xml version=\"1.0\" ?>\n\n";

    ss << "<!--\n";
    ss << "    Generated using cave world generator:\n";
    ss << "        world_generator_cave " << (gui ? "-g" : "");
    ss <<          " -o " << outputFile;
    ss << "-->\n\n";

    ss << "<sdf version=\"1.6\">\n";
    ss << "  <world name=\"" << worldName << "\">\n\n";

    ss << "    <physics name=\"1ms\" type=\"ode\">\n";
    ss << "      <max_step_size>0.004</max_step_size>\n";
    ss << "      <real_time_factor>1.0</real_time_factor>\n";
    ss << "    </physics>\n\n";

    ss << "    <scene>\n";
    ss << "      <ambient>0.1 0.1 0.1 0.1</ambient>\n";
    ss << "      <background>0 0 0 1.0</background>\n";
    ss << "      <grid>false</grid>\n";
    ss << "      <origin_visual>false</origin_visual>\n";
    ss << "    </scene>\n\n";

    ss << "    <!-- The staging area -->\n";
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>staging_area</name>\n";
    ss << "      <pose>0 0 0 0 0 0</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss <<        "Urban Starting Area</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- The base station -->\n";
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>base_station</name>\n";
    ss << "      <pose>1 27 7.5 0 0 -1.5708</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss  <<       "Base Station</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- Fiducial marking the origin for artifacts reports -->\n";
    ss << "    <include>\n";
    ss << "      <name>artifact_origin</name>\n";
    ss << "      <pose>17.5 27 7.5 0 0 0</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss <<        "Fiducial</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- Urban tiles and artifacts -->\n";

    return ss.str();
  }
  // Must be 'Cave' worldType
  else
  {
    std::stringstream ss;

    ss << "<?xml version=\"1.0\" ?>\n\n";

    ss << "<!--\n";
    ss << "    Generated using cave world generator:\n";
    ss << "        world_generator_cave " << (gui ? "-g" : "");
    ss <<          " -o " << outputFile;
    ss << "-->\n\n";

    ss << "<sdf version=\"1.6\">\n";
    ss << "  <world name=\"" << worldName << "\">\n\n";

    ss << "    <physics name=\"1ms\" type=\"ode\">\n";
    ss << "      <max_step_size>0.004</max_step_size>\n";
    ss << "      <real_time_factor>1.0</real_time_factor>\n";
    ss << "    </physics>\n\n";

    ss << "    <scene>\n";
    ss << "      <ambient>1.0 1.0 1.0 1.0</ambient>\n";
    ss << "      <background>0 0 0 1.0</background>\n";
    ss << "      <grid>false</grid>\n";
    ss << "      <origin_visual>false</origin_visual>\n";
    ss << "    </scene>\n\n";

    ss << "    <!-- The staging area -->\n";
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>staging_area</name>\n";
    ss << "      <pose>0 0 0 0 0 0</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss <<        "Cave Starting Area Type B</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- The base station -->\n";
    ss << "    <include>\n";
    ss << "      <static>true</static>\n";
    ss << "      <name>base_station</name>\n";
    ss << "      <pose>-8 0 0 0 0 -1.5708</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss  <<       "Base Station</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- Fiducial marking the origin for artifacts reports -->\n";
    ss << "    <include>\n";
    ss << "      <name>artifact_origin</name>\n";
    ss << "      <pose>10.0 0.0 0.0 0 0 0</pose>\n";
    ss << "      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/";
    ss <<        "Fiducial</uri>\n";
    ss << "    </include>\n\n";

    ss << "    <!-- Cave tiles and artifacts -->\n";

    return ss.str();
  }
}