require_relative 'common.rb'

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins='')
  _spawner2(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins)
end

def rosExecutables(_name, _worldName)
  _rosExecutables2(_name, _worldName)
end
