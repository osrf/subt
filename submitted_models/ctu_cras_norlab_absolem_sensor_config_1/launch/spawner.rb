require_relative 'common.rb'

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF='', _max_velocity=0.4, _max_acceleration=3)
  _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSDF, _max_velocity, _max_acceleration)
end

def rosExecutables(_name, _worldName)
  _rosExecutables(_name, _worldName)
end
