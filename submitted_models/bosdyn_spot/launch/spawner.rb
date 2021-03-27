require_relative 'common.rb'

# look in common.rb for the actual code; the separation of the code to common.rb was done so that
# other sensor configs can reuse the code; as they reuse it by calling `require`, if they included
# directly this spawner script, their own spawner() and rosExecutables() functions would be overwritten
# if they `require` the common.rb file, functions with non-colliding names _spawner and _rosExecutables
# will be imported

def spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins='')
  _spawner(_name, _modelURI, _worldName, _x, _y, _z, _roll, _pitch, _yaw, _additionalSpawnPlugins)
end

def rosExecutables(_name, _worldName)
  _rosExecutables(_name, _worldName)
end
