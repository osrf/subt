#!/usr/bin/env ruby
#
# Copyright (C) 2019 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

require 'json'
require 'optparse'
require 'tempfile'

# Example JSON.
example_json=<<EOF
{
  "circuit": "cave",
  "world": "simple_cave_01",
  "headless": "false",
  "robots":
  [
    {
      "name":"X1",
      "type":"X1_SENSOR_CONFIG_1",
      "image":"osrf/subt-virtual-testbed:subt_solution_latest"
    },
    {
      "name":"X2",
      "type":"X2_SENSOR_CONFIG_2",
      "image":"osrf/subt-virtual-testbed:subt_solution_latest"
    }
  ]
}
EOF

# Usage information.
usage =<<EOF
Convert a SubT JSON solution to a Docker Compose yaml file.

Usage:
  subt-compose -f <json_file> -o <docker_yaml_file>
  subt-compose [Options]

Options:
  -b [--bridge] arg      Set the cloudsim_bridge image.
  -e [--example]         Output an example JSON file.
  -f [--file] arg        The JSON file to convert.
  -h [--help]            Output this help message.
  -o [--output] arg      Specifiy output Docker Compose YAML file.
  -s [--sim] arg         Set the cloudsim_sim image.
  -r [--run]             Run the JSON file using docker-compose.

Example JSON submission:
#{example_json}
EOF

# Options hash
options = {
  'file' => '',
  'output' => '',
  'sim_image' => 'osrf/subt-virtual-testbed:cloudsim_sim_latest',
  'bridge_image' => 'osrf/subt-virtual-testbed:cloudsim_bridge_latest',
  'run' => false
}

# Options parser
opt_parser = OptionParser.new do |opts|
  opts.banner = usage

  opts.on('-b', '--bridge [arg]', String, 'cloudsim_bridge image') do |b|
    options['bridge_image'] = b
  end

  # Output the example json.
  opts.on('-e', '--example') do
    puts example_json
    exit
  end

  # JSON input file
  opts.on('-f', '--file [arg]', String, 'JSON file to convert') do |f|
    options['file'] = f
  end

  # Output the usage.
  opts.on('-h', '--help') do
    puts usage
    exit
  end

  # YAML output file
  opts.on('-o', '--output [arg]', String, 'Output YAML file') do |f|
    options['output'] = f
  end

  # Simulation image
  opts.on('-s', '--sim [arg]', String, 'cloudsim_sim image') do |s|
    options['sim_image'] = s
  end

  # Should the submission be run?
  opts.on('-r', '--run') do
    options['run'] = true
  end
end

if ARGV.length < 1
  puts usage
end

# Process the command line arguments.
opt_parser.parse!(ARGV)

# Check the input file is specified.
if options['file'] == '' || options['file'] == nil
  puts "Missing filename argument."
  exit
end

# Make sure the provided file exists.
if !File.exist?(options['file'])
  puts "JSON file, #{options['file']}, does not exist."
  exit
end

# Read the json file.
begin
  submission = JSON.parse(File.read(options['file']))
rescue
  puts "Unable to parse JSON file #{options['file']}."
  puts "Output an example JSON file using the '-e' option."
  exit
end

# Check the JSON contents.
if !submission.key?('circuit')
  puts "JSON file does not contain a 'circuit' name."
  puts "Output an example JSON file using the '-e' option."
  exit
end
if !submission.key?('world')
  puts "JSON file does not contain a 'world' name."
  puts "Output an example JSON file using the '-e' option."
  exit
end
if !submission.key?('robots')
  puts "JSON file does not contain a 'robots' array."
  puts "Output an example JSON file using the '-e' option."
  exit
end
if !submission.key?('headless')
  submission['headless'] = 'false'
end

# Construct a the robot string that is passed to the cloudsim_sim image.
robotStr = ""
submission['robots'].each_with_index do |robot, index|
  if !robot.key?('name')
    puts "A robot in the 'robots' array is missing a 'name'."
    puts "Output an example JSON file using the '-e' option."
    exit
  end
  if !robot.key?('type')
    puts "A robot in the 'robots' array is missing a 'type'."
    puts "Output an example JSON file using the '-e' option."
    exit
  end
  if !robot.key?('image')
    puts "A robot in the 'robots' array is missing a 'image'."
    puts "Output an example JSON file using the '-e' option."
    exit
  end

  robotStr += "robotName#{index+1}:=#{robot['name']} robotConfig#{index+1}:=#{robot['type']} "
end

marsupialChildren = []
if submission.key?('marsupials')
  submission['marsupials'].each_with_index do |(parent, child), index|
    robotStr += "marsupial#{index+1}:=#{parent}:#{child} "
    marsupialChildren.append(child)
  end
end

# Set where the output is going
if options['run']
  output = Tempfile.new()
elsif options['output'] == '' || options['output'] == nil
  output = $stdout.dup
else
  output = File.open(options['output'], "w")
end

begin
  # Output the simulation container
  output.puts <<EOF
version: '2.4'
services:
  # The sim container runs Gazebo and all simulation plugins.
  # In this example, two robots are started with the names X1 and X2.
  sim:
    image: #{options['sim_image']}
    command: cloudsim_sim.ign headless:=#{submission['headless']} circuit:=#{submission['circuit']} ros:=true worldName:=#{submission['world']} #{robotStr}
    networks:
      sim_net:
        ipv4_address: 172.28.1.1
    environment:
      - DISPLAY
      - QT_X11_NO_MITSHM=1
      - XAUTHORITY=/tmp/.docker.xauth
      - IGN_PARTITION=sim
      - IGN_IP=172.28.1.1
    privileged: true
    runtime: nvidia
    security_opt:
      - seccomp=unconfined
    volumes:
      - "/tmp/.docker.xauth:/tmp/.docker.xauth"
      - "/tmp/.X11-unix:/tmp/.X11-unix"
      - "/etc/localtime:/etc/localtime:ro"
      - "/dev/input:/dev/input" 

EOF

  # Output bridge and solution definitions
  submission['robots'].each_with_index do |robot, index|
    marsupialStr = "marsupial:=true" if marsupialChildren.include? robot['name']
    output.puts <<EOF
  bridge#{index+1}:
    image: #{options['bridge_image']}
    command: circuit:=#{submission['circuit']} worldName:=#{submission['world']} robotName1:=#{robot['name']} robotConfig1:=#{robot['type']} #{marsupialStr}
    networks:
      relay_net#{index+1}:
        ipv4_address: 172.#{29+index}.1.1
      sim_net:
        ipv4_address: 172.28.1.#{index+2}
    environment:
      - IGN_PARTITION=sim
      - IGN_IP=172.28.1.#{index+2}
      - ROS_MASTER_URI=http://172.#{29+index}.1.1:11311
    depends_on:
      - "sim"

  solution#{index+1}:
    image: #{robot['image']}
    networks:
      relay_net#{index+1}:
        ipv4_address: 172.#{29+index}.1.2
    environment:
      - ROS_MASTER_URI=http://172.#{29+index}.1.1:11311
    runtime: nvidia
    depends_on:
      - "bridge#{index+1}"

EOF
  end

  # Output network definitions
  output.puts <<EOF
networks:
  sim_net:
    ipam:
      driver: default
      config:
        - subnet: 172.28.0.0/16
EOF

  submission['robots'].each_with_index do |robot, index|
    output.puts <<EOF
  relay_net#{index+1}:
    internal: true
    ipam:
      driver: default
      config:
        - subnet: 172.#{29+index}.0.0/16
EOF
  end

 if options['run']
   output.close
   system "docker-compose -f #{output.path} up"
   system "docker-compose -f #{output.path} down"
   output.unlink
 end
ensure
  output.close
end
