#!/bin/bash

# Check for the existance of the team name on the command line
if [ $# -ne 2 ]; then
  echo "Please Specify the team name."
  echo "Construct the URI of a Docker image on a team's repository:"
  echo "  ./subt_docker_list.sh <team_name> <upstream_tag>"
  exit
fi

echo "138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/$1:$2"
