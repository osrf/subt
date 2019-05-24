#!/bin/bash

# Log into docker

# Check for the existance of the team name on the command line
if [ $# -eq 0 ]; then
  echo "Specify the team name."
  echo "Usage:"
  echo "  ./subt_docker.sh <team_name> <docker_image> <local_docker_tag> <upstream_docker_tag>"
  exit
fi

team=$1
image=$2
local_tag=$3
upstream_tag=$4

# Check for the existance of your aws credentials
myAccessKeyId=`aws configure get aws_access_key_id`
mySecretAccessKey=`aws configure get aws_secret_access_key`
if [ -z "${myAccessKeyId}" ]; then
  echo "Your aws access key id is not set."
  exit
fi

if [ -z "${mySecretAccessKey}" ]; then
  echo "Your aws secret access key is not set."
  exit
fi

# Log into docker
echo "Logging into docker"
$(aws ecr get-login --no-include-email --region us-east-1)

# Tag the docker file
echo "Tagging image $image:$local_tag with $upstream_tag"
docker tag $image:$local_tag 200670743174.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag

# Docker push
echo "Pushing to docker repository"
docker push 200670743174.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag

echo "Docker image URL: 200670743174.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag"
