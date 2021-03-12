#!/bin/bash

# Check for the existance of the team name on the command line
if [ $# -lt 4 ]; then
  echo "Specify the team name, local docker image, local docker tag, and upstream docker tag."
  echo "To push a new Docker image:"
  echo "  ./subt_docker.sh <team_name> <local_docker_image> <local_docker_tag> <upstream_docker_tag>"
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
awsVersion=`aws --version`
if [[ "$awsVersion" == *"aws-cli/1"* ]]; then
  $(aws ecr get-login --no-include-email --region us-east-1)
elif [[ "$awsVersion" == *"aws-cli/2"* ]]; then
  eval 'aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin 138467776890.dkr.ecr.us-east-1.amazonaws.com'
else
  echo "Unsupported aws cli version $awsVersion"
  exit
fi

if [ $? != 0 ]; then
  echo "Failed to log into docker. Check your AWS credentials."
  exit
fi

# Tag the docker file
echo "Tagging image $image:$local_tag with $upstream_tag"
docker tag $image:$local_tag 138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag

# Docker push
echo "Pushing to docker repository"
docker push 138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag

echo "Docker image URL: 138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/$team:$upstream_tag"
