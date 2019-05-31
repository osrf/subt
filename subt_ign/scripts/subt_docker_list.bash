#!/bin/bash

# Check for the existance of the team name on the command line
if [ $# -ne 1 ]; then
  echo "Please Specify the team name."
  echo "To list available images:"
  echo "  ./subt_docker_list.sh <team_name>"
  exit
fi

team=$1

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

if [ $? != 0 ]; then
  echo "Failed to log into docker. Check your AWS credentials."
  exit
fi

aws ecr list-images --repository-name subt/$team
