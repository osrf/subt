#!/bin/bash
name=$(basename $0)

echo -e "[\e[93m$name\e[m] Converting file lily.xacro to ../lily.sdf."

./xacro2sdf.sh lily.xacro ../lily.sdf || exit $?

echo -e "[\e[93m$name\e[m] File lily.xacro coverted to ../lily.sdf."