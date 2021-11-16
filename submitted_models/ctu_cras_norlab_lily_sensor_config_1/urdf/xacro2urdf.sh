#!/bin/bash
if [ "$#" -lt 1 ];
then
    echo -e "Usage: xacro2urdf.sh XACRO-FILE [URDF-FILE] \nXACRO-FILE .. xacro file to be transfered into the urdf file \nURDF-FILE .. optional filename for the converted urdf. Ff provided, output is not written to stdout but saved to this file."
    exit 1
fi

# conversion file
tmp_file="__tmp.urdf"

# main converion
xacro --xacro-ns $1 target:=ign > $tmp_file 

# handle tmp file
if [ "$#" -ge 2 ];
then
    # in case of saving to file, just rename it...
    mv $tmp_file "${2%.*}.urdf"
else
    # ... otherwise writing file to stdout
    cat $tmp_file

    # and remove it.
    rm $tmp_file
fi