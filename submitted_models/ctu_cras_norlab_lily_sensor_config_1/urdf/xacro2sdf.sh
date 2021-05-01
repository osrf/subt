#!/bin/bash
if [ "$#" -lt 1 ];
then
    echo -e "Usage: xacro2sdf.sh XACRO-FILE [SDF-FILE] \nXACROF-FILE .. xacro file to be transfered into the sdf file \nSDF-FILE .. optional filename for the output sdf. ptional filename for the converted sdf. If provided, output is not written to stdout but saved to this file."
    exit 1
fi

# conversion file
tmp_file="___tmp.sdf"
pipe_file="__out.urdf"

# both conversions
./xacro2urdf.sh $1 $pipe_file || exit $?
./urdf2sdf.sh $pipe_file $tmp_file || exit $?
rm $pipe_file

# handle tmp file
if [ "$#" -ge 2 ];
then
    # in case of saving to file, just rename it...
    mv $tmp_file "${2%.*}.sdf"
else
    # ... otherwise writing file to stdout
    cat $tmp_file

    # and remove it.
    rm $tmp_file
fi