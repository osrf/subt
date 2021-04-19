#!/bin/bash
if [ "$#" -lt 1 ];
then
    echo -e "Usage: urdf2sdf.sh URDF-FILE [SDF-FILE] \nURDF-FILE .. urdf file to be transfered into the sdf file \nSDF-FILE .. optional filename for the converted sdf. If provided, output is not written to stdout but saved to this file."
    exit 1
fi

# conversion file
tmp_file="__tmp.sdf"

# some magic for hight precision constants script by MP
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

sdf8_version="$(ign sdf --versions | grep '^8')" || true
[ -z "$sdf8_version" ] && echo "libsdformat8 not found. It is required to update this robot's SDF. Please install libsdformat8-dev and try again." >&2 && exit 1
echo "Found libsdformat ${sdf8_version}" >&2

# main converion
ign sdf --force-version "$sdf8_version" -p $1 |
"${DIR}/high_precision_constants.py" - |
sed -e 's#model://ctu_cras_norlab_lily_sensor_config_1/##g' > $tmp_file

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

