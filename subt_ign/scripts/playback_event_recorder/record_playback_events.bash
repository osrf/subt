#!/bin/bash


echo "==================================="
echo "Staring Playback Events Recorder"
echo "==================================="

if [ -z "$1" ]; then
  echo "Usage: bash ./record_playback_events.bash [path_to_log_dir]"
  exit 0
fi

logDirPath=$1

if [ ! -d "$logDirPath" ]; then
  echo "Directory does not exist: $logDirPath"
  exit 0
fi

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmpDir="$scriptDir/tmp_record"

echo "Creating tmp dir for recording: $tmpDir"

if [ -d "$tmpDir" ]; then
 rm -fr $tmpDir
fi

ln -s $logDirPath $tmpDir

echo "Starting log playback and video recording"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$LD_LIBRARY_PATH
sdfName="playback_event_recorder"
ign gazebo -v 4 $sdfName.sdf

echo "Video recording ended. Shutting down playback"

pgrep -f $sdfName | xargs kill -9 &> /dev/null

videoDir=$(date +%s)
echo "Moving mp4 videos to dir: $videoDir"
mkdir $videoDir
mv *.mp4 $videoDir

# remove tmp dir
if [ -d "$tmpDir" ]; then
 rm -fr $tmpDir
fi
echo "Done"

