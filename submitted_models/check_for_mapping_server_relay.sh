#usage:
#all robot dirs: . check_for_mapping_server_relay.sh _
#robot dirs containing test: . check_for_mapping_server_relay.sh test
#robot dirs containing references to specific configs: grep -ir "wait darpa_robot"
dirs=$(find -maxdepth 1 | sed 's|^./||' | grep -Ev "check_for_mapping_server_relay.sh" | grep $1)
dir_list=($dirs)
for directory in "${dir_list[@]:0}"
do
  echo $directory
  cd $directory
  grep -ir "mapping_server_relays"
  cd ..
done
