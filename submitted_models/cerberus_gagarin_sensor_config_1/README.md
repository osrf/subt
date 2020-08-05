# darpa_virtual_model_submit
Simulation models to submit to Darpa | CERBERUS Aerial Scouts (Gagarin)

## workspace setup
Follow these step to setup the Darpa Subt Virtual Environment: 

https://github.com/osrf/subt/wiki/Get%20Started

## add a new model

```
cd ~/subt_ws/src/subt/submitted_models

git clone https://github.com/unr-arl/darpa_virtual_model_submit.git cerberus_gagarin_sensor_config_1

cd ~/subt_ws/
catkin_make install
source install/setup.bash
```

## launch the simulation
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=gagarin robotConfig1:=CERBERUS_GAGARIN_SENSOR_CONFIG_1 localModel:=true

