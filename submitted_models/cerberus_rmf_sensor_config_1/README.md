# darpa_virtual_model_submit
Simulation models to submit to Darpa | CERBERUS Aerial Scouts (RMF - Resilient Micro Flyer)

## workspace setup
Follow these step to setup the Darpa Subt Virtual Environment: 

https://github.com/osrf/subt/wiki/Get%20Started

## add a new model

```
cd ~/subt_ws/src/subt/submitted_models

git clone https://github.com/unr-arl/darpa_virtual_model_submit.git cerberus_rmf_sensor_config_1

cd ~/subt_ws/
catkin_make install
source install/setup.bash
```

## launch the simulation
ign launch -v 4 cave_circuit.ign worldName:=cave_circuit_practice_01 robotName1:=rmf robotConfig1:=CERBERUS_RMF_SENSOR_CONFIG_1 localModel:=true

