# darpa_virtual_model_submit
Simulation models to submit to Darpa | CERBERUS Aerial Scouts (Alpha/Bravo/Charlie)

## workspace setup
Follow these step to setup the Darpa Subt Virtual Environment: 

https://bitbucket.org/osrf/subt/wiki/tutorials/SystemSetupInstall

## add a new model

```
cd ~/subt_ws/src/subt/submitted_models

git clone https://github.com/unr-arl/darpa_virtual_model_submit.git cerberus_m100_sensor_config_1

cd ~/subt_ws/
catkin_make install
source install/setup.bash
```

## launch the simulation
ign launch -v 4 src/subt/submitted_models/cerberus_m100_sensor_config_1/launch/m100_test.ign robotName:=m100 modelName:=CERBERUS_M100_SENSOR_CONFIG_1 localModel:=true

