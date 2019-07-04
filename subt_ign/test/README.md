## Scoring test

You will need to run this test manually after starting the
`virtual_stix` environment with the following command:

~~~
ign launch -v 4 virtual_stix.ign \
  robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 \
  robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG_1
~~~

Make sure the test is built by running `make tests` from the build folder
or using `catkin_make tests`.

After the world has loaded, use the ROS service call to start scoring:

~~~
rosservice call /subt/start "data: true"
~~~

Then find the location of the `test_score` binary, which is in different
locations for different workspace types, and execute it manually:

~~~
# using a catkin_make workspace
${WS_PATH}/devel/lib/subt_ign/test_score
# using a colcon workspace
${WS_PATH}/build/subt_ign/devel/lib/subt_ign/test_score
~~~

It should give the following output:

~~~
[==========] Running 1 test from 1 test case.
[----------] Global test environment set-up.
[----------] 1 test from ScoreTest
[ RUN      ] ScoreTest.ScoreAfterStart
[ INFO] [1562200405.891439394, 5.000000000]: Storing callback for X2:4000
[ INFO] [1562200405.893117720, 5.000000000]: Storing callback for broadcast:4000
[       OK ] ScoreTest.ScoreAfterStart (3925 ms)
[----------] 1 test from ScoreTest (3925 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (3925 ms total)
[  PASSED  ] 1 test.
~~~
