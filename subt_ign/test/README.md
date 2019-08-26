## Scoring test

You will need to run this test manually after starting the
`tunnel_qual_ign` environment with the following command:

### Score test #1

~~~
ign launch -v 4 src/subt/subt_ign/test/test.ign \
  robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 \
  robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG_1
~~~

Make sure the test is built by running `make tests` from the build folder
or using `catkin_make tests`.

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
[ RUN      ] ScoreTest.TestScoring
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg] Scoring has Started
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 1
[Msg] Total score: 1
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 1
[Msg] Total score: 2
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 0
[Msg] Total score: 2
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 0
[Msg] Total score: 2
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 0
[Msg] Total score: 2
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg] Max score has been reached. Congratulations!
[Msg] Scoring has finished. Elapsed time: 2 seconds
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg] Max score has been reached. Congratulations!
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[       OK ] ScoreTest.TestScoring (18813 ms)
[----------] 1 test from ScoreTest (18813 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (18813 ms total)
[  PASSED  ] 1 test.

~~~

### Score test #2

~~~
ign launch -v 4 src/subt/subt_ign/test/test.ign \
  robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 \
  robotName2:=X2 robotConfig2:=X2_SENSOR_CONFIG_1
~~~

Make sure the test is built by running `make tests` from the build folder
or using `catkin_make tests`.

Then find the location of the `test_score` binary, which is in different
locations for different workspace types, and execute it manually:

~~~
# using a catkin_make workspace
${WS_PATH}/devel/lib/subt_ign/max_test_score
# using a colcon workspace
${WS_PATH}/build/subt_ign/devel/lib/subt_ign/max_test_score
~~~

It should give the following output:

~~~
[==========] Running 1 test from 1 test case.
[----------] Global test environment set-up.
[----------] 1 test from ScoreTest
[ RUN      ] ScoreTest.TestScoring
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg] Scoring has Started
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 1
[Msg] Total score: 1
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 1
[Msg] Total score: 2
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg]   [Total]: 1
[Msg] Total score: 3
[Msg] Max score has been reached. Congratulations!
[Msg] Scoring has finished. Elapsed time: 1 seconds
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[Msg] Max score has been reached. Congratulations!
[Dbg] [VisibilityRfModel.cc:80] Range: 12.3695, Exp: 2.5, TX: 20, RX: -47.3088
[       OK ] ScoreTest.TestScoring (18212 ms)
[----------] 1 test from ScoreTest (18212 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (18212 ms total)
[  PASSED  ] 1 test.

~~~
