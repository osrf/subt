#!/usr/bin/env python

from subt_comms_test import CentralizedTesterRun, run_curses
import rospy
import rosservice

agents = [s.split('/')[1].split('_')[0] for s in rosservice.get_service_list() if s.find('create_peer') > 0]

rospy.loginfo('Found agents: {}'.format(agents))

if len(agents) <= 1:
    rospy.logwarn('Only found one agent, quitting')

else:

    testers = []
    for i in range(0, len(agents)):
        for j in range(i, len(agents)):
            if i != j:
                testers.append(CentralizedTesterRun(5, -1, agents[i], agents[j]))
    
    # syntax of CentralizedTesterRun is:
    # (frequency, ?, send agent name, , source id, destination id)
    
    # testers.append(CentralizedTesterRun(5, -1, 'X1', 'X3'))
    # testers.append(CentralizedTesterRun(5, -1, 'X1', 'X4'))
    # testers.append(CentralizedTesterRun(5, -1, 'X3', 'X2'))
    # testers.append(CentralizedTesterRun(5, -1, 'X4', 'X2'))        

    run_curses(testers)
