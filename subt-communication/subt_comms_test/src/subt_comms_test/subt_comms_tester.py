#!/usr/bin/env python

from subt_comms_test import CentralizedTesterRun, run_curses

testers = []
# syntax of XBeeTesterRun is:
# (frequency, ?, send agent name, , source id, destination id)
testers.append(CentralizedTesterRun(5, -1, 'X1', 'X2'))
testers.append(CentralizedTesterRun(5, -1, 'X1', 'X3'))
testers.append(CentralizedTesterRun(5, -1, 'X1', 'X4'))
testers.append(CentralizedTesterRun(5, -1, 'X3', 'X2'))
testers.append(CentralizedTesterRun(5, -1, 'X4', 'X2'))        

run_curses(testers)
