#!/usr/bin/env python

from centralized_tester import *

import curses
import threading
import sys

class CentralizedTesterRun (threading.Thread):
    def __init__(self, send_rate, send_count, name1, name2):
        self.test = CentralizedTester(send_rate, send_count, name1, name2)
        threading.Thread.__init__ ( self )

    def run(self):
        self.test.Spin()

class RoundRobinSched (threading.Thread):
    def __init__(self, dwell_time, tester_pool):
        self.dwell_time = dwell_time
        self.tester_pool = tester_pool
        self.stop = False
        threading.Thread.__init__(self)

    def run(self):
        while not self.stop:
            for x in self.tester_pool:
                x.test.send = True
                rospy.rostime.sleep(self.dwell_time)
                x.test.send = False
                rospy.rostime.sleep(self.dwell_time)

def run_curses(testers):
    rr = RoundRobinSched(0.5, testers)

    rospy.init_node('CentralizedTester', anonymous=True)

    burst_size = 1024*1024

    col1 = 0  # Source
    col2 = 4 # Dest
    col3 = 10 # Percentage
    col4 = 31 # Sending
    col5 = 35 # Send rate
    col6 = 46 # Payload size
    col7 = 53 # tx bitrate
    col8 = 64 # rx bitrate
    col9 = 75 # Latency

    help_row = 2 + len(testers) + 3

    help_string="""
    Help:
      q              quit
      up/down (j/k)  change selection
      s              start sending selected data
      S              start sending all data
      g              Run round-robin sending
      r (R)          Reset statistics (for all)
      +/-            Increase/decrease transmit rate
      ]/[            Increase/decrease payload size
      B              Send burst of data
      }/{            Increase/decrease burst size
    """
    

    stdscr_orig = curses.initscr()
    stdscr = curses.newpad(help_row + 15, col9 + 15)
    curses.noecho()
    curses.cbreak()
    curses.start_color()
    curses.curs_set(0)
    stdscr.keypad(1)
    stdscr.nodelay(1)
    #curses.halfdelay(3)

    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_RED)
    curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_CYAN)

    for x in testers:
        x.start()

    done = False

    stdscr.addstr(0,0,'Comms Test Client')
    stdscr.addstr(1,0,'-'*(col8+11))
    stdscr.addstr(2,col1,'Src')
    stdscr.addstr(2,col2,'Dest')
    stdscr.addstr(2,col3,'% Recv')
    stdscr.addstr(2,col4,'Enb')
    stdscr.addstr(2,col5,'Rate (Hz)')
    stdscr.addstr(2,col6,'Bytes')
    stdscr.addstr(2,col7,'Tx (kbps)')
    stdscr.addstr(2,col8,'Rx (kbps)')
    stdscr.addstr(2,col9,'Latency (ms)')

    stdscr.addstr(help_row-2,0,'-'*(col8+11))
    stdscr.addstr(help_row-1, 0, 'Burst size (bytes): {}'.format(burst_size))
    stdscr.addstr(help_row, 8, help_string)

    selected_row_idx = 3
    selected_tester = None
    round_robin_run = False
    while not done:

        idx = 3
        for x in testers:

            if idx == selected_row_idx:
                color_selection = curses.color_pair(1)
                selected_tester = x
            else:
                color_selection = curses.color_pair(0)

            stdscr.addstr(idx,0,' '*(col9+11), color_selection)

            stats = x.test.GetStatistics()
            stdscr.addstr(idx,col1,'{}'.format(x.test.name1), color_selection)
            stdscr.addstr(idx,col2,'{}'.format(x.test.name2), color_selection)
            stdscr.addstr(idx,col3,'{:2.2f} ({}/{})'.format(stats[0]*100, stats[1], len(x.test.outgoing_data)), color_selection)
            # stdscr.addstr(idx,col4,'%d' % x.test.GetAverageRSSI(), color_selection)
            if x.test.send:
                stdscr.addstr(idx, col4, 'On', color_selection)
            else:
                stdscr.addstr(idx, col4, 'Off', color_selection)
            stdscr.addstr(idx, col5, '{:2.2f}'.format(x.test.send_rate), color_selection)
            stdscr.addstr(idx, col6, '{}'.format(x.test.payload_size), color_selection)
            # stdscr.addstr(idx, col7, '{:2.2f}'.format(x.test.send_rate * float(x.test.payload_size) / 1024.0), color_selection)
            stdscr.addstr(idx, col7, '{:2.2f}'.format(x.test.GetTXRate() / 1000.0), color_selection)
            stdscr.addstr(idx, col8, '{:2.2f}'.format(x.test.GetRXRate() / 1000.0), color_selection)
            stdscr.addstr(idx, col9, '{:2.2f}'.format(x.test.GetLatency() * 1000.0), color_selection)
            idx = idx + 1

        # stdscr.refresh()

        height, width = stdscr_orig.getmaxyx()
        stdscr.refresh(0, 0, 0, 0, height-1, width-1)

        # Key-event handling
        c = stdscr.getch()
        if c == ord('q'):
            done = True
            if round_robin_run:
                rr.stop = True
                rr.join()
            for x in testers:
                x.test.done = True
                x.join()
        if c == ord('s'):
            if selected_tester != None:
                if selected_tester.test.send:
                    selected_tester.test.send = False
                else:
                    selected_tester.test.send = True
        if c == ord('S'):
            for x in testers:
                if x.test.send:
                    x.test.send = False
                else:
                    x.test.send = True
        if c == ord('g'):
            if round_robin_run:
                round_robin_run = False
                rr.stop = True
                rr.join()
            else:
                round_robin_run = True
                rr.start()
        if c == ord('r'):
            if selected_tester != None:
                selected_tester.test.ResetStatistics()
        if c == ord('R'):
            for x in testers:
                x.test.ResetStatistics()
        elif c == curses.KEY_UP or c == ord('k'):
            selected_row_idx = selected_row_idx - 1
            if selected_row_idx < 3:
                selected_row_idx = 3
        elif c == curses.KEY_DOWN or c == ord('j'):
            selected_row_idx = selected_row_idx + 1
            if selected_row_idx >= 3+len(testers):
                selected_row_idx = 3 + len(testers) - 1

        if c == ord('+'):
            if selected_tester != None:
                selected_tester.test.UpdateRate(selected_tester.test.send_rate + 1)
        if c == ord('-'):
            if selected_tester != None:
                selected_tester.test.UpdateRate(max(selected_tester.test.send_rate - 1, 1.0))

        if c == ord(']'):
            if selected_tester != None:
                selected_tester.test.payload_size = min(selected_tester.test.payload_size+1, 1000000)

        if c == ord('['):
            if selected_tester != None:
                selected_tester.test.payload_size = max(selected_tester.test.payload_size-1, 0)

        if c == ord('}'):
            burst_size = burst_size+10
            stdscr.addstr(help_row-1, 0, 'Burst size (bytes): {}'.format(burst_size))

        if c == ord('{'):
            burst_size = max(burst_size - 10, 1)
            stdscr.addstr(help_row-1, 0, 'Burst size (bytes): {}'.format(burst_size))

        if c == ord('B'):
            if selected_tester != None:
                selected_tester.test.SendBurst(burst_size)


    curses.nocbreak(); stdscr.keypad(0); curses.echo()
    curses.endwin()
    
                
if __name__ == '__main__':

    testers = []
    # syntax of CentralizedTesterRun is:
    # (frequency, ?, send agent name, , source id, destination id)
    testers.append(CentralizedTesterRun(5, -1, 'X1', 'X2'))
    testers.append(CentralizedTesterRun(5, -1, 'X1', 'X3'))
    testers.append(CentralizedTesterRun(5, -1, 'X1', 'X4'))
    testers.append(CentralizedTesterRun(5, -1, 'X3', 'X2'))
    testers.append(CentralizedTesterRun(5, -1, 'X4', 'X2'))        

    run_curses(testers)
