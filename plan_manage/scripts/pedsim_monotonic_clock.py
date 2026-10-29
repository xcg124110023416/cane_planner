#!/usr/bin/env python3
"""Clock source for an owned lightweight simulation, independent of later wall jumps."""
import time

import rospy
from rosgraph_msgs.msg import Clock


def main():
    # The supervisor sets this before starting any simulation nodes. Do not
    # change a running master's time mode from inside a publisher.
    rospy.init_node('pedsim_monotonic_clock', disable_rosout=True)
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Owned simulation clock requires /use_sim_time before startup')
    publisher = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
    # Anchor to a readable epoch once; all subsequent advancement and waiting
    # use monotonic time, including while the host corrects its realtime clock.
    epoch = time.time_ns()
    began = time.monotonic_ns()
    ready = False
    while not rospy.is_shutdown():
        elapsed = time.monotonic_ns()-began
        seconds, nanoseconds = divmod(epoch+elapsed, 1000000000)
        publisher.publish(Clock(rospy.Time(seconds, nanoseconds)))
        if not ready:
            rospy.set_param('~ready', True)
            ready = True
        # Do not use rospy.Rate: this node must advance its own clock. A 1 ms
        # tick stays well below LFPC's 20 ms execution timer period.
        time.sleep(.001)


if __name__ == '__main__':
    main()
