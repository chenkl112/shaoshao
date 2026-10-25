"""Optional external watchdog: ask our parent to dump stacks during a stall.

This diagnostic remains runnable when a native extension holds the parent's GIL.
It observes only a simulator heartbeat; it never produces motion commands.
"""
import mmap
import os
import signal
import struct
import sys
import time

parent=int(sys.argv[1])
with open(sys.argv[2],'rb') as stream:
    memory=mmap.mmap(stream.fileno(),16,access=mmap.ACCESS_READ)
    dumped=0.
    while os.getppid()==parent:
        stamp,moving=struct.unpack_from('dd',memory)
        if moving and stamp and stamp!=dumped and time.monotonic()-stamp>.12:
            os.kill(parent,signal.SIGUSR2)
            dumped=stamp
        time.sleep(.02)
