"""Test-only Python wall-time jump, without changing kernel semaphore deadlines."""
import os
import time

original_time = time.time
original_time_ns = time.time_ns


def offset_ns():
    with open(os.environ['CANE_TEST_WALL_OFFSET_FILE']) as source:
        return int(source.read())


time.time = lambda: original_time()+offset_ns()/1e9
time.time_ns = lambda: original_time_ns()+offset_ns()
