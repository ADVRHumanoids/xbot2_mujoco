"""Checks and the rate schedule shared by the IMU and odometry sensors."""

import math
import re


def check_topic(topic, kind):
    if not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', topic):
        raise ValueError(f'{kind} topic must be a valid ROS topic')


def check_frame(frame, kind):
    if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise ValueError(f'{kind} frame ids must be nonempty without a leading slash')


def check_rate(rate, kind):
    if isinstance(rate, bool) or not math.isfinite(rate) or rate <= 0:
        raise ValueError(f'{kind} rate must be finite and positive')


def check_unique_topics(configs, kind):
    topics = [cfg.topic.lstrip('/') for cfg in configs]
    for topic in topics:
        if topics.count(topic) > 1:
            raise ValueError(f'Duplicate {kind} topic: {topic}')


class RateSchedule:
    """Due at the step nearest each multiple of 1/rate (sim time is a sum of steps)."""

    def __init__(self, rate, timestep):
        self._period = 1.0 / rate
        self._tolerance = timestep / 2
        self._next = None

    def due(self, time):
        if self._next is None or time + self._period < self._next:
            self._next = time
        if time + self._tolerance < self._next:
            return False
        self._next += (math.floor((time + self._tolerance - self._next) / self._period) + 1) * self._period
        return True
