#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import threading
import logging

from pcdsdevices.daq import Daq

logger = logging.getLogger(__name__)


class SimDaq(Daq):
    def connect(self):
        logger.debug('SimDaq.connect()')
        self.control = SimControl()
        self.control.connect()
        msg = 'Connected to sim DAQ'
        logger.info(msg)


class SimControl:
    _all_states = ['Disconnected', 'Connected', 'Configured', 'Open',
                   'Running']
    _state = _all_states[0]
    _transitions = dict(
        connect=dict(ignore=_all_states[1:],
                     begin=[_all_states[0]],
                     end=_all_states[1]),
        disconnect=dict(ignore=[],
                        begin=_all_states[0:3],
                        end=_all_states[0]),
        configure=dict(ignore=[],
                       begin=_all_states[1:3],
                       end=_all_states[2]),
        begin=dict(ignore=[],
                   begin=_all_states[2:4],
                   end=_all_states[4]),
        stop=dict(ignore=_all_states[0:4],
                  begin=[_all_states[4]],
                  end=_all_states[3]),
        endrun=dict(ignore=_all_states[0:3],
                    begin=_all_states[3:5],
                    end=_all_states[2])
    )

    def __init__(self, *args, **kwargs):
        self._duration = None
        self._time_remaining = 0
        self._done_flag = threading.Event()

    def _do_transition(self, transition):
        info = self._transitions[transition]
        if self._state in info['ignore']:
            return False
        elif self._state in info['begin']:
            self._state = info['end']
            return True
        else:
            err = 'Invalid SimControl transition {} from state {}'
            raise RuntimeError(err.format(transition, self._state))

    def state(self):
        logger.debug('SimControl.state()')
        return self._all_states.index(self._state)

    def connect(self):
        logger.debug('SimControl.connect()')
        self._do_transition('connect')

    def disconnect(self):
        logger.debug('SimControl.disconnect()')
        self._do_transition('disconnect')

    def configure(self, *, record=False, key=0, events=None, l1t_events=None,
                  l3t_events=None, duration=0, controls=None, monitors=None,
                  partition=None):
        logger.debug(('SimControl.configure(record=%s, key=%s, events=%s, '
                      'l1t_events=%s, l3t_events=%s, duration=%s, '
                      'controls=%s, monitors=%s, partition=%s)'),
                     record, key, events, l1t_events, l3t_events, duration,
                     controls, monitors, partition)
        if self._do_transition('configure'):
            dur = self._pick_duration(events, l1t_events, l3t_events, duration)
            if dur is None:
                raise RuntimeError('configure requires events or duration')
            else:
                self._duration = dur

    def begin(self, *, events=None, l1t_events=None, l3t_events=None,
              duration=None, controls=None, monitors=None):
        logger.debug(('SimControl.begin(events=%s, l1t_events=%s, '
                      'l3t_events=%s, duration=%s, controls=%s, '
                      'monitors=%s)'),
                     events, l1t_events, l3t_events, duration, controls,
                     monitors)
        if self._do_transition('begin'):
            dur = self._pick_duration(events, l1t_events, l3t_events, duration)
            if dur is None:
                err = 'SimControl stops here because pydaq segfaults here'
                raise RuntimeError(err)
            self._time_remaining = dur
            self._done_flag.clear()
            thr = threading.Thread(target=self._begin_thread, args=())
            thr.start()

    def _pick_duration(self, events, l1t_events, l3t_events, duration):
        logger.debug('SimControl._pick_duration(%s, %s, %s, %s)', events,
                     l1t_events, l3t_events, duration)
        for ev in (events, l1t_events, l3t_events):
            if ev is not None:
                if ev == 0:
                    return float('inf')
                else:
                    return ev / 120
            if duration is not None:
                return duration
            return None

    def stop(self):
        logger.debug('SimControl.stop()')
        self._do_transition('stop')
        self._time_remaining = 0
        self._done_flag.set()

    def endrun(self):
        logger.debug('SimControl.endrun()')
        self._do_transition('endrun')
        self._time_remaining = 0
        self._done_flag.set()

    def _begin_thread(self):
        logger.debug('SimControl._begin_thread()')
        logger.debug('%ss remaining', self._time_remaining)
        start = time.time()
        interrupted = False
        while self._time_remaining > 0:
            self._time_remaining -= 0.1
            if self._done_flag.wait(0.1):
                interrupted = True
                break
        if not interrupted:
            try:
                self.stop()
            except:
                pass
        end = time.time()
        logger.debug('%ss elapased in SimControl._begin_thread()',
                     end-start)

    def end(self):
        logger.debug('SimControl.end()')
        if self._state != 'Running':
            raise RuntimeError('Not running!')
        self._done_flag.wait()
