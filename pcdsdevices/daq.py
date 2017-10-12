#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import functools
import threading
import copy
import enum
import logging

from ophyd.status import Status
from ophyd.flyers import FlyerInterface

logger = logging.getLogger(__name__)

try:
    import pydaq
except:
    logger.warning('pydaq not in environment. Will not be able to use DAQ!')


class Daq(FlyerInterface):
    """
    The LCLS1 DAQ as a flyer object. This uses the pydaq module to connect with
    a running daq instance, controlling it via socket commands. It can be used
    as a flyer in a bluesky plan to have the daq start at the beginning of the
    run and end at the end of the run. It has additional knobs for pausing
    and resuming acquisition if desired.

    Unlike a normal bluesky flyer, this has no data to report to the RunEngine
    on the collect call. No data will pass into the python layer from the daq.
    """
    state_enum = enum.Enum('pydaq state',
                           'Disconnected Connected Configured Open Running',
                           start=0)

    def __init__(self, name=None, platform=0, parent=None):
        super().__init__()
        self.name = name or 'daq'
        self.parent = parent
        self.control = None
        self.config = None
        self._host = os.uname()[1]
        self._plat = platform

    # Convenience properties
    @property
    def connected(self):
        return self.control is not None

    @property
    def configured(self):
        return self.config is not None

    @property
    def state(self):
        logger.debug('accessing Daq.state')
        if self.connected:
            num = self.control.state()
            return self.state_enum(num).name
        else:
            return 'Disconnected'

    # Wrapper to make sure we're connected
    def check_connect(f):
        @functools.wraps(f)
        def wrapper(self, *args, **kwargs):
            logger.debug('Checking for daq connection')
            if not self.connected:
                msg = 'DAQ is not connected. Attempting to connect...'
                logger.info(msg)
                self.connect()
            if self.connected:
                return f(self, *args, **kwargs)
            else:
                raise RuntimeError('Could not connect to DAQ.')
        return wrapper

    # Wrapper to make sure we've configured
    def check_config(f):
        @functools.wraps(f)
        def wrapper(self, *args, **kwargs):
            logger.debug('Checking for daq config')
            if self.configured:
                return f(self, *args, **kwargs)
            else:
                raise RuntimeError('DAQ is not configured.')
        return wrapper

    # Interactive methods
    def connect(self):
        """
        Connect to the DAQ instance, giving full control to the Python process.
        """
        logger.debug('Daq.connect()')
        if self.control is None:
            try:
                self.control = pydaq.Control(self._host, platform=self._plat)
                self.control.connect()
                msg = 'Connected to DAQ'
            except:
                del self.control
                self.control = None
                msg = ('Failed to connect to DAQ - check that it is up and '
                       'allocated.')
        else:
            msg = 'Connect requested, but already connected to DAQ'
        logger.info(msg)

    def disconnect(self):
        """
        Disconnect from the DAQ instance, giving control back to the GUI
        """
        logger.debug('Daq.disconnect()')
        if self.control is not None:
            self.control.disconnect()
        del self.control
        self.control = None
        self.config = None
        logger.info('DAQ is disconnected.')

    @check_connect
    def wait(self):
        """
        Pause the thread until the DAQ is done aquiring.
        """
        logger.debug('Daq.wait()')
        status = self.complete()
        status.wait()

    @check_config
    def begin(self, events=None, duration=None, wait=False):
        """
        Start the daq with the current configuration. Block until
        the daq has begun acquiring data. Optionally block until the daq has
        finished aquiring data.

        Parameters
        ----------
        events: int, optional
            Number events to stop acquisition at.

        duration: int, optional
            Time to run the daq in seconds.

        wait: bool, optional
            If switched to True, wait for the daq to finish aquiring data.
        """
        logger.debug('Daq.begin(events=%s, duration=%s, wait=%s',
                     events, duration, wait)
        begin_status = self.kickoff(events=events, duration=duration)
        begin_status.wait()
        if wait:
            self.wait()

    @check_connect
    def stop(self):
        """
        Stop the current acquisition, ending it early.
        """
        logger.debug('Daq.stop()')
        self.control.stop()

    @check_connect
    def end_run(self):
        """
        Stop the daq if it's running, then mark the run as finished.
        """
        logger.debug('Daq.end_run()')
        self.stop()
        self.control.endrun()

    # Flyer interface
    @check_config
    def kickoff(self, events=None, duration=None, use_l3t=None):
        """
        Begin acquisition. This method is non-blocking.

        Returns
        -------
        ready_status: DaqStatus
            Status that will be marked as 'done' when the daq has begun to
            record data.
        """
        logger.debug('Daq.kickoff()')

        def start_thread(control, status, events, duration, use_l3t):
            if any(map(lambda x: x is not None, events, duration)):
                kwargs = self._parse_config_args(events, duration, use_l3t)
                control.begin(**kwargs)
            else:
                control.begin()
            status._finished(success=True)
            logger.debug('Marked kickoff as complete')
        begin_status = DaqStatus(obj=self)
        watcher = threading.Thread(target=start_thread,
                                   args=(self.control, begin_status, events,
                                         duration))
        watcher.start()
        return begin_status

    def complete(self):
        """
        Return a status object that will be marked as 'done' when the DAQ has
        finished acquiring.

        Returns
        -------
        end_status: DaqStatus
        """
        logger.debug('Daq.complete()')

        def finish_thread(control, status):
            control.end()
            status._finished(success=True)
            logger.debug('Marked acquisition as complete')
        end_status = DaqStatus(obj=self)
        watcher = threading.Thread(target=finish_thread,
                                   args=(self.control, end_status))
        watcher.start()
        return end_status

    def collect(self):
        """
        As per the bluesky interface, this is a generator that is expected to
        output partial event documents. However, since we don't have any events
        to report to python, this will be a generator that immediately ends.
        """
        logger.debug('Daq.collect()')
        raise GeneratorExit

    def describe_collect(self):
        """
        As per the bluesky interface, this is how you interpret the null data
        from collect. There isn't anything here, as nothing will be collected.
        """
        logger.debug('Daq.describe_configuration()')
        return {}

    @check_connect
    def configure(self, events=None, duration=None, use_l3t=False,
                  record=False, controls=None):
        """
        Changes the daq's configuration for the next run.

        Parameters
        ----------
        events: int, optional
            If provided, the daq will run for this many events before stopping.
            If not provided, we'll use the duration argument instead.

        duration: int, optional
            If provided, the daq will run for this many seconds before
            stopping. If not provided, and events was also not provided, an
            error will be raised.

        use_l3t: bool, optional
            If True, the events argument will be reinterpreted to only count
            events that pass the level 3 trigger.

        record: bool, optional
            If True, we'll record the data. Otherwise, we'll run without
            recording.

        controls: list of tuple(str, value), optional
            If provided, these will make it into the data stream as control
            variables.

        Returns
        -------
        old, new: tuple of dict
        """
        logger.debug(('Daq.configure(events=%s, duration=%s, use_l3t=%s, '
                      'record=%s, controls=%s'),
                     events, duration, use_l3t, record, controls)
        if self.config is None:
            old = {}
        else:
            old = self.read_configuration()
        config_args = self._parse_config_args(events, duration, use_l3t,
                                              record, controls)
        try:
            # self.config should reflect exactly the arguments to configure,
            # this is different than the arguments that pydaq.Control expects
            self.control.configure(**config_args)
            self.config = dict(events=events, duration=duration,
                               use_l3t=use_l3t, record=record,
                               controls=controls)
            msg = 'Daq configured'
            logger.info(msg)
        except:
            self.config = None
            msg = 'Failed to configure!'
            logger.exception(msg)
            return old, {}
        new = self.read_configuration()
        return old, new

    def _parse_config_args(self, events, duration, use_l3t=None, record=None,
                           controls=None):
        config_args = {}
        if events is not None:
            if use_l3t is None and self.config is not None:
                use_l3t = self.config['use_l3t']
            if use_l3t:
                config_args['l3t_events'] = events
            else:
                config_args['events'] = events
        elif duration is not None:
            config_args['duration'] = duration
        else:
            raise RuntimeError('Either events or duration must be provided.')
        if record is not None:
            config_args['record'] = record
        if controls is not None:
            config_args['controls'] = controls
        return config_args

    @check_config
    def read_configuration(self):
        """
        Returns
        -------
        config: dict
            Mapping of config key to current configured value.
        """
        logger.debug('Daq.read_configuration()')
        return copy.deepcopy(self.config)

    def describe_configuration(self):
        """
        Returns
        -------
        config_desc: dict
            Mapping of config key to field metadata.
        """
        logger.debug('Daq.describe_configuration()')
        try:
            config = self.read_configuration()
            controls_shape = [len(config['control']), 2]
        except (RuntimeError, AttributeError):
            controls_shape = None
        return dict(events=dict(source='daq_events_in_run',
                                dtype='number',
                                shape=None),
                    duration=dict(source='daq_run_duration',
                                  dtype='number',
                                  shape=None),
                    use_l3t=dict(source='daq_use_l3trigger',
                                 dtype='number',
                                 shape=None),
                    record=dict(source='daq_record_run',
                                dtype='number',
                                shape=None),
                    controls=dict(source='daq_control_vars',
                                  dtype='array',
                                  shape=controls_shape))

    def unstage(self):
        """
        If the daq is running and the run has not ended, end it.

        Returns
        -------
        unstaged: list
            list of devices unstaged
        """
        logger.debug('Daq.unstage()')
        self.end_run()
        return super().unstage()

    def pause(self):
        """
        Stop acquiring data, but don't end the run.
        """
        logger.debug('Daq.pause()')
        if self.state() == 'Running':
            self.stop()

    def resume(self):
        """
        Continue acquiring data in a previously paused run.
        """
        logger.debug('Daq.resume()')
        if self.state() == 'Open':
            self.begin()

    def __del__(self):
        self.disconnect()


class DaqStatus(Status):
    def wait(self, timeout=None):
        self._wait_done = threading.Event()

        def cb(*args, **kwargs):
            self._wait_done.set()
        self.finished_cb = cb
        self._wait_done.wait(timeout=timeout)
