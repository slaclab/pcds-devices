"""
Aerotech devices.
"""
import contextlib
import logging

import numpy as np
import ophyd
from ophyd import Component as Cpt
from ophyd.signal import EpicsSignal, EpicsSignalRO, Signal

from .epics_motor import EpicsMotorInterface, MotorDisabledError
from .utils import stop_on_keyboardinterrupt

logger = logging.getLogger(__name__)


class AerotechMotorError(Exception):
    """
    Exception raised when an action requiring the motor not have an error is
    requested.
    """
    pass


class AerotechMotorFaulted(AerotechMotorError):
    """
    Exception raised when an action requiring the motor not be faulted is
    requested.
    """
    pass


class AerotechMotorStopped(AerotechMotorError):
    """
    Exception raised when an action requiring the motor to not be stopped is
    requested.
    """
    pass


class AerotechMotor(EpicsMotorInterface):
    """
    Aerotech motor class.

    Attributes
    ----------
    power : EpicsSignal
        Enables or disables power to the axis.

    retries : EpicsSignalRO
        Number of retries attempted.

    retries_max : EpicsSignal
        Maximum number of retries.

    retries_deadband : EpicsSignal
        Tolerance of each retry.

    axis_fault : EpicsSignalRO
        Fault readback for the motor.

    axis_status : EpicsSignalRO
        Status readback for the motor.

    clear_error : EpicsSignal
        Clear error signal.

    config : EpicsSignal
        Signal to reconfig the motor.

    print_set : bool
        Print verbose status information.
    """
    tab_component_names = True
    tab_whitelist = [
        'check_status', 'clear', 'disable', 'enable', 'get_enabled',
        'get_faulted', 'get_state', 'homf', 'homr', 'ready_motor', 'reconfig',
        'set_position', 'set_state', 'status', 'zero_all',
    ]

    _state_list = ("Stop", "Pause", "Move", "Go")

    # TODO: Remove when these have been figured out
    low_limit_switch = Cpt(Signal)
    high_limit_switch = Cpt(Signal)

    power = Cpt(EpicsSignal, ".CNEN")
    retries = Cpt(EpicsSignalRO, ".RCNT")
    retries_max = Cpt(EpicsSignal, ".RTRY")
    retries_deadband = Cpt(EpicsSignal, ".RDBD")
    axis_status = Cpt(EpicsSignalRO, ":AXIS_STATUS")
    axis_fault = Cpt(EpicsSignalRO, ":AXIS_FAULT")
    clear_error = Cpt(EpicsSignal, ":CLEAR")
    config = Cpt(EpicsSignal, ":CONFIG")
    zero_all_proc = Cpt(EpicsSignal, ":ZERO_P.PROC")
    home_forward = Cpt(EpicsSignal, ".HOMF")
    home_reverse = Cpt(EpicsSignal, ".HOMR")
    dial = Cpt(EpicsSignalRO, ".DRBV")
    state_component = Cpt(EpicsSignal, ".SPMG")

    def __init__(self, prefix, *args, name=None, **kwargs):
        super().__init__(prefix, *args, name=name, **kwargs)
        self.motor_done_move.unsubscribe(self._move_changed)
        self.user_readback.unsubscribe(self._pos_changed)
        self.configuration_attrs.append("power")
        self.print_set = True
        self.set_timeout = 10

    @contextlib.contextmanager
    def _print_settings_context(self, print_set):
        """
        Print status information while performing operations.

        Parameters
        ----------
        print_set : bool
            Print during operations.
        """
        old_value = self.print_set
        self.print_set = print_set
        yield
        self.print_set = old_value

    def _status_print(self, status, msg=None, print_set=True,
                      timeout=None, wait=True, reraise=False):
        """
        Internal method that optionally returns the status object and
        optionally prints a message about the set. If a message is passed but
        print_set is False then the message is logged at the debug level.

        Parameters
        ----------
        status : StatusObject or list
            The status object.

        msg : str or None, optional
            Message to print if print_set is True.

        wait : bool, optional
            Wait for the status to complete.

        reraise : bool, optional
            Raise the RuntimeError in the except.

        Returns
        -------
        Status
            The status object, for convenience.
        """
        try:
            # Wait for the status to complete
            if wait:
                status.wait(timeout)

            # Notify the user
            if msg is not None:
                if self.print_set:
                    logger.info(msg)
                else:
                    logger.debug(msg)
            return status

        # The operation failed for some reason
        except RuntimeError:
            error = "Operation completed, but reported an error."
            logger.error(error)
            if reraise:
                raise

    @stop_on_keyboardinterrupt
    def homf(self, check_status=True):
        """
        Home the motor forward.

        Parameters
        ----------
        check_status : bool, optional
            Check if the motors are in a valid state to move.

        Returns
        -------
        Status : StatusObject
            Status of the set.
        """
        # Check the motor status
        if check_status:
            self.check_status()
        status = self.home_forward.set(1, timeout=self.set_timeout)
        return self._status_print(status, f"Homing '{self.name}' forward.")

    @stop_on_keyboardinterrupt
    def homr(self, check_status=True):
        """
        Home the motor in reverse.

        Parameters
        ----------
        check_status : bool, optional
            Check if the motors are in a valid state to move.

        Returns
        -------
        Status : StatusObject
            Status of the set.
        """
        # Check the motor status
        if check_status:
            self.check_status()
        status = self.home_reverse.set(1, timeout=self.set_timeout)
        return self._status_print(
            status, f"Homing '{self.name}' in reverse.",
        )

    def check_status(self, position=None):
        """
        Checks the status of the motor to make sure it is ready to move. Checks
        the current position of the motor, and if a position is provided it
        also checks that position.

        Parameters
        ----------
        position : float, optional
            Position to check for validity.

        Raises
        ------
        MotorDisabledError
            If the motor is disabled.

        MotorFaulted
            If the motor is faulted.

        MotorStopped
            If the motor is stopped.
        """
        enabled = self.get_enabled()
        if not enabled:
            err = f"Motor '{self.name}' is currently disabled"
            logger.error(err)
            raise MotorDisabledError(err)

        faulted = self.get_faulted()
        if faulted:
            err = f"Motor '{self.name}' is currently faulted."
            logger.error(err)
            raise AerotechMotorFaulted(err)

        state = self.get_state()
        if state == "Stop":
            err = f"Motor '{self.name}' is currently stopped."
            logger.error(err)
            raise AerotechMotorStopped(err)

        # Check if the current position is valid
        self.check_value(self.position)
        # Check if the move position is valid
        if position:
            self.check_value(position)

    def set_position(self, position_des):
        """
        Sets the current position to be the desired position by changing the
        offset.

        Parameters
        ----------
        position_des : float
            The desired current position.
        """
        # Print to console or just to the log
        if self.print_set:
            log_level = logger.info
        else:
            log_level = logger.debug

        log_level("'%s' previous position: %s, offset: %s", self.position,
                  self.offset)
        self.offset += position_des - self.position
        log_level("'%s' New position: %s, offset: %s", self.position,
                  self.offset)

    def enable(self):
        """
        Enables the motor power.

        Returns
        -------
        Status
            The status object for setting the power signal.
        """
        status = self.power.set(1, timeout=self.set_timeout)
        return self._status_print(status, f"Enabled motor '{self.name}'.")

    def disable(self):
        """
        Disables the motor power.

        Returns
        -------
        Status
            The status object for setting the power signal.
        """
        status = self.power.set(0, timeout=self.set_timeout)
        return self._status_print(status, f"Disabled motor '{self.name}'.")

    def get_enabled(self, **kwargs):
        """
        Returns if the motor is curently enabled.

        Returns
        -------
        enabled : bool
            True if the motor is powered, False if not.
        """
        return bool(self.power.get(**kwargs))

    def clear(self):
        """
        Clears the motor error.

        Parameters
        ----------

        Returns
        -------
        Status
            The status object for setting the clear_error signal.
        """
        status = self.clear_error.set(1, timeout=self.set_timeout)
        return self._status_print(status, f"Cleared motor '{self.name}'.")

    def reconfig(self):
        """
        Re-configures the motor.

        Returns
        -------
        Status
            The status object for setting the config signal.
        """
        status = self.config.set(1, timeout=self.set_timeout)
        return self._status_print(status, f"Reconfigured motor '{self.name}'.")

    def get_faulted(self, **kwargs):
        """
        Returns the current fault with the motor.

        Returns
        -------
        faulted
            Fault enumeration.
        """
        return bool(self.axis_fault.get(**kwargs))

    def zero_all(self):
        """
        Sets the current position to be the zero position of the motor.

        Returns
        -------
        status : StatusObject
            Status object for the set.
        """
        status = self.zero_all_proc.set(1, timeout=self.set_timeout)
        return self._status_print(status, f"Zeroed motor '{self.name}'.")

    def get_state(self, **kwargs):
        """
        Returns the state of the motor. State can be one of the following:
            'Stop', 'Pause', 'Move', 'Go'

        Returns
        -------
        state : str
            The current state of the motor
        """
        return self._state_list[self.state_component.get(**kwargs)]

    def set_state(self, state):
        """
        Sets the state of the motor. State can be one of the following states
        or the index of the desired state:

            'Stop', 'Pause', 'Move', 'Go'

        Parameters
        ----------
        val : int or str

        Returns
        -------
        Status
            The status object for setting the state signal.
        """
        # Make sure it is in title case if it's a string
        val = state
        if isinstance(state, str):
            val = state.title()

        # Make sure it is a valid state or enum
        if val not in self._state_list + list(range(len(self._state_list))):
            error = f"Invalid state input: '{val}'."
            logger.error(error)
            raise ValueError(error)

        # Lets enforce it's a string or value
        status = self.state_component.set(val, timeout=self.set_timeout)

        return self._status_print(
            status,
            f"Changed state of '{self.name} to '{val}'.",
        )

    def ready_motor(self):
        """
        Sets the motor to the ready state by clearing any errors, enabling it,
        and setting the state to be 'Go'.

        Returns
        -------
        Status
            The status object for all the sets.
        """
        with self._print_settings_context(False):
            status = ophyd.status.AndStatus(
                self.clear(),
                self.enable(),
            )
            status = ophyd.status.AndStatus(
                status,
                self.set_state("Go"),
            )
        return self._status_print(
            status, f"Motor '{self.name}' is now ready to move.",
        )

    # def expert_screen(self, print_msg=True):
    #     """
    #     Launches the expert screen for the motor.
    #
    #     Parameters
    #     ----------
    #     print_msg : bool, optional
    #         Prints that the screen is being launched.
    #     """
    #     path = absolute_submodule_path(
    #         "hxrsnd/screens/motor_expert_screens.sh")
    #     if print_msg:
    #         logger.info("Launching expert screen.")
    #     os.system(f"{path} {self.prefix} aerotech &")

    def status(self, status="", offset=0, print_status=True, newline=False,
               short=False):
        """
        Returns the status of the device.

        Parameters
        ----------
        status : str, optional
            The string to append the status to.

        offset : int, optional
            Amount to offset each line of the status.

        print_status : bool, optional
            Determines whether the string is printed or returned.

        newline : bool, optional
            Adds a new line to the end of the string.

        Returns
        -------
        status : str
            Status string.
        """
        prefix = ' ' * offset
        indent = ' ' * (offset + 2)
        if short:
            dial = self.dial.get()
            status += (
                f"\n{prefix}"
                f"{self.name:<16}|"
                f"{self.position:^16.3f}|"
                f"{dial:^16.3f}"
            )
        else:
            position = np.round(self.wm(), 6)
            dial = np.round(self.dial.get(), 6)
            limits = (int(self.low_limit), int(self.high_limit))
            status += f"""
                {prefix}{self.name}
                {indent}PV: {self.prefix:>25}
                {indent}Enabled: {self.enabled:>20}
                {indent}Faulted: {self.faulted:>20}
                {indent}State: {self.get_state():>22}
                {indent}Position: {position:>19}
                {indent}Dial: {dial:>23}
                {indent}Limits: {str(limits):>21}"""

        if newline:
            status += "\n"

        return status
