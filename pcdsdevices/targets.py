"""
Module for common target stage stack configurations.
"""
import logging
import numpy as np
from datetime import datetime
import os

from ophyd.device import Device
import json
import jsonschema
import yaml
from itertools import chain

from pcdsdevices.epics_motor import _GetMotorClass
from .interface import tweak_base


logger = logging.getLogger(__name__)


def StageStack(mdict, name):
    """
    Conveniencefunction for generating a stage stack device. Intended for
    bundling various motors into a single object. The function takes a
    dictionary of PVs and/or previously instantiated motor objects and bundles
    them together. If given a PV, The factory function attempts to determine
    the appropriate motor class from the given base PV; if this fails then it
    will attempt to create an EpicsMotor. Axes are given the same name as they
    are assigned in the provided dictionary. See examples below.

    Parameters
    ----------
    mdict : dictionary
        Dictionary of motor objects and or base PVs.

    name : str
        Name for the stack. Used to make a class name. No whitespace.

    Examples
    --------
    # Make a classic XYZ stack with two PVs and one motor object
    d = {'x': 'TST:MMS:01', 'y': 'TST:MMS:02', 'z': z_motor}
    xyz = StageStack(d, 'my_xyz')
    """
    cpts = {}
    for mname, mitem in mdict.items():
        # Check if this is a PV or motor object
        if issubclass(type(mitem), Device):  # Motor object
            cpts[mname] = mitem
        elif isinstance(mitem, (str)):  # PV
            mcls = _GetMotorClass(mitem)
            cpt = mcls(prefix=mitem, name=mname)
            cpts[mname] = cpt
        else:  # Something is wrong
            logger.warning("Unrecognized input {}. "
                           "Skipping axis {}.".format(mitem, mname))
    cls_name = name + '_StageStack'
    cls = type(cls_name, (object,), cpts)

    dev = cls()

    return dev


# Internal class
class GridAxis():
    """
    Class for axes that move in regularly spaced intervals.
    """
    def __init__(self, stage, spacing):
        assert isinstance(spacing, float), "Specify a float target spacing"
        self.stage = stage
        self.spacing = spacing

    def advance(self, nspaces, direction, wait=False):
        """
        Move the grid axis in the specified direction by (n_spacings * spacing)
        """
        assert direction in [1, -1], "Direction must be 1 or -1"
        assert nspaces >= 1, "n_targets must be >= 1"

        curr_pos = self.stage.wm()
        next_pos = curr_pos + direction*(nspaces*self.spacing)
        self.stage.mv(next_pos, wait=wait)


class XYTargetGrid():
    """
    Class methods for managing a target grid oriented normal to the beam, with
    regular X-Y spacing between targets.

    Parameters
    ----------
    x : Ophyd motor object
        X stage (LUSI coords.) to be using in the stack

    y : Ophyd motor object
        Y stage (LUSI coords.) to be using in the stack

    x_init : float
        The initial position for the x motor. Defines the x-position of target
        'zero'.

    x_spacing : float
        The nominal spacing between targets in the x-direction.

    x_comp : float (optional)
        A parameter to account for skew in target position due to non-ideal
        mounting. This skew is assumed to be identical between targets.

    y_init : float
        The initial position for the y motor. Defines the y-position of target
        'zero'.

    y_spacing : float
        The nominal spacing between targets in the y-direction.

    y_comp : float (optional)
        A parameter to account for skew in target position due to non-ideal
        mounting. This skew is assumed to be identical between targets.

    Examples
    --------
    # Make a target stack with targets spaced in a 1.0mm square grid, starting
    # at motor positions (0.0, 0.0), with no skew.

    xygrid = XYTargetGrid(x=x_motor, y=y_motor, x_init=0.0, y_init=0.0,
                          x_spacing=1.0, y_spacing=1.0)

    # Make a target stack as above, but with some skew in x and y.

    xygrid = XYTargetGrid(x=x_motor, y=y_motor, x_init=0.0, y_init=0.0,
                          x_spacing=1.0, y_spacing=1.0, x_comp=0.05,
                          y_comp=0.01)
    """
    def __init__(self, x=None, y=None, x_init=None, x_spacing=None,
                 x_comp=0.0, y_init=None, y_spacing=None, y_comp=0.0,
                 name=None):

        self.x_init = x_init
        self.x_spacing = x_spacing
        self.x_comp = x_comp

        self.y_init = y_init
        self.y_spacing = y_spacing
        self.y_comp = y_comp

        d = {'x': x, 'y': y}
        self._stack = StageStack(d, name)

        self.x = self._stack.x
        self._xgrid = GridAxis(self.x, self.x_spacing)
        self.y = self._stack.y
        self._ygrid = GridAxis(self.y, self.y_spacing)

        # Treat compensation axes like additional grid axes
        if self.x_comp:
            self._x_comp_axis = GridAxis(self.x, self.x_comp)

        if self.y_comp:
            self._y_comp_axis = GridAxis(self.y, self.y_comp)

    def wm(self):
        """
        Return current position of X and Y axes as a dictionary, i.e.
        {x: <x_position>, y: <y_position>}.
        """
        return {'x': self.x.wm(), 'y': self.y.wm()}

    def reset(self, wait=False):
        """
        Return to the defined initial position (x_init, y_init). Should be
        called during experiment setup prior to using other class methods to
        initialize the position.

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.
        """
        self.x.mv(self.x_init, wait=wait)
        self.y.mv(self.y_init, wait=wait)

    def next(self, nspaces=1, wait=False):
        """
        Move forward (in X) by specified integer number of targets.

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.

        nspaces : int (default = 1)
            Number of spaces to move "forward" on x-axis.
        """
        self._xgrid.advance(nspaces, 1, wait=wait)
        if self.y_comp:
            self._y_comp_axis.advance(nspaces, 1, wait=wait)

    def back(self, nspaces=1, wait=False):
        """
        Move backward (in X) by specified integer number of targets.

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.

        nspaces : int (default = 1)
            Number of spaces to move "backward" on x-axis.
        """
        self._xgrid.advance(nspaces, -1, wait=wait)
        if self.y_comp:
            self._y_comp_axis.advance(nspaces, -1, wait=wait)

    def up(self, nspaces=1, wait=False):
        """
        Move to higher target position by specified integer number of targets
        (stage moves down).

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.

        nspaces : int (default = 1)
            Number of spaces to move "up" on y-axis.
        """
        self._ygrid.advance(nspaces, 1, wait=wait)
        if self.x_comp:
            self._x_comp_axis.advance(nspaces, 1, wait=wait)

    def down(self, nspaces=1, wait=False):
        """
        Move to lower target position by specified integer number of targets
        (stage moves up).

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.

        nspaces : int (default = 1)
            Number of spaces to move "down" on y-axis.
        """
        self._ygrid.advance(nspaces, -1, wait=wait)
        if self.x_comp:
            self._x_comp_axis.advance(nspaces, -1, wait=wait)

    def move(self, nxspaces, nyspaces, wait=False):
        """
        Move to a specific target position given by (xspaces, yspaces)
        from the defined initial position. Includes compensation if defined.

        Parameters:
        -----------
        wait : bool (default = False)
            Flag to wait for movement to complete before returning.

        nxspaces : int (default = 1)
            Number of spaces to move on x-axis.

        nyspaces : int (default = 1)
            Number of spaces to move on y-axis.
        """
        xpos = self.x_init + self.x_spacing*nxspaces + self.x_comp*nyspaces
        ypos = self.y_init + self.y_spacing*nyspaces + self.y_comp*nxspaces

        self.x.mv(xpos, wait=wait)
        self.y.mv(ypos, wait=wait)


class XYGridStage():
    """
    Class that helps support multiple samples on a mount for an XY Grid setup.

    We could have multiple samples mounted in a setup. This class helps
    figuring out the samples xy coordinates for each target on the grid,
    maps points accordingly, and saves those records in a file.

    Parameters
    ----------
    name : str, otpinal
        Name of the XYGridStage object.
    x_motor : str or motor object
        Epics PV prefix for x motor, or a motor object.
    y_motor : str or motor object
        Epics PV prefix for y motor, or a motor object.
    m_points : int
        Number of rows the grid has, used to determine the coordinate
        points, where for e.g.: `(0, m_points)` would represent the top right
        corner of the desired sample grid.
    n_points : int
        Number of columns the grid has, used to determine the coordinate
        points, where for e.g.: `(n_points, 0)` would represent the bottom
        left corner of the desired sample grid.
    path : str
        Path to an `yaml` file where to save the grid patterns for
        different samples.
    """

    sample_schema = json.loads("""
    {
        "type": "object",
        "properties": {
            "time_created": {"type": "string"},
            "top_left": {"type": "array", "items": {"type": "number"}},
            "top_rigt": {"type": "array", "items": {"type": "number"}},
            "bottom_right": {"type": "array", "items": {"type": "number"}},
            "bottom_left": {"type": "array", "items": {"type": "number"}},
            "M": {"type": "number"},
            "N": {"type": "number"},
            "coefficients": {"type": "array", "items": {"type": "number"}},
            "xx" : {"type": "array", "items": {"type": "object"}},
            "yy" : {"type": "array", "items": {"type": "object"}}
        },
        "required": ["time_created", "top_left", "top_right", "bottom_right",
        "bottom_left", "coefficients", "xx", "yy"],
        "additionalProperties": true
    }
    """)

    def __init__(self, x_motor, y_motor, m_points, n_points, path):
        self._path = path
        self._m_points = m_points
        self._n_points = n_points
        d = {'x': x_motor, 'y': y_motor}
        self._stack = StageStack(d, 'xy_stage_grid')
        self.x = self._stack.x
        self.y = self._stack.y
        # TODO: assert here for a valid path, also valid yaml file
        # assert os.path.exists(path)
        self._coefficients = []
        self._current_sample = ''
        self._positions_x = []
        self._positions_y = []

    @property
    def m_n_points(self):
        """
        Get the current m and n points.

        The m and n points represent the number of grid points on the current
        grid, `m` -> representing the number of rows, and `n` -> representing
        the number of columns.

        Returns
        -------
        m_points, n_points : tuple
            The number of grid points on the x and y axis.
            E.g.: `10, 5` -> 10 rows by 5 columns grid.
        """
        return self._m_points, self._n_points

    @m_n_points.setter
    def m_n_points(self, m_n_values):
        """
        Set m and n points.

        Set the m value representing the number of grid points in x direction,
        and n value representing the number of grid points in the y direction.

        Parameters
        ----------
        m_n_values : tuple
            The number of grid points on the x and y axis respectively.

        Examples
        --------
        >>> xy_grid.m_n_points = 10, 10
        """
        try:
            self._m_points, self._n_points = m_n_values
        except Exception:
            err_msg = ("Please pass an iterable with two items for m points"
                       " and n points respectively.")
            raise Exception(err_msg)

    @property
    def coefficients(self):
        """
        Get the current coefficients if any.

        These coefficients are calculated from the projective transformation.
        Knowing the coefficients, the x an y values can be determined.

        Returns
        -------
        coefficients : list
            Array of 8 projective transformation coefficients.
            First 4 -> alpha, last 4 -> beta
        """
        return self._coefficients

    @coefficients.setter
    def coefficients(self, coefficients):
        """
        Set the current coefficients.

        These coefficients are calculated from the projective transformation.
        Knowing the coefficients, the x an y values can be determined.

        Parameters
        ----------
        coefficients : array
            Array of 8 projective transformation coefficients.
            First 4 -> alpha, last 4 -> beta
        """
        self._coefficients = coefficients

    @property
    def positions_x(self):
        """
        Get the current mapped x positions if any.

        These positions are set when `map_points` method is called.

        Returns
        -------
        positions_x : list
            List of all target x positions mapped for a sample.
        """
        return self._positions_x

    @property
    def positions_y(self):
        """
        Get the current mapped y positions if any.

        These positions are set when `map_points` method is called.

        Returns
        -------
        positions_y : list
            List of all target y positions mapped for a sample.
        """
        return self._positions_y

    @positions_x.setter
    def positions_x(self, x_positions):
        """
        Set the x positions.

        These positions are be saved in the sample file when `save_grid`
        method is called.

        Parameters
        ----------
        x_positions : list
            List of all the x positions.
        """
        self._positions_x = x_positions

    @positions_y.setter
    def positions_y(self, y_positions):
        """
        Set the y positions.

        These positions are be saved in the sample file when `save_grid`
        method is called.

        Parameters
        ----------
        y_positions : list
            List of all the y positions.
        """
        self._positions_y = y_positions

    def tweak(self):
        """
        Call the tweak function from `pcdsdevice.interface`.

        Use the Left arrow to move x motor left.
        Use the Right arrow to move x motor right.
        Use the Down arrow to move y motor down.
        Use the Up arrow to move y motor up.
        Use Shift & Up arrow to scale*2.
        Use Shift & Down arrow to scale/2.
        Press q to quit.
        """
        tweak_base(self.x, self.y)

    def set_presets(self):
        """
        Save four preset coordinate points.

        These are the coordinates from the four corners of the
        wanted/defined grid. The points for these coordinates shuld be taken
        from the middle of the four targets that are encasing the grid.

        The user will be asked to define the coordinates using the `tweak`
        method.

        Examples
        --------
        # Press q when ready to save the coordinates
        >>> xy.set_presets()
        Setting coordinates for (0, 0) top left corner:

        0.0000, : 0.0000, scale: 0.1

        Setting coordinates for (0, M) top right corner:

        10.0000, : 0.0000, scale: 0.1

        Setting coordinates for (N, M) bottom right corner:

        10.0000, : -10.0000, scale: 0.1

        Setting coordinates for (N, 0) bottom left corner:

        -0.0000, : -10.0000, scale: 0.1
        """
        # check to see the the presets are setup
        if not hasattr(self.x.presets, 'add_hutch'):
            raise AttributeError('No folder setup for motor presets. '
                                 'Please add a location to save the positions '
                                 'to, using setup_preset_paths from '
                                 'pcdsdevices.interface to save the position.')

        print('\nSetting coordinates for (0, 0) top left corner: \n')
        self.tweak()
        pos = [self.x.position, self.y.position]
        print('\nSetting coordinates for (0, M) top right corner: \n')
        self.tweak()
        pos.extend([self.x.position, self.y.position])
        print('\nSetting coordinates for (N, M) bottom right corner: \n')
        self.tweak()
        pos.extend([self.x.position, self.y.position])
        print('\nSetting coordinates for (N, 0) bottom left corner: \n')
        self.tweak()
        pos.extend([self.x.position, self.y.position])
        # create presets
        # corner (0, 0)
        self.x.presets.add_hutch(value=pos[0], name="x_top_left")
        self.y.presets.add_hutch(value=pos[1], name="y_top_left")
        # corner (0, M)
        self.x.presets.add_hutch(value=pos[2], name="x_top_right")
        self.y.presets.add_hutch(value=pos[3], name="y_top_right")
        # corner (M, N)
        self.x.presets.add_hutch(value=pos[4], name="x_bottom_right")
        self.y.presets.add_hutch(value=pos[5], name="y_bottom_right")
        # corner (N, 0)
        self.x.presets.add_hutch(value=pos[6], name="x_bottom_left")
        self.y.presets.add_hutch(value=pos[7], name="y_bottom_left")

    def get_presets(self):
        """
        Get the saved presets if any.

        Examples
        --------
        >>> xy.get_presets()
        ((0, 0),
        (9.99999999999998, 0),
        (9.99999999999998, -9.99999999999998),
        (-6.38378239159465e-16, -9.99999999999998))

        Returns
        -------
        coord : tuple
            Four coordinate positions.
            (top_left, top_right, bottom_right, bottom_left)
        """
        try:
            top_left = (self.x.presets.positions.x_top_left.pos,
                        self.y.presets.positions.y_top_left.pos)
            # corner (0, M)
            top_right = (self.x.presets.positions.x_top_right.pos,
                         self.y.presets.positions.y_top_right.pos)
            # corner (M, N)
            bottom_right = (self.x.presets.positions.x_bottom_right.pos,
                            self.y.presets.positions.y_bottom_right.pos)
            # corner (N, 0)
            bottom_left = (self.x.presets.positions.x_bottom_left.pos,
                           self.y.presets.positions.y_bottom_left.pos)
            return top_left, top_right, bottom_right, bottom_left
        except Exception:
            logger.warning('Could not get presets, try to set_presets.')

    def get_samples(self, path=None):
        """
        Get all the available sample grids names that are currently saved.

        Returns
        -------
        samples : list
            List of strings of all the sample names available.
        """
        samples = []
        path = path or self._path
        with os.scandir(path) as entries:
            for entry in entries:
                if entry.is_file():
                    samples.append(entry.name.split('.yml')[0])
        return samples

    @property
    def current_sample(self):
        """
        Get the current sample that is loaded.

        Returns
        -------
        sample : dict
            Dictionary with current sample information.
        """
        return self._current_sample

    @current_sample.setter
    def current_sample(self, sample_name):
        """
        Set the current sample.

        Parameters
        ----------
        sample_name : str
            The name of the sample to be set as current one.
        """
        self._current_sample = str(sample_name)

    @property
    def status(self):
        x_index = ''
        y_index = ''
        x_pos = self.x.position
        y_pos = self.y.position
        data = self.get_sample_data(self.current_sample)
        try:
            xx = data['xx']
            yy = data['yy']

            x_index = next((index for (index, d) in enumerate(xx)
                            if np.isclose(d["pos"], x_pos)))
            y_index = next((index for (index, d) in enumerate(yy)
                            if np.isclose(d["pos"], y_pos)))
        except Exception:
            logger.warning('Could not determine the m n points from position.')
        n_points = self.m_n_points[1]

        x_index += 1
        m = int(np.ceil(x_index / n_points))
        res = np.mod(x_index, 2 * n_points)
        if res == 0:
            n = 1
        elif res <= n_points:
            n = res
        else:
            n = 2 * n_points - (res + 1)

        if x_index != '':
            # to start from 1 instead of 0
            x_index = n
        if y_index != '':
            y_index = m
        lines = []
        sample = f'current_sample: {self.current_sample}'
        grid = f'grid M x N: {self.m_n_points}'
        m_n = f'current m, n : {y_index, x_index}'
        lines.extend([sample, grid, m_n])

        print('\n'.join(lines))

    @property
    def current_sample_path(self):
        """
        Get the current path for the sample that is loaded.

        Returns
        -------
        sample: dict
        Dictionary with current sample information.
        """
        if self._current_sample != '':
            return os.path.join(self._path, self._current_sample + '.yml')
        raise ValueError('No current sample loaded, please use load() first.')

    def load(self, sample_name, path=None):
        """
        Get the sample information and populate these parameters.

        This function displays the parameters for the sample just loaded, but
        also populates them, in the sense that it sets the current
        `coefficients` and current `m, n` values.

        Parameters
        ----------
        sample_name : str
            Name of the sample to load.
        path : str, optional
            Path where the samples yaml file exists.
        """
        path = path or self._path
        entry = os.path.join(path, sample_name + '.yml')
        m_points, n_points, coeffs = self.get_sample_map_info(
                                            str(sample_name), path=entry)
        self.m_n_points = m_points, n_points
        self.coefficients = coeffs
        # make this sample the current one
        self.current_sample = str(sample_name)

    def get_sample_data(self, sample_name, path=None):
        """
        Get the information for a saved sample.

        Parameters
        ----------
        sample_name : str
            The sample name that we want the grid for. To see current
            available samples call `mapped_grids`
        path : str, optional
            Path to the `.yml` file. Defaults to the path defined when
            creating this object.

        Returns
        -------
        data : dictionary
            Dictionary of all the information for a saved sample, or empty
            dictionary if troubles getting the sample.

        Examples
        --------
        >>> get_sample('sample1')
        {'time_created': '2021-01-06 11:43:40.701095',
        'top_left': [0, 0],
        'top_right': [4.0, -1.0],
        'bottom_right': [4.4, -3.5],
        'bottom_left': [1.0, -3.0],
        'M': 10,
        'N': 10,
        'coefficients': [1.1686746987951824,
        -0.3855421686746996,
        -9.730859023513261e-15,
        -0.29216867469879476,
        1.1566265060240974,
        6.281563288265657e-16,
        0.042168674698794054,
        -0.05220883534136586],
        xx:
        ...
        yy:
        ...}
        """
        path = path or os.path.join(self._path, sample_name + '.yml')
        data = None
        with open(path) as sample_file:
            try:
                data = yaml.safe_load(sample_file)
            except yaml.YAMLError as err:
                logger.error('Error when loading the samples yaml file: %s',
                             err)
                raise err
        if data is None:
            logger.warning('The file is empty, no sample grid yet. '
                           'Please use `save_presets` to insert grids '
                           'in the file.')
            return {}
        try:
            return data[str(sample_name)]
        except Exception:
            logger.error('The sample %s might not exist in the file.',
                         sample_name)
            return {}

    def get_sample_map_info(self, sample_name, path=None):
        """
        Given a sample name, get the m and n points, as well as the coeffs.

        Parameters
        ----------
        sample_name : str
            The name of the sample to get the mapped points from. To see the
            available mapped samples call the `mapped_samples()` method.
        path : str, optional
            Path to the samples yaml file.
        """
        path = path or os.path.join(self._path, sample_name + '.yml')
        sample = self.get_sample_data(str(sample_name), path=path)
        coeffs = []
        m_points, n_points = 0, 0
        if sample:
            try:
                coeffs = sample["coefficients"]
                m_points = sample['M']
                n_points = sample['N']
            except Exception as ex:
                logger.error('Something went wrong when getting the '
                             'information for sample %s. %s', sample_name, ex)
                raise ex
        else:
            err_msg = ('This sample probably does not exist. Please call'
                       ' mapped_samples() to see which ones are available.')
            logger.error(err_msg)
            raise Exception(err_msg)

        return m_points, n_points, coeffs

    def save_grid(self, sample_name, path=None):
        """
        Save a grid file of mapped points for a sample.

        This will save the date it was created, along with the sample name,
        the m and n points, the coordinates for the four corners, and the
        coefficients that will help get the x and y position on the grid.

        If an existing name for a sample is saved again, it will override
        the information for that samplefile keeping the status of the targets.
        When overriding a sample, this is assuming that a re-calibration was
        needed for that sample, so in case we have already shot targets from
        that sample - we want to keep track of that.

        Parameters
        ----------
        sample_name : str
            A name to identify the sample grid, should be snake_case style.
        path : str, optional
            Path to the sample folder where this sample will be saved.
            Defaults to the path defined when creating this object.

        Examples
        --------
        >>> save_grid('sample_1')
        """
        path = path or self._path
        entry = os.path.join(path, sample_name + '.yml')
        now = str(datetime.now())
        top_left, top_right, bottom_right, bottom_left = [], [], [], []
        if self.get_presets():
            top_left, top_right, bottom_right, bottom_left = self.get_presets()
        xx, yy = self.positions_x, self.positions_y
        flat_xx, flat_yy = [], []
        if xx and yy:
            flat_xx = [float(x) for x in xx]
            flat_yy = [float(y) for y in yy]
            # add False to each target to indicate they
            # have not been shot yet
            flat_xx = [{"pos": x, "status": False} for x in flat_xx]
            flat_yy = [{"pos": y, "status": False} for y in flat_yy]
        m_points, n_points = self.m_n_points
        coefficients = self.coefficients
        data = {sample_name: {"time_created": now,
                              "top_left": list(top_left),
                              "top_right": list(top_right),
                              "bottom_right": list(bottom_right),
                              "bottom_left": list(bottom_left),
                              "M": m_points,  # number of rows
                              "N": n_points,  # number of columns
                              "coefficients": coefficients,
                              "xx": flat_xx,
                              "yy": flat_yy}}
        try:
            jsonschema.validate(data[sample_name], self.sample_schema)
        except jsonschema.exceptions.ValidationError as err:
            logger.warning('Invalid input: %s', err)
            raise err
        # entry = os.path.join(path, sample_name + '.yml')
        # if this is an existing file, overrite the info but keep the statuses
        if os.path.isfile(entry):
            with open(entry) as sample_file:
                yaml_dict = yaml.safe_load(sample_file)
                sample = yaml_dict[sample_name]
                # when overriding the same sample, this is assuming that a
                # re-calibration was done - so keep the previous statuses.
                temp_xx = sample['xx']
                temp_yy = sample['yy']
                temp_x_status = [i['status'] for i in temp_xx]
                temp_y_status = [i['status'] for i in temp_yy]
                # update the current data statuses with previous ones
                for xd, status in zip(data[sample_name]['xx'], temp_x_status):
                    xd.update((k, status)
                              for k, v in xd.items() if k == 'status')
                for yd, status in zip(data[sample_name]['yy'], temp_y_status):
                    yd.update((k, status)
                              for k, v in yd.items() if k == 'status')
                yaml_dict.update(data)
            with open(entry, 'w') as sample_file:
                yaml.safe_dump(data, sample_file,
                               sort_keys=False, default_flow_style=False)
        else:
            # create a new file
            with open(entry, 'w') as sample_file:
                yaml.safe_dump(data, sample_file,
                               sort_keys=False, default_flow_style=False)

    def reset_statuses(self, sample_name, path=None):
        """
        Reset the statuses to `False` for the sample targets.

        Parameters
        ----------
        sample_name : str
            A name to identify the sample grid, should be snake_case style.
        path : str, optional
            Path to the `.yml` file. Defaults to the path defined when
            creating this object.
        """
        path = path or os.path.join(self._path, sample_name + '.yml')
        with open(path) as sample_file:
            yaml_dict = yaml.safe_load(sample_file) or {}
            sample = yaml_dict.get(sample_name)
            if sample:
                for xd in sample.get('xx'):
                    xd.update((k, False)
                              for k, v in xd.items() if k == 'status')
                for yd in sample.get('yy'):
                    yd.update((k, False)
                              for k, v in yd.items() if k == 'status')
                yaml_dict[sample_name].update(sample)
            else:
                raise ValueError('Could not find this sample name in the file:'
                                 f' {sample}')
        with open(path, 'w') as sample_file:
            yaml.safe_dump(yaml_dict, sample_file,
                           sort_keys=False, default_flow_style=False)

    def map_points(self, snake_like=True, top_left=None, top_right=None,
                   bottom_right=None, bottom_left=None, m_rows=None,
                   n_columns=None):
        """
        Map the points of a quadrilateral.

        Given the 4 corners coordinates of a grid, and the numbers of rows and
        columns, map all the sample positions in 2-d coordinates.

        Parameters
        ----------
        snake_like : bool
            Indicates if the points should be saved in a snake_like pattern.
        top_left : tuple, optional
            (x, y) coordinates of the top left corner
        top_right : tuple, optional
            (x, y) coordinates of the top right corner
        bottom_right : tuple, optional
            (x, y) coordinates of the bottom right corner
        bottom_left : tuple, optional
            (x, y) coordinates of the bottom left corner
        m_rows : int, optional
            Number of rows the grid has.
        n_columns : int, optional
            Number of columns the grid has.

        Returns
        -------
        xx, yy : tuple
            Tuple of two lists with all mapped points for x and y positions in
            the grid.
        """
        top_left = top_left or self.get_presets()[0]
        top_right = top_right or self.get_presets()[1]
        bottom_right = bottom_right or self.get_presets()[2]
        bottom_left = bottom_left or self.get_presets()[3]

        if any(v is None for v in [top_left, top_right, bottom_right,
                                   bottom_left]):
            raise ValueError('Could not get presets, make sure you set presets'
                             ' first using the `set_presets` method.')
        rows = m_rows or self.m_n_points[0]
        columns = n_columns or self.m_n_points[1]

        a_coeffs, b_coeffs = mesh_interpolation(top_left, top_right,
                                                bottom_right, bottom_left)
        self.coefficients = a_coeffs.tolist() + b_coeffs.tolist()
        x_points, y_points = [], []

        xx, yy = get_unit_meshgrid(m_rows=rows, n_columns=columns)

        # return x_points, y_points
        for rowx, rowy in zip(xx, yy):
            for x, y in zip(rowx, rowy):
                i, j = convert_to_physical(a_coeffs=a_coeffs,
                                           b_coeffs=b_coeffs,
                                           logic_x=x, logic_y=y)
                x_points.append(i)
                y_points.append(j)
        if snake_like:
            x_points = snake_grid_list(
                np.array(x_points).reshape(rows, columns))
            y_points = snake_grid_list(
                np.array(y_points).reshape(rows, columns))
        self.positions_x = x_points
        self.positions_y = y_points
        return x_points, y_points

    def is_target_shot(self, m, n, sample=None, path=None):
        """
        Check to see if the target position at MxN is shot.

        Parameters
        ----------
        sample_name : str, optional
            The name of the sample to get the mapped points from. To see the
            available mapped samples call the `mapped_samples()` method.
        m_point : int
            Represents the row value of the point we want the position for.
        n_point : int
            Represents the column value of the point we want the position for.
        path : str, optional
            Sample path.

        Returns
        -------
        is_shot : bool
            Indicates is target is shot or not.
        """
        sample = sample or self.current_sample
        path = path or self.current_sample_path
        x, y = self.compute_mapped_point(m_row=m,
                                         n_column=n,
                                         sample_name=sample, path=path)

        data = self.get_sample_data(sample)
        xx = data.get('xx')
        x_status = None
        # one value should be enough
        # TODO: this is assuming that none of the points will be the unique.
        if xx is not None:
            x_status = next((item['status']
                             for item in xx if item['pos'] == x), None)
        return x_status

    def compute_mapped_point(self, m_row, n_column, sample_name=None,
                             path=None, compute_all=False):
        """
        For a given sample, compute the x, y position for M and N respecively.

        Parameters
        ----------
        sample_name : str
            The name of the sample to get the mapped points from. To see the
            available mapped samples call the `mapped_samples()` method.
        m_point : int
            Represents the row value of the point we want the position for.
        n_point : int
            Represents the column value of the point we want the position for.
        compute_all : boolean, optional
            If `True` all the point positions will be computed for this sample.
        path : str, optional
            Path to the samples yaml file.

        Returns
        -------
        x, y : tuple
            The x, y position for m n location.
        """
        path = path or self._path
        sample_name = sample_name or self.current_sample

        if sample_name is None or sample_name == '':
            raise ValueError(
                'Please make sure you provide a sample name or use load()')
        # if we have a current loaded sample, use the current M, N values and
        # current coefficients
        if self.current_sample != '':
            m_points, n_points = self.m_n_points
            coeffs = self.coefficients
        else:
            # try to get them from the sample_name file
            entry = os.path.join(path, sample_name + '.yml')
            m_points, n_points, coeffs = self.get_sample_map_info(
                str(sample_name), path=entry)

        if any(v is None for v in [m_points, n_points, coeffs]):
            raise ValueError('Some values are empty, please check the sample '
                             f'{sample_name} in the has the M and N values as '
                             'well as coefficients saved')

        if (m_row > m_points) or (n_column > n_points):
            raise IndexError('Index out of range, make sure the m and n values'
                             f' are between ({m_points, n_points})')
        if (m_row or n_column) == 0:
            raise IndexError('Please start at 1, 1, as the initial points.')

        xx_origin, yy_origin = get_unit_meshgrid(m_rows=m_points,
                                                 n_columns=n_points)

        a_coeffs = coeffs[:4]
        b_coeffs = coeffs[4:]

        if not compute_all:
            logic_x = xx_origin[m_row - 1][n_column - 1]
            logic_y = yy_origin[m_row - 1][n_column - 1]
            x, y = convert_to_physical(a_coeffs, b_coeffs, logic_x, logic_y)
            return x, y
        else:
            # compute all points
            x_points, y_points = [], []
            for rowx, rowy in zip(xx_origin, yy_origin):
                for x, y in zip(rowx, rowy):
                    i, j = convert_to_physical(a_coeffs=a_coeffs,
                                               b_coeffs=b_coeffs,
                                               logic_x=x, logic_y=y)
                    x_points.append(i)
                    y_points.append(j)
            return x_points, y_points

    def move_to_sample(self, m, n):
        """
        Move x,y motors to the computed positions of n, m of current sample.

        Given m (row) and n (column), compute the positions for x and y based
        on the current sample's parameters. See `current_sample` and move
        the x and y motor to those positions.

        Parameters
        ----------
        m : int
            Indicates the row on the grid.
        n : int
            Indicates the column on the grid.
        """
        sample_name = self.current_sample
        if sample_name:
            n, m = self.compute_mapped_point(m_row=m, n_column=n)
        self.x.mv(n)
        self.y.mv(m)

    def move_to(self, sample, m, n):
        """
        Move x,y motors to the computed positions of n, m of given sample.

        Given m (row) and n (column), compute the positions for x and y based
        on the current sample's parameters. See `current_sample`

        Parameters
        ----------
        m : int
            Indicates the row on the grid.
        n : int
            Indicates the column on the grid.
        """
        entry = os.path.join(self._path, sample + '.yml')
        n, m = self.compute_mapped_point(m_row=m, n_column=n,
                                         sample_name=sample, path=entry)
        self.x.mv(n)
        self.y.mv(m)

    def set_status(self, m, n, status=False, sample_name=None, path=None):
        """
        TODO not working properly yet
        Set the status for a specific m and n point.

        Parametrs:
        ---------
        m : int
            Indicates the row number starting at 1.
        n : int
            Indicates the column number starting at 1.
        status : bool, optional
            `False` to indicate that is has been shot, and `True` for
            available.
        """
        assert isinstance(status, bool)
        sample_name = sample_name or self.current_sample
        path = path or os.path.join(self._path, sample_name + '.yml')
        m_points, n_points = self.m_n_points

        if (m > m_points) or (n > n_points):
            raise IndexError('Index out of range, make sure the m and n values'
                             f' are between ({m_points, n_points})')
        if (m or n) == 0:
            raise IndexError('Please start at 1, 1, as the initial points.')

        with open(path) as sample_file:
            yaml_dict = yaml.safe_load(sample_file) or {}
            sample = yaml_dict.get(sample_name)
            if sample:
                xx = sample['xx']
                yy = sample['yy']

                n_pos = next(d['pos'] for (index, d) in enumerate(xx)
                             if index == n - 1)

                m_pos = next(d['pos'] for (index, d) in enumerate(yy)
                             if index == m - 1)

                for xd in sample.get('xx'):
                    for k, v in xd.items():
                        if k == 'pos' and v == n_pos:
                            xd.update((st, status)
                                      for st, vv in xd.items()
                                      if st == 'status')
                for yd in sample.get('yy'):
                    for k, v in yd.items():
                        if k == 'pos' and v == m_pos:
                            yd.update((st, status)
                                      for st, vv in xd.items()
                                      if st == 'status')
                yaml_dict[sample_name].update(sample)
            else:
                raise ValueError('Could not find this sample name in the file:'
                                 f' {sample}')
        with open(path, 'w') as sample_file:
            yaml.safe_dump(yaml_dict, sample_file,
                           sort_keys=False, default_flow_style=False)


def mesh_interpolation(top_left, top_right, bottom_right, bottom_left):
    """
    Mapping functions for an arbitrary quadrilateral.

    Reference: https://www.particleincell.com/2012/quad-interpolation/

    In order to perform the interpolation on an arbitrary quad, we need to
    obtain a mapping function. Our goal is to come up with a function such
    as (x, y) = f(l, m) where l = [0, 1] and m = [0, 1] describes the
    entire point space enclosed by the quadrilateral. In addition, we want
    f(0, 0) = (x1, y1), f(1, 0) = (x2, y2) and so on to correspond to the
    polygon vertices. This function forms a map that allows us to
    transform the quad from the physical coordinates set to a logical
    coordinate space. In the logical coordinates, the polygon morphs into
    a square, regardless of its physical form. Once the logical
    coordinates are obtained, we perform the scatter and find the
    physical x, y values.

    To find the map, we assume a bilinear mapping function given by:
    x = alpha_1 + alpha_2*l + alpha_3*m + alpha_4 * l _ m
    y = beta_1 + beta_2 * l + beta_3 * m + beta_4 * l * m

    Next we use these experessions to solve for the 4 coefficients:
    x1    1  0  0  0   alpha_1
    x2    1  1  0  0   alpha_2
    x3    1  1  1  1   alpha_3
    x4    1  0  1  0   alpha_4

    We do the same for the beta coefficients.

    Parameters
    ----------
    top_left : tuple
        (x, y) coordinates of the top left corner
    top_right : tuple
        (x, y) coordinates of the top right corner
    bottom_right : tuple
        (x, y) coordinates of the bottom right corner
    bottom_left : tuple
        (x, y) coordinates of the bottom left corner

    Returns
    -------
    a_coeffs, b_coeffs : tuple
        List of tuples with the alpha and beta coefficients for projective
        transformation. They are used to find x and y.
    """
    # describes the entire point space enclosed by the quadrilateral
    unit_grid = np.array([[1, 0, 0, 0],
                          [1, 1, 0, 0],
                          [1, 1, 1, 1],
                          [1, 0, 1, 0]])
    # x value coordinates for current grid (4 corners)
    px = np.array([top_left[0],
                   top_right[0],
                   bottom_right[0],
                   bottom_left[0]])
    # y value coordinates for current grid (4 corners)
    py = np.array([top_left[1],
                   top_right[1],
                   bottom_right[1],
                   bottom_left[1]])

    a_coeffs = np.linalg.solve(unit_grid, px)
    b_coeffs = np.linalg.solve(unit_grid, py)

    return a_coeffs, b_coeffs


def get_unit_meshgrid(m_rows, n_columns):
    """
    Based on the 4 coordinates and m and n points, find the meshgrid.

    Regardless of the physical form of our polygon, we first need to morph
    it into a unit square.

    Parameters
    ----------
    m_rows : int
        Number of rows our grid has.
    n_columns : int
        Number of columns our grid has.
    """
    px = [0, 1, 1, 0]
    py = [0, 0, 1, 1]
    x0 = min(px)
    lx = max(px) - min(px)
    y0 = min(py)
    ly = max(py) - min(py)

    ni = n_columns
    nj = m_rows

    dx = lx / (ni - 1)
    dy = ly / (nj - 1)

    xx = [x0 + (i - 1) * dx for i in range(1, ni + 1)]
    yy = [y0 + (j - 1) * dy for j in range(1, nj + 1)]

    return np.meshgrid(xx, yy)


def convert_to_physical(a_coeffs, b_coeffs, logic_x, logic_y):
    """
    Convert to physical coordinates from logical coordinates.

    Parameters
    ----------
    a_coeffs : array
        Perspective transformation coefficients for alpha.
    b_coeffs : array
        Perspective transformation coefficients for beta.
    logic_x : float
        Logical point in the x direction.
    logic_y : float
        Logical point in the y direction.

    Returns
    -------
    x, y : tuple
        The x and y physical values on the specified grid.
    """
    # x = a(1) + a(2)*l + a(3)*m + a(4)*l*m
    x = (a_coeffs[0] + a_coeffs[1] * logic_x + a_coeffs[2]
         * logic_y + a_coeffs[3] * logic_x * logic_y)
    # y = b(1) + b(2)*l + b(3)*m + b(4)*l*m
    y = (b_coeffs[0] + b_coeffs[1] * logic_x +
         b_coeffs[2] * logic_y + b_coeffs[3] * logic_x * logic_y)
    return x, y


def snake_grid_list(points):
    """
    Flatten them into lists with snake_like pattern coordinate points.
    [[1, 2], [3, 4]] => [1, 2, 4, 3]

    Parameters
    ----------
    points : array
        Array containing the grid points for an axis with shape MxN.

    Returns
    -------
    flat_points : list
        List of all the grid points folowing a snake-like pattern.
    """
    temp_points = []
    for i in range(points.shape[0]):
        if i % 2 == 0:
            temp_points.append(points[i])
        else:
            t = points[i]
            tt = t[::-1]
            temp_points.append(tt)
    flat_points = list(chain.from_iterable(temp_points))
    # convert the numpy.float64 to normal float to be able to easily
    # save them in the yaml file
    flat_points = [float(v) for v in flat_points]
    return flat_points
