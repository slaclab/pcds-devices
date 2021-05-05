import importlib
import inspect
import logging
import os
import pkgutil
import shutil
import sys
import warnings
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Dict

import ophyd
import pytest
from epics import PV
from ophyd.signal import LimitError
from ophyd.sim import FakeEpicsSignal, make_fake_device

import pcdsdevices
import pcdsdevices.analog_signals
import pcdsdevices.lens
import pcdsdevices.lxe
from pcdsdevices.attenuator import MAX_FILTERS, Attenuator, _att_classes
from pcdsdevices.device import UnrelatedComponent
from pcdsdevices.mv_interface import setup_preset_paths

MODULE_PATH = Path(__file__).parent


# Signal.put warning is a testing artifact.
# FakeEpicsSignal needs an update, but I don't have time today
# Needs to not pass tons of kwargs up to Signal.put
warnings.filterwarnings('ignore',
                        message='Signal.put no longer takes keyword arguments')

logger = logging.getLogger(__name__)


# Other temporary patches to FakeEpicsSignal
def check_value(self, value):
    if value is None:
        raise ValueError('Cannot write None to epics PVs')
    if not self._use_limits:
        return

    low_limit, high_limit = self.limits
    if low_limit >= high_limit:
        return

    if not (low_limit <= value <= high_limit):
        raise LimitError('Value {} outside of range: [{}, {}]'
                         .format(value, low_limit, high_limit))


# Check value is busted, ignores (0, 0) no limits case
FakeEpicsSignal.check_value = check_value
# Metadata changed is missing
FakeEpicsSignal._metadata_changed = lambda *args, **kwargs: None
# pvname is missing
FakeEpicsSignal.pvname = ''
# lots of things are missing
FakeEpicsSignal._read_pv = SimpleNamespace(get_ctrlvars=lambda: None)
# Stupid patch that somehow makes the test cleanup bug go away
PV.count = property(lambda self: 1)

for name, cls in _att_classes.items():
    _att_classes[name] = make_fake_device(cls)


# Used in multiple test files
@pytest.fixture(scope='function')
def fake_att():
    att = Attenuator('TST:ATT', MAX_FILTERS-1, name='test_att')
    att.readback.sim_put(1)
    att.done.sim_put(0)
    att.calcpend.sim_put(0)
    for i, filt in enumerate(att.filters):
        filt.state.put('OUT')
        filt.thickness.put(2*i)
    return att


@pytest.fixture(scope='function')
def presets():
    folder_obj = Path(__file__).parent / 'test_presets'
    folder = str(folder_obj)
    if folder_obj.exists():
        shutil.rmtree(folder)
    hutch = folder + '/hutch'
    user = folder + '/user'
    os.makedirs(hutch)
    os.makedirs(user)
    setup_preset_paths(hutch=hutch, user=user)
    yield
    setup_preset_paths()
    shutil.rmtree(folder)


def find_pcdsdevices_submodules() -> Dict[str, ModuleType]:
    """Find all pcdsdevices submodules, as a dictionary of name to module."""
    modules = {}
    package_root = str(MODULE_PATH.parent / 'pcdsdevices')
    for item in pkgutil.walk_packages(path=[package_root],
                                      prefix='pcdsdevices.'):
        try:
            modules[item.name] = sys.modules[item.name]
        except KeyError:
            # Submodules may not yet be imported; do that here.
            try:
                modules[item.name] = importlib.import_module(
                    item.name, package='pcdsdevices'
                )
            except Exception:
                logger.exception('Failed to import %s', item.name)

    return modules


def find_all_device_classes() -> list:
    """Find all device classes in pcdsdevices and return them as a list."""
    def should_include(obj):
        return (
            inspect.isclass(obj) and
            issubclass(obj, ophyd.Device) and
            not obj.__module__.startswith('ophyd')
        )

    def sort_key(cls):
        return (cls.__module__, cls.__name__)

    devices = [
        obj
        for module in find_pcdsdevices_submodules().values()
        for _, obj in inspect.getmembers(module, predicate=should_include)
    ]

    return list(sorted(set(devices), key=sort_key))


def find_all_callables() -> list:
    """Find all callables in pcdsdevices and return them as a list."""
    def should_include(obj):
        try:
            name = obj.__name__
            module = obj.__module__
        except AttributeError:
            return False

        return (
            callable(obj) and
            not inspect.isclass(obj) and
            module.startswith('pcdsdevices') and
            not module.startswith('pcdsdevices._version')
            and not name.startswith("_")
        )

    def sort_key(obj):
        return (obj.__module__, obj.__name__)

    devices = [
        obj
        for module in find_pcdsdevices_submodules().values()
        for _, obj in inspect.getmembers(module, predicate=should_include)
    ]

    return list(sorted(set(devices), key=sort_key))


# If your device class has some essential keyword arguments necesary to be
# instantiated that cannot be automatically determined from its signature,
# add them here.
class_to_essential_kwargs = {
    pcdsdevices.analog_signals.Mesh: dict(sp_ch=0, rb_ch=0),
    pcdsdevices.lens.LensStack: dict(
        path=str(MODULE_PATH / 'test_lens_sets' / 'test'),
    ),
    pcdsdevices.lens.SimLensStack: dict(
        path=str(MODULE_PATH / 'test_lens_sets' / 'test'),
    ),
    pcdsdevices.lxe.LaserEnergyPositioner: dict(
        calibration_file=MODULE_PATH / 'xcslt8717_wpcalib_opa',
    ),
}


def best_effort_instantiation(device_cls, *, skip_on_failure=True):
    """
    Best effort create a fake device instance from "real" device_cls.

    Optionally skips the test automatically on failure.

    Parameters
    ----------
    device_cls : type
        Device class, a subclass of ophyd.Device

    skip_on_failure : bool, optional
        If set, skip the test with a reasonable message.
    """
    fake_cls = make_fake_device(device_cls)

    kwargs = {
        'name': device_cls.__name__,
    }

    # Add in unrelated components as strings - we know about these ahead
    # of time.
    for cpt_walk in fake_cls.walk_components():
        if isinstance(cpt_walk.item, UnrelatedComponent):
            kwarg = cpt_walk.dotted_name.replace('.', '_') + '_prefix'
            kwargs[kwarg] = f'{kwarg}:'  # this is arbitrary

    # Otherwise, try to look at the signature and give us *something* for
    # the required ones without defaults.
    sig = inspect.signature(fake_cls)
    for param in sig.parameters.values():
        if param.default is inspect.Signature.empty:
            if param.kind not in {param.VAR_KEYWORD, param.VAR_POSITIONAL}:
                # This is best effort, after all!
                kwargs.setdefault(
                    param.name,
                    'test:abcd' if 'prefix' in param.name else 'test'
                )

    # Add in essential kwargs, if available:
    kwargs.update(class_to_essential_kwargs.get(device_cls, {}).items())

    try:
        return fake_cls(**kwargs)
    except Exception as ex:
        if skip_on_failure:
            pytest.skip(
                f'Unable to instantiate {device_cls}: {ex} (kwargs={kwargs})'
            )
        raise
