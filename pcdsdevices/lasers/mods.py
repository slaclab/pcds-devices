"""
Module for the L2SI Laser MODS Table TILEs and Table classes.

The L2SI laser system MODS table is designed to be a modular system for
manipulating a 800nm base laser beam to fit the needs of experiments in the
hutches. This system provides controls for adjusting parameters such as pulse
energy, wavelenth, pulse duration, etc., though not all necessarily at the same
time. The MODS table is built out of "TILEs" (Tabletop Interchange-able Laser
Elements) that provide certain units of functionality, such as harmonics
generation or pulse compression. These TILEs are meant to be swapped in or out
as needed. 

Within a TILE, all elements, whether they are controllable via EPICs or not,
are given PVs. The fungible taxon for laser PVs follow the following naming
convention: 

    <TILE>_<Laser Path>_<Assembly/Component>_<Component>
    
For example, the near-field camera on the first TuttiFrutti on diagnostic path
1 in the ejection TILE would be given the PV:

    EJX_DP1_TF1_NF1

Since all elements, controllable or not, are given PVs, the TILE class
components often skip numbers, e.g. MR1 is followed by MR3. 

Below is a compilation of abbreviations used in the laser MODS system.

TILEs
-----
INJ: Injection
COM: Compressor
HRM: Harmonics generation
EJX: Ejection

Laser Paths
-----------
MP: Main path
DP: Diagnostic path

Assembly/Components
-------------------
ATT:  Attenuator
ZOO:  Zoom telescope
PER:  Periscope
TF:   TuttiFrutti
RBS:  Rotary Beam Switch
SHS:  Shutters
HRO:  Horizontal retro-reflector
VRO:  Vertical retro-reflector
DLY:  Delay Stage
OAP:  Off axis parabola
ANT:  Anamorphic telescope
IMG:  ATM imager
HNF:  High sensitivity NF/FF
LIC:  Laser in-coupling
GT:   Grating translation
AP:   Aperature
S4:   4-position Thorlabs Elliptec slider (Ell9)
CVS:  CVMI Switch
CMC:  CM compressor
SPO:  Second-harmonic-generation Pick Off
PC:   Prism compressor
TAU:  Pulse duration measurement
MR:   Mirror
BS:   Beam splitter
WP:   Waveplate
LS:   Lens positioner
BD:   Beam dump
SL:   Slider
DI:   Diode
SP:   Spectrometer
WF:   Wavefront sensor
PM:   Power meter
WIN:  Window
PIN:  Pinhole
POL:  Polarizer
XTL:  Crystal
EM:   Energy meter
QC:   Quad cell
AS:   ATM sample
OBJ:  Objective
DEV:  Device
PZM:  Prism
NF:   Near-field camera
FF:   Far-field camera
PD:   Pulse duration camera
TIP:  Mirror tip stage
TILT: Mirror tilt stage
ALC:  ATM line camera
AAC:  ATM area camera
LM:   Linear motion
RM:   Rotary motion
"""
from ophyd import Device
from ophyd import Component as Cpt

from pcdsdevices.epics_motor import SmarAct, SmarActTipTilt
from pcdsdevices.areadetector.detectors import LasBasler

from pcdsdevices.lasers.tuttifrutti import TuttiFrutti, TuttiFruttiCls
from pcdsdevices.lasers.zoomtelescope import ZoomTelescope
from pcdsdevices.lasers.ek9000 import EnvironmentalMonitor, El3174AiCh
from pcdsdevices.lasers.elliptec import EllLinear, EllRotation, Ell9


### Generic TILE Devices
class TileBase(Device):
    """
    Base class for MODS TILEs.
    """
    pass


class InjectionTile(TileBase):
    """
    Class for L2SI MODS injection TILE. Common to several installations.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM1K4
        (TMO).

    name : str
        Name for the device, e.g. tmo_mods_inj_tile

    Examples
    --------
    # TMO Injection TILE
    tmo_inj = InjectionTile('LM1K4', 'tmo_inj')
    # ChemRIXS Injection TILE
    crixs_inj = InjectionTile('LM2K2', 'crixs_inj')
    """
    zoom_telescope = Cpt(ZoomTelescope, ':INJ_MP1_ZOO1', kind='normal')
    waveplate1 = Cpt(SmarAct, ':INJ_MP1_ATT1_WP1', kind='normal')
    waveplate2 = Cpt(SmarAct, ':INJ_MP1_ATT1_WP2', kind='normal')
    mp1_mr1 = Cpt(SmarActTipTilt, ':INJ_MP1_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    dp2_mr1 = Cpt(SmarAct, ':INJ_DP2_MR1')
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'inj_dp1_tf1', nf=True, ff=True, ell=True),
                  ':INJ_DP1_TF1', kind='normal')
    dp2_tf1 = Cpt(TuttiFruttiCls('', 'inj_dp2_tf1', spec=True, pm=True, ell=True,
                                 wfs=True),
                  ':INJ_DP2_TF1', kind='normal')
    env_sensors = Cpt(EnvironmentalMonitor, '_BHC_AI_01', kind='normal')


class CompressorTile(TileBase):
    """
    Class for L2SI MODS compressor TILE. Common to several installations.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM1K4
        (TMO).

    name : str
        Name for the device, e.g. tmo_mods_com_tile

    Examples
    --------
    # TMO Compressor TILE
    tmo_com = CompressorTile('LM1K4', 'tmo_com')
    # ChemRIXS Compressor TILE
    crixs_com = CompressorTile('LM2K2', 'crixs_com')
    """
    mp1_mr1 = Cpt(SmarActTipTilt, ':COM_MP1_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_mr4 = Cpt(SmarActTipTilt, ':COM_MP1_MR4', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_gt2_lm1 = Cpt(SmarAct, ':COM_MP1_GT2_LM1', kind='normal')
    mp1_mr8 = Cpt(SmarActTipTilt, ':COM_MP1_MR8', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp2_dly1 = Cpt(SmarAct, ':COM_MP2_DLY1', kind='normal')
    mp2_bs2 = Cpt(SmarActTipTilt, ':COM_MP2_BS2', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    dp1_mr1 = Cpt(SmarAct, ':COM_DP1_MR1', kind='normal')
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'com_dp1_tf1', nf=True, ff=True,
                  ell=True),
                  ':COM_DP1_TF1', kind='normal')
    dp2_xtl1 = Cpt(EllLinear, ':COM_DP2_XTL1:ELL', port=0, channel=1,
                   kind='normal')
    dp2_tf1 = Cpt(TuttiFruttiCls('', 'com_dp2_tf1', nf=True, ff=True, ell=True,
                                 spec=True, pm=True),
                                 ':COM_DP2_TF1', kind='normal')
    dp3_tf1 = Cpt(TuttiFruttiCls('', 'com_dp3_tf1', nf=True, ff=True, ell=True,
                                 pm=True),
                                 ':COM_DP3_TF1', kind='normal')
    env_sensors = Cpt(EnvironmentalMonitor, '_BHC_AI_01', kind='normal')


class HarmonicsTile(TileBase):
    """
    Class for L2SI MODS harmonics TILE. Common to several installations.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM1K4
        (TMO).

    name : str
        Name for the device, e.g. tmo_mods_hrm_tile

    Examples
    --------
    # TMO Harmonics TILE
    tmo_hrm = HarmonicsTile('LM1K4', 'tmo_hrm')
    # ChemRIXS Compressor TILE
    crixs_hrm = HarmonicsTile('LM2K2', 'crixs_hrm')
    """
    mp1_mr1 = Cpt(SmarActTipTilt, ':HRM_MP1_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_mr1_lm1 = Cpt(SmarAct, ':HRM_MP1_MR1_LM1', kind='normal')
    mp1_mr4 = Cpt(SmarActTipTilt, ':HRM_MP4_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_spo1 = Cpt(SmarAct, ':HRM_MP1_SPO1', kind='normal')
    mp1_mr7 = Cpt(SmarActTipTilt, ':HRM_MP7_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_pc1 = Cpt(SmarAct, ':HRM_MP1_PC1', kind='normal')
    mp1_pc1_pzm2 = Cpt(SmarAct, ':HRM_MP1_PC1', kind='normal')
    mp1_mr8 = Cpt(SmarActTipTilt, ':HRM_MP8_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_mr8_lm1 = Cpt(SmarAct, ':HRM_MP1_MR8_LM1', kind='normal')
    mp3_mr1 = Cpt(SmarActTipTilt, ':HRM_MP3_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp3_mr1_lm1 = Cpt(SmarAct, ':HRM_MP3_MR1_LM1', kind='normal')
    mp3_mr2 = Cpt(SmarActTipTilt, ':HRM_MP2_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp3_mr2_lm1 = Cpt(SmarAct, ':HRM_MP3_MR2_LM1', kind='normal')
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'hrm_dp1_tf1', nf=True, ff=True,
                  ell=True, pm=True),
                  ':HRM_DP1_TF1', kind='normal')
    dp1_tf2 = Cpt(TuttiFruttiCls('', 'hrm_dp2_tf1', nf=True, ff=True,
                  ell=True, pm=True, spec=True),
                  ':HRM_DP1_TF1', kind='normal')


### Hutch-specific TILE Devices
class XppInjectionTile(TileBase):
    """
    Class for the XPP injection TILE.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LMXPP.

    name : str
        Name for the device, e.g. xpp_inj_tile

    Examples
    --------
    # XPP Injection TILE
    xpp_inj = XppInjectionTile('LMXPP', 'xpp_inj')
    """
    mp1_mr1 = Cpt(SmarActTipTilt, ':INJ_MP1_MR1', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    zoom_telescope = Cpt(ZoomTelescope, ':INJ_MP1_ZOO1', kind='normal')
    mp1_mr4 = Cpt(SmarActTipTilt, ':INJ_MP1_MR4', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_mr5 = Cpt(SmarActTipTilt, ':INJ_MP1_MR5', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'inj_dp1_tf1', nf=True, ff=True, ell=True),
                  ':INJ_DP1_TF1', kind='normal')
    dp2_tf1 = Cpt(TuttiFruttiCls('', 'inj_dp2_tf1', spec=True, pm=True, ell=True,
                                 wfs=True),
                  ':INJ_DP2_TF1', kind='normal')
    env_sensors = Cpt(EnvironmentalMonitor, '_BHC_AI_01', kind='normal')


class TmoIp1EjectionTile(TileBase):
    """
    Class for the TMO Ip1 Ejection TILE.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM1K4.

    name : str
        Name for the device, e.g. tmo_ejx_tile

    Examples
    --------
    # TMO IP1 Ejection TILE
    tmo_ejx = TmoIp1EjectionTile('LM1K4', 'tmo_ejx')
    """
    mp1_mr1 = Cpt(SmarActTipTilt, ':EJX_MP1_MR1', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_mr2 = Cpt(SmarActTipTilt, ':EJX_MP1_MR2', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_s41 = Cpt(Ell9, ':EJX_MP1_S41', port=0, channel=1, kind='normal')
    mp1_s42 = Cpt(Ell9, ':EJX_MP2_S41', port=0, channel=2, kind='normal')
    mp1_mr3 = Cpt(SmarAct, ':EJX_MP1_MR3', kind='normal')
    mp1_oap1 = Cpt(SmarAct, ':EJX_MP1_OAP1', kind='normal')
    mp1_oap1_mr1 = Cpt(SmarActTipTilt, ':EJX_OAP1', tip_pv='_TIP1',
                       tilt_pv='_TILT1', kind='normal')
    mp1_oap1_mr3 = Cpt(SmarActTipTilt, ':EJX_OAP1', tip_pv='_TIP3',
                       tilt_pv='_TILT3', kind='normal')
    dp11_ff1 = Cpt(LasBasler, ':EJX_DP11_FF1', kind='normal')
    #TODO: Add dp12_tf1 diode 
    dp12_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp12_tf1', ff=True, ell=True, 
                   pm=True, wfs=True),
                   ':EJX_DP12_TF1', kind='normal')
    #TODO: Add pulse duration diagnostic (dp2_tau1)
    #dp2_tau1_lm1 = Cpt(SmarAct, ':EJX_DP2_LM1', kind='normal')
    #dp2_tau1_pd1 = Cpt(LasBasler, ':EJX_DP2_TAU1_PD1', kind='normal')
    #TODO: Add dp31 energy meter
    #TODO: Add dp32 diode
    dp32_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp32_tf1', nf=True, ff=True,                            ell=True, spec=True),
                   ':EJX_DP32_TF1', kind='normal')
    mp2_mr3 = Cpt(SmarAct, ':EJX_MP2_MR3', kind='normal')
    mp2_att1_pol1 = Cpt(EllRotation, ':EJX_MP2_ATT1_POL1:ELL', port=0,
                        channel=1, kind='normal')
    mp2_att1_pol2 = Cpt(EllRotation, ':EJX_MP2_ATT1_POL2:ELL', port=0,
                        channel=2, kind='normal')
    mp2_mr5 = Cpt(SmarAct, ':EJX_MP2_MR5', kind='normal')
    mp2_ls3_lm1 = Cpt(SmarAct, ':EJX_MP2_LS3_LM1', kind='normal')
    mp2_ls3_lm2 = Cpt(SmarAct, ':EJX_MP2_LS3_LM2', kind='normal')
    mp2_ls3_lm3 = Cpt(SmarAct, ':EJX_MP2_LS3_LM3', kind='normal')
    dp4_ap1 = Cpt(SmarActOpenLoop, ':EJX_DP4_AP1', kind='normal')
    dp4_ap2 = Cpt(SmarActOpenLoop, ':EJX_DP4_AP2', kind='normal')
    dp4_pm1 = Cpt(El3174AiCh, ':EJX_DP4_PM1', kind='normal')
    dp5_bs1 = Cpt(SmarAct, ':EJX_DP5_BS1', kind='normal')
    #TODO: Add dp5 QC1, QC2, diode
    dp5_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp5_tf1', ell=True, pm=True),
                   ':EJX_DP5_TF1', kind='normal')
    dp6_cvs1_mr1 = Cpt(SmarAct, ':EJX_DP6_CVS1_MR1', kind='normal')
    dp6_cvs1_mr2 = Cpt(SmarAct, ':EJX_DP6_CVS1_MR2', kind='normal')
    #TODO: Add dp6 energy meter, diodes
    dp6_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp6_tf1', ell=True, nf=True, ff=True,
                  spec=True),
                  ':EJX_DP6_TF1', kind='normal')


class TmoAtmTile(TileBase):
    """
    Class for L2SI TMO MODS ATM TILE. Unique to TMO.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM1K4
        (TMO).

    name : str
        Name for the device, e.g. tmo_mods_atm_tile

    Examples
    --------
    # TMO ATM TILE
    tmo_atm = TmoAtmTile('LM1K4', 'tmo_atm')
    """
    mp1_mr1 = Cpt(SmarActTipTilt, ':ATM_MP1_MR1', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_mr3 = Cpt(SmarActTipTilt, ':ATM_MP1_MR3', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_wp1_rm1 = Cpt(EllRotation, ':ATM_MP1_WP1_RM1:ELL', port=0, channel=3,
                      kind='normal')
    mp1_wp1_lm1 = Cpt(EllLinear, ':ATM_MP1_WP1_LM1:ELL', port=0, channel=2,
                      kind='normal')
    mp2_pol1 = Cpt(EllRotation, ':ATM_MP2_POL1:ELL', port=0, channel=1,
                   kind='normal')
    # TODO: Add diode once it's ready
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'atm_dp1_tf1', nf=True, ff=True,
                  ell=True),
                  ':ATM_DP1_TF1', kind='normal')
    # TODO: Add diode once it's ready
    dp2_tf1 = Cpt(TuttiFruttiCls('', 'atm_dp2_tf1', nf=True, ff=True,
                  ell=True, pm=True, spec=True),
                  ':ATM_DP2_TF1', kind='normal')


class ChemRixsEjectionTile(TileBase):
    """
    Class for the ChemRIXS Ejection TILE.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM2K2.

    name : str
        Name for the device, e.g. crixs_ejx_tile

    Examples
    --------
    # ChemRIXS Ejection TILE
    crixs_ejx = ChemRixsEjectionTile('LM2K2', 'crixs_ejx')
    """
    mp1_s41 = Cpt(Ell9, ':EJX_MP1_S41', port=0, channel=1, kind='normal')
    mp1_mr3 = Cpt(SmarActTipTilt, ':EJX_MP1_MR3', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_mr6 = Cpt(SmarActTipTilt, ':EJX_MP1_MR6', tip_pv='_TIP1',
                  tilt_pv='_TILT1', kind='normal')
    mp1_ls3_lm1 = Cpt(SmarAct, ':EJX_MP2_LS3_LM1', kind='normal')
    mp1_ls3_lm2 = Cpt(SmarAct, ':EJX_MP2_LS3_LM2', kind='normal')
    mp1_ls3_lm3 = Cpt(SmarAct, ':EJX_MP2_LS3_LM3', kind='normal')
    mp21_mr1 = Cpt(SmarAct, ':EJX_MP21_MR1', kind='normal')
    mp21_mr3 = Cpt(SmarAct, ':EJX_MP21_MR3', kind='normal')
    dp1_mr1 = Cpt(SmarAct, ':EJX_DP1_MR1', kind='normal')
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp1_tf1', ell=True, pm=True,
                  wfs=True),
                  ':EJX_DP1_TF1', kind='normal')
    dp2_mr1 = Cpt(SmarAct, ':EJX_DP2_MR1', kind='normal')
    #TODO: Add pulse duration diagnostic (dp2_tau1)
    #dp2_tau1_lm1 = Cpt(SmarAct, ':EJX_DP2_LM1', kind='normal')
    #dp2_tau1_pd1 = Cpt(LasBasler, ':EJX_DP2_TAU1_PD1', kind='normal')
    #TODO: Add dp3_tf1 diode
    dp3_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp3_tf1', ell=True, nf=True, ff=True,
                  spec=True, pm=True),
                  ':EJX_DP3_TF1', kind='normal')
    #TODO: Add dp4_tf1 energy meter
    dp4_tf1 = Cpt(TuttiFruttiCls('', 'ejx_dp4_tf1', ell=True, nf=True, ff=True,
                  spec=True, pm=True),
                  ':EJX_DP4_TF1', kind='normal')


class ChemRixsAtmTile(TileBase):
    """
    Class for L2SI ChemRIXS MODS ATM TILE. Unique to ChemRIXS.

    Parameters
    ----------
    prefix : str
        Five character prefix for particular MODS installation, e.g. LM2K2
        (ChemRIXS).

    name : str
        Name for the device, e.g. crixs_mods_atm_tile

    Examples
    --------
    # ChemRIXS ATM TILE
    crixs_atm = ChemRixsAtmTile('LM1K4', 'crixs_atm')
    """
    mp1_mr3 = Cpt(SmarActTipTilt, ':ATM_MP1_MR3', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_mr6 = Cpt(SmarActTipTilt, ':ATM_MP1_MR6', tip_pv='_TIP1',
                 tilt_pv='_TILT1', kind='normal')
    mp1_wp1_rm1 = Cpt(EllRotation, ':ATM_MP1_WP1_RM1:ELL', port=0, channel=3,
                      kind='normal')
    mp1_wp1_lm1 = Cpt(EllLinear, ':ATM_MP1_WP1_LM1:ELL', port=0, channel=2,
                      kind='normal')
    mp2_pol1 = Cpt(EllRotation, ':ATM_MP2_POL1:ELL', port=0, channel=1,
                   kind='normal')
    # TODO: Add diode once it's ready
    dp1_tf1 = Cpt(TuttiFruttiCls('', 'atm_dp1_tf1', nf=True, ff=True,
                  ell=True, pm=True, spec=True),
                  ':ATM_DP1_TF1', kind='normal')


### Generic MODS Table Devices
class ModsBase(Device):
    """
    Base class for MODS Tables.
    """
    pass


### Hutch-specific MODS Table Devices
class TmoIp1ModsTable(ModsBase):
    """
    Class for L2SI IP1 MODS table. Unique to IP1.

    Parameters
    ----------
    name : str
        Name for the table, e.g. ip1_mods_table

    Examples
    --------
    # IP1 MODS Table 
    ip1_mods = TmoIp1ModsTable('ip1_mods_table')
    """
    Cpt(InjectionTile, 'LM1K4', kind='normal')
    Cpt(CompressorTile, 'LM1K4', kind='normal')
    Cpt(HarmonicsTile, 'LM1K4', kind='normal')
    Cpt(TmoIp1EjectionTile, 'LM1K4', kind='normal')

    def __init__(self, *, name,  **kwargs):
        super().__init__('', name=name, **kwargs)


class ChemRixsModsTable(ModsBase):
    """
    Class for L2SI ChemRIXS MODS table. Unique to ChemRIXS.

    Parameters
    ----------
    name : str
        Name for the table, e.g. crixs_mods_table

    Examples
    --------
    # ChemRIXS MODS Table 
    crixs_mods = ChemRixsModsTable('crixs_mods_table')
    """
    Cpt(InjectionTile, 'LM2K2', kind='normal')
    Cpt(CompressorTile, 'LM2K2', kind='normal')
    Cpt(HarmonicsTile, 'LM2K2', kind='normal')
    Cpt(ChemRixsEjectionTile, 'LM2K2', kind='normal')

    def __init__(self, *, name,  **kwargs):
        super().__init__('', name=name, **kwargs)
