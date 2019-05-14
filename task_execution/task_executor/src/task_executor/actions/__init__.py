#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .approach_schunk import ApproachSchunkAction
from .arm import ArmAction
from .arm_cartesian import ArmCartesianAction
from .beep import BeepAction
# from .bin_pick import BinPickAction
from .detach_objects import DetachObjectsAction
from .detect_bins import DetectBinsAction
from .detect_schunk import DetectSchunkAction
from .find_grasps import FindGraspsAction
from .get_beliefs import GetBeliefsAction
from .gripper import GripperAction
from .in_hand_localize import InHandLocalizeAction
from .joystick_trigger import JoystickTriggerAction
from .load_static_octomap import LoadStaticOctomapAction
from .look import LookAction
from .look_at_gripper import LookAtGripperAction
from .look_pan_tilt import LookPanTiltAction
from .move import MoveAction
from .move_backward import MoveBackwardAction
from .move_planar import MovePlanarAction
from .pick import PickAction
from .pick_kit import PickKitAction
from .pick_kit_base import PickKitBaseAction
# from .place import PlaceAction
from .place_kit_base import PlaceKitBaseAction
from .playback_trajectory import PlaybackTrajectoryAction
from .recognize_object import RecognizeObjectAction
from .reposition import RepositionAction
from .reset_object_frame import ResetObjectFrameAction
from .retrieve_grasps import RetrieveGraspsAction
from .segment import SegmentAction
from .schunk import SchunkAction
from .schunk_insertion import SchunkInsertionAction
from .speak import SpeakAction
from .store_object import StoreObjectAction
from .toggle_breakers import ToggleBreakersAction
from .torso import TorsoAction
from .torso_linear import TorsoLinearAction
from .update_beliefs import UpdateBeliefsAction
from .verify_grasp import VerifyGraspAction
from .wait import WaitAction

class Actions(object):
    """
    A registry of actions. It is recommended that you create this object with
    :func:`get_default_actions`. In order to use the actions, they must be
    intialized, which includes connecting to their action servers, services,
    etc. You can use the :attr:`initialized` attribute of this object to know
    if the actions are initialized or not.
    """

    def __init__(self, registry):
        """
        Args:
            registry (dict) : This is a dict of name -> Action class mappings
        """
        self.registry = { key: klass() for key, klass in registry.iteritems() }

        # Flag for if the actions are initialized
        self.initialized = False

        # Quick sanity check because I don't trust people. Also set the action
        # as an attribute for '.' based referencing
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractStep)
            setattr(self, key, action)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self):
        for key, action in self.registry.iteritems():
            action.init(key)

        # Mark the actions as initialized
        self.initialized = True


# The default actions contain all the action interfaces that are known to this
# package
default_actions_dict = {
    'approach_schunk': ApproachSchunkAction,
    'arm': ArmAction,
    'arm_cartesian': ArmCartesianAction,
    'beep': BeepAction,
    # 'bin_pick': BinPickAction,
    'detach_objects': DetachObjectsAction,
    'detect_bins': DetectBinsAction,
    'detect_schunk': DetectSchunkAction,
    'find_grasps': FindGraspsAction,
    'get_beliefs': GetBeliefsAction,
    'gripper': GripperAction,
    'in_hand_localize': InHandLocalizeAction,
    'joystick_trigger': JoystickTriggerAction,
    'load_static_octomap': LoadStaticOctomapAction,
    'look': LookAction,
    'look_at_gripper': LookAtGripperAction,
    'look_pan_tilt': LookPanTiltAction,
    'move': MoveAction,
    'move_backward': MoveBackwardAction,
    'move_planar': MovePlanarAction,
    'pick': PickAction,
    'pick_kit': PickKitAction,
    'pick_kit_base': PickKitBaseAction,
    # 'place': PlaceAction,
    'place_kit_base': PlaceKitBaseAction,
    'playback_trajectory' : PlaybackTrajectoryAction,
    'recognize_object': RecognizeObjectAction,
    'reposition': RepositionAction,
    'reset_object_frame': ResetObjectFrameAction,
    'retrieve_grasps': RetrieveGraspsAction,
    'segment': SegmentAction,
    'schunk': SchunkAction,
    'schunk_insertion': SchunkInsertionAction,
    'speak': SpeakAction,
    'store_object': StoreObjectAction,
    'toggle_breakers': ToggleBreakersAction,
    'torso': TorsoAction,
    'torso_linear': TorsoLinearAction,
    'update_beliefs': UpdateBeliefsAction,
    'verify_grasp': VerifyGraspAction,
    'wait': WaitAction,
}

def get_default_actions():
    """
    Provides a consistent manner of fetching all the actions that are available
    on the robot. This is the preferred manner of getting the actions:

    >>> actions = get_default_actions()
    >>> move_params = { 'location': 'waypoints.origin' }
    >>> actions.move(**move_params)

    Returns:
        (Actions) : The default registry of all the actions that are available
    """
    return Actions(default_actions_dict)
