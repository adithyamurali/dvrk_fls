#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time
import tfx
import smach
import rospy

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

SLEEP_TIME = 0

class StateTestClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """

    def __init__(self):
        self.counter = -1
        self.outcomes = None

        # self.homePose = tfx.pose([0.03362918371560202, 0.01301848139359759, -0.11116944355568323],(-0.7715710017016915, 0.5959403546597739, -0.039617083586624295, 0.2190063234455157))
        # self.retractionStagingPose = tfx.pose([0.046502262400628255, 0.021261836800885284, -0.12519359834952945],(-0.6808902902461443, 0.7257476567154133, 0.0961062550757443, 0.02103186049601479))
        # self.graspPoint = tfx.pose([0.046502262400628255, 0.021261836800885284, -0.13019359834952945],(-0.6808902902461443, 0.7257476567154133, 0.0961062550757443, 0.02103186049601479))
        # self.dropOffStagingPose = tfx.pose([0.046502262400628255, 0.041261836800885284, -0.12619359834952945],(-0.6808902902461443, 0.7257476567154133, 0.0961062550757443, 0.02103186049601479))
        # self.dropOffPose = tfx.pose([0.046502262400628255, 0.041261836800885284, -0.13119359834952945],(-0.6808902902461443, 0.7257476567154133, 0.0961062550757443, 0.02103186049601479))

        self.homePose = tfx.pose([0.04062973244319928, 0.0058030822657793475, -0.07932521787553881],(-0.837099978972779, 0.4780810746632525, -0.12685651335638867, 0.2336868337576227))
        self.retractionStagingPose = tfx.pose([0.06529437744320556, -0.0021532497103947915, -0.09578533223662396],(-0.7422489178426571, 0.6532677419803014, -0.12280820926518593, 0.08500555856202298))
        self.graspPoint = tfx.pose([0.06529437744320556, -0.0021532497103947915, -0.10078533223662396],(-0.7422489178426571, 0.6532677419803014, -0.12280820926518593, 0.08500555856202298))
        self.dropOffStagingPose = tfx.pose([0.06529437744320556, 0.01784675, -0.09578533223662396],(-0.7422489178426571, 0.6532677419803014, -0.12280820926518593, 0.08500555856202298))
        self.dropOffPose = tfx.pose([0.06529437744320556, 0.01784675, -0.10078533223662396],(-0.7422489178426571, 0.6532677419803014, -0.12280820926518593, 0.08500555856202298))

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

class Start(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Start"
        self.davinciArm.executeInterpolatedTrajectory(self.homePose)
        return 'success'

class MoveToRetractionStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm        
    
    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"
        self.davinciArm.setGripperPositionDaVinci(1.000)
        self.davinciArm.executeInterpolatedTrajectory(self.retractionStagingPose)
        return 'success'

class HomePosition(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: HomePosition"
        return 'success'

class MoveToGraspPoint(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: MoveToGraspPoint"

        # self.davinciArm.setGripperPositionDaVinci(0.756851)
        self.davinciArm.executeInterpolatedTrajectory(self.graspPoint)
        return 'success'

class GraspBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: GraspBlock"

        self.davinciArm.setGripperPositionDaVinci(0.30)
        # startPose = self.davinciArm.ravenController.currentPose
        # delta = tfx.pose([0.0, 0.0, -0.002])
        # endPose = raven_util.endPose(startPose, delta)
        self.davinciArm.executeInterpolatedTrajectory(self.graspPoint)
        return "success"        

class CheckGrasp(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: CheckGrasp"
        return 'success'

class ReleaseGrippersNoBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: ReleaseGrippersNoBlock"
        return 'success'

class ReturnToRetractionStagingAreaWithBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReturnToRetractionStagingAreaWithBlock"
        self.davinciArm.executeInterpolatedTrajectory(self.retractionStagingPose)
        return 'success'

class MoveToDropOffStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"
        self.davinciArm.executeInterpolatedTrajectory(self.dropOffStagingPose)
        return 'success'

class MoveToDropOffPoint(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffPoint"
        self.davinciArm.executeInterpolatedTrajectory(self.dropOffPose)
        return 'success'

class ReleaseGripper(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReleaseGripper"
        self.davinciArm.setGripperPositionDaVinci(0.80)
        # startPose = self.davinciArm.ravenController.currentPose
        # delta = tfx.pose([0.0, 0.0, 0.005])
        # endPose = raven_util.endPose(startPose, delta)
        self.davinciArm.executeInterpolatedTrajectory(self.dropOffPose)
        return 'success'

class CheckDropOff(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: CheckDropOff"
        self.davinciArm.executeInterpolatedTrajectory(self.homePose)
        self.davinciArm.stop()        
        return 'success'

class Abort(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Abort"
        return 'success'