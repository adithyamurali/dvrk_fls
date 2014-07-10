# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time

import smach

SLEEP_TIME = 0


class StateTestClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """

    def __init__(self):
        self.counter = -1
        self.outcomes = None

    def execute(self, userdata):
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]


class Start(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'])

class MoveToRetractionStagingArea(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class HomePosition(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class MoveToGraspPoint(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'])

class GraspBlock(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['checkGrasp'])

class CheckGrasp(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class ReleaseGrippersNoBlock(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class ReturnToRetractionStagingAreaWithBlock(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class MoveToDropOffStagingArea(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class MoveToDropOffPoint(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class ReleaseGripper(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])

class CheckDropOff(StateTestClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])