#!/usr/bin/python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros

from simple_script_server import simple_script_server

import states.interaction
import states.movement

# main
def main():
    sss = simple_script_server()
    rospy.init_node('test_reg')

    # Create a SMACH state machine
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'scan':[]}
    sm.userdata.nonblocking = False

    # Open the container
    with sm:
        
        smach.StateMachine.add('ReadDataForRegistration', states.movement.ReadDataForRegistration('scan'), 
                               transitions={'pass':'success'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome
    
    print sm.userdata.data['scan']


if __name__ == '__main__':
    main()
