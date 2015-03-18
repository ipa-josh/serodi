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
                               transitions={'pass':'MoveRel'})
        
        smach.StateMachine.add('MoveRel', states.movement.MoveRel(sss, [0.1,0.1,0.1]), 
                               transitions={'success':'MoveRel_Registration', 'failed':'failure'})
                               
        smach.StateMachine.add('MoveRel_Registration', states.movement.MoveRel_Registration(sss,sm.userdata.data['scan']), 
                               transitions={'success':'success', 'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main()
