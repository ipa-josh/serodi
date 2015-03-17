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
    rospy.init_node('serodi_smach_op_patrol')

    # Create a SMACH state machine
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_patrol':{}}
    sm.userdata.nonblocking = False

    # Open the container
    with sm:        
        # just move on path (poses)
        
        smach.StateMachine.add('LoadYaml', states.interaction.LoadYaml('op_patrol', '../config/op_patrol.yaml', sm), 
                               transitions={'success':'Patrol1',  'failed':'failure'})
			
        smach.StateMachine.add('Patrol1',  states.movement.MoveOnPath(sss,sm.userdata.data['op_patrol']['path']),
			transitions={'success':'Patrol2',  'failed':'failure'})
			
        smach.StateMachine.add('Patrol2',  states.movement.MoveOnPath(sss,sm.userdata.data['op_patrol']['path'], True),
			transitions={'success':'MoveToHome',  'failed':'failure'})
			
        smach.StateMachine.add('MoveToHome',  states.movement.MoveToPose(sss,'home'),
			transitions={'success':'success',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main()
