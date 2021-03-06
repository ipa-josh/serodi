#!/usr/bin/python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros
import sys

import states.interaction
import states.initialization

# main
def main(en_patrol, en_op):
    rospy.init_node('serodi_smach_prog_scenario')

    # Create a SMACH state machine
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    # Open the container
    with sm:
		sq = smach.Sequence(
				outcomes = ['success','failed'],
				connector_outcome = 'success')
		sq.userdata = sm.userdata
		with sq:
			last_pose=[[]]
			
			if en_patrol:
				smach.Sequence.add('Patrol1', states.initialization.System('rosrun serodi_smach op_patrol.py'))
			elif en_op:
				smach.Sequence.add('Op', states.initialization.System('rosrun serodi_smach op_small.py'))
			else:
				smach.Sequence.add('Patrol1', states.initialization.System('rosrun serodi_smach op_patrol.py'))
				smach.Sequence.add('Wait1', states.interaction.Wait(20))
				smach.Sequence.add('Patrol2', states.initialization.System('rosrun serodi_smach op_patrol.py'))
				smach.Sequence.add('Wait2', states.interaction.Wait(20))
				smach.Sequence.add('Op', states.initialization.System('rosrun serodi_smach op_small.py'))
				smach.Sequence.add('Wait3', states.interaction.Wait(20))
				smach.Sequence.add('Patrol3', states.initialization.System('rosrun serodi_smach op_patrol.py'))
				smach.Sequence.add('Wait4', states.interaction.Wait(20))
				smach.Sequence.add('Patrol4', states.initialization.System('rosrun serodi_smach op_patrol.py'))
				smach.Sequence.add('Wait5', states.interaction.Wait(20))
			
		smach.StateMachine.add('Main', sq, 
						   transitions={'success':'Main',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main("--patrol" in sys.argv[1:], "--op" in sys.argv[1:])
