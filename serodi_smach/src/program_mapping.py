#!/usr/bin/python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros
import sys

from simple_script_server import simple_script_server

import states.interaction
import states.movement
import states.initialization

# main
def main(do_setup):
    sss = simple_script_server()
    rospy.init_node('serodi_smach_prog_mapping')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_small':{}, 'op_patrol':{}}

    # Open the container
    with sm:
		sq = smach.Sequence(
				outcomes = ['success','failed'],
				connector_outcome = 'success')
		with sq:
			last_pose=[[]]
			
			smach.Sequence.add('StopLocalization', states.interaction.SendChoice('loc_kill'))
			smach.Sequence.add('WaitM1', states.interaction.Wait(10))
			if do_setup:
				smach.Sequence.add('Mapping1', states.initialization.ROSLaunch('cob_mapping_slam','2dslam.launch'))
				smach.Sequence.add('WaitM2', states.interaction.Wait(10))
				smach.Sequence.add('Mapping2', states.movement.Explore(sss))
				smach.Sequence.add('LastPose', states.movement.GetLastPose(last_pose))
				smach.Sequence.add('UI_MappingDone', states.interaction.ShowMenu('next'))
			
			smach.Sequence.add('Localization1', states.interaction.SendChoice('loc_start'))
			smach.Sequence.add('WaitL', states.interaction.Wait(10))
			if do_setup:
				smach.Sequence.add('Localization2', states.movement.AutoLocalize(sss, last_pose))
			else:
				smach.Sequence.add('Localization2', states.movement.AutoLocalize(sss))
			smach.Sequence.add('UI_LocalizationDone', states.interaction.ShowMenu('next'))
			
			if do_setup:
				smach.Sequence.add('SetPatrolPath', states.interaction.ReadVariableFromChoice('path', 'op_patrol.path'))
				smach.Sequence.add('SavePatrolPath', states.interaction.SaveYaml('op_patrol', 'config/op_patrol.yaml'))
				smach.Sequence.add('UI_SetPatrolPathDone', states.interaction.ShowMenu('next'))
				
				smach.Sequence.add('SetLightPoses', states.interaction.ReadVariableFromChoice('lights', 'op_small.lights'))
				smach.Sequence.add('SaveLightPoses', states.interaction.SaveYaml('op_small', 'config/op_small.yaml'))
				smach.Sequence.add('UI_SetLightPosesDone', states.interaction.ShowMenu('next'))
			
		smach.StateMachine.add('Main', sq, 
						   transitions={'success':'success',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main("--setup" in sys.argv[1:])