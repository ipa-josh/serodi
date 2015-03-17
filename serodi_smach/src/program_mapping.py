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
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_small':{}, 'op_patrol':{}}
    sm.userdata.running_processes = {}

    # Open the container
    with sm:
		sq = smach.Sequence(
				outcomes = ['success','failed'],
				connector_outcome = 'success')
		sq.userdata = sm.userdata
		with sq:
			last_pose=[[]]
			
			smach.Sequence.add('StopLocalization', states.interaction.SendChoice('loc_kill'))
			smach.Sequence.add('WaitM1', states.interaction.Wait(10))
			if do_setup:
				smach.Sequence.add('Mapping1', states.initialization.ROSLaunch('serodi_mapping','2dnav_ros_dwa.launch'))
				smach.Sequence.add('WaitM2', states.interaction.Wait(10))
				smach.Sequence.add('Mapping2', states.movement.Explore(sss))
				smach.Sequence.add('SaveMap', states.initialization.System("rosrun map_server map_server ~/map.yaml"))
				smach.Sequence.add('LastPose', states.movement.GetLastPose(last_pose))
				smach.Sequence.add('UI_MappingDone', states.interaction.ShowMenu('next'))
			
			smach.Sequence.add('Localization1', states.interaction.SendChoice('loc_start'))
			smach.Sequence.add('WaitL', states.interaction.Wait(10))
		smach.StateMachine.add('Main', sq, 
						   transitions={'success':'Localization2',  'failed':'failure'})
			
		if do_setup:
			smach.StateMachine.add('Localization2', states.movement.AutoLocalize(sss, last_pose),
				transitions={'success': 'Main2', 'failed': 'failure'})
		else:
 			smach.StateMachine.add('Localization2', states.interaction.WaitForChoice(["loc_pos_global","loc_pos_zero"]), 
                               transitions={'loc_pos_global': 'Localization2_Global', 'unknown': 'Localization2', 'loc_pos_zero': 'Localization2_Zero'})
                        
			smach.StateMachine.add('Localization2_Global', states.interaction.ShowMenu('next'),
				transitions={'success': 'Localization2_Global2'})
			smach.StateMachine.add('Localization2_Global2', states.movement.AutoLocalize(sss, [0,0,0]),
				transitions={'success': 'Main2', 'failed': 'failure'})
				
			smach.StateMachine.add('Localization2_Zero', states.interaction.ShowMenu('next'),
				transitions={'success': 'Localization2_Zero2'})
			smach.StateMachine.add('Localization2_Zero2', states.movement.AutoLocalize(sss),
				transitions={'success': 'Main2', 'failed': 'failure'})
				
		sq = smach.Sequence(
				outcomes = ['success','failed'],
				connector_outcome = 'success')
		sq.userdata = sm.userdata
		with sq:
			smach.Sequence.add('UI_LocalizationDone', states.interaction.ShowMenu('next'))
			
			if do_setup:
				smach.Sequence.add('SetPatrolPath', states.interaction.ReadVariableFromChoice('path', 'op_patrol.path'))
				smach.Sequence.add('SavePatrolPath', states.interaction.SaveYaml('op_patrol', '../config/op_patrol.yaml'))
				smach.Sequence.add('UI_SetPatrolPathDone', states.interaction.ShowMenu('next'))
				
				smach.Sequence.add('SetLightPoses', states.interaction.ReadVariableFromChoice('lights', 'op_small.lights'))
				smach.Sequence.add('SaveLightPoses', states.interaction.SaveYaml('op_small', '../config/op_small.yaml'))
				smach.Sequence.add('UI_SetLightPosesDone', states.interaction.ShowMenu('next'))
			
		smach.StateMachine.add('Main2', sq, 
						   transitions={'success':'success',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main("--setup" in sys.argv[1:])
