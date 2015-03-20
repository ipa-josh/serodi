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
def main(do_patrol, do_lights):
    sss = simple_script_server()
    rospy.init_node('serodi_smach_prog_setup')

    # Create a SMACH state machine
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_small':{}, 'op_patrol':{}}
    sm.userdata.running_processes = {}

    mapping_launch = '2dnav_eband.launch'
	
    # Open the container
    with sm:				
		sq = smach.Sequence(
				outcomes = ['success','failed'],
				connector_outcome = 'success')
		sq.userdata = sm.userdata
		
		with sq:
			
			if do_patrol:
				smach.Sequence.add('LoadPatrolPath', states.interaction.LoadYaml('op_patrol', '../config/op_patrol.yaml'))
				smach.Sequence.add('SetParam_PatrolPath', states.interaction.SetROSParam('op_patrol','/ui/poses_patrol'))
				smach.Sequence.add('UI_SetPatrolPath', states.interaction.ShowMenu('next_patrol'))
				smach.Sequence.add('SetPatrolPath', states.interaction.ReadVariableFromChoice('path', 'op_patrol.path'))
				smach.Sequence.add('SavePatrolPath', states.interaction.SaveYaml('op_patrol', '../config/op_patrol.yaml'))
				smach.Sequence.add('UI_SetPatrolPathDone', states.interaction.ShowMenu('finished'))
				
			if do_lights:
				smach.Sequence.add('LoadLightPoses', states.interaction.LoadYaml('op_small', '../config/op_small.yaml'))
				smach.Sequence.add('SetParam_LightPoses', states.interaction.SetROSParam('op_small','/ui/poses_lights'))
				smach.Sequence.add('UI_SetLightPoses', states.interaction.ShowMenu('next_lights'))
				smach.Sequence.add('SetLightPoses', states.interaction.ReadVariableFromChoice('lights', 'op_small.lights'))
				smach.Sequence.add('SaveLightPoses', states.interaction.SaveYaml('op_small', '../config/op_small.yaml'))
				smach.Sequence.add('UI_SetLightPosesDone', states.interaction.ShowMenu('finished'))
				
			smach.Sequence.add('WaitBeforeFinish', states.interaction.Wait(10))
			
		smach.StateMachine.add('UpdatePoses', sq, 
						   transitions={'success':'success',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main("--patrol" in sys.argv[1:], "--lights" in sys.argv[1:])
