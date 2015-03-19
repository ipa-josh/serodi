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
    
    sm.userdata.data = {'op_patrol_poses':{}, 'op_patrol_scans':{}}
    sm.userdata.nonblocking = False

    # Open the container
    with sm:        
        # just move on path (poses)
        
        smach.StateMachine.add('ShowMsg1', states.interaction.ShowMenu('Patrouille','/ui/msg'), 
                               transitions={'success':'LoadYaml'})
				
        smach.StateMachine.add('LoadYaml', states.interaction.LoadYaml('op_patrol_poses', '../config/op_patrol_backup_poses.yaml', sm), 
                               transitions={'success':'LoadYaml2',  'failed':'failure'})
        smach.StateMachine.add('LoadYaml2', states.interaction.LoadYaml('op_patrol_scans', '../config/op_patrol_backup_scans.yaml', sm), 
                               transitions={'success':'Patrol1',  'failed':'failure'})
			
        smach.StateMachine.add('Patrol1',  states.movement.MoveOnPathRel(sss,sm.userdata.data['op_patrol_poses']['path'],sm.userdata.data['op_patrol_scans']['scans']),
			transitions={'success':'ShowMsgReady',  'failed':'failure'})
        
        smach.StateMachine.add('ShowMsgReady', states.interaction.ShowMenu('Bereit','/ui/msg'), 
                               transitions={'success':'Patrol1'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main()
