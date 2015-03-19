#!/usr/bin/python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros

from simple_script_server import simple_script_server

import states.interaction
import states.initialization

# main
def main():
    sss = simple_script_server()
    rospy.init_node('serodi_smach_overall')

    # Create a SMACH state machine
    logger = states.interaction.SMACHCustomLogger()
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_small':{}}
    sm.userdata.running_processes = {}
    
    LOC_PKG="serodi_mapping"
    LOC_BIN="nav_eband.launch"

    # Open the container
    with sm:
        smach.StateMachine.add('WaitBeforeInit', states.interaction.Wait(5), 
                               transitions={'success':'Init'})
        smach.StateMachine.add('Init', states.initialization.InitComponents(["base"], sss), 
                               transitions={'succeeded':'MainMenu', 'failed':'WaitBeforeInit'})
                               
        smach.StateMachine.add('MainMenu', states.interaction.WaitForChoice(["canceled", "setup", "shutdown", "localization", "scenario", "scenario_operation", "scenario_patrol", "loc_start", "loc_kill"]), 
                               transitions={'canceled': 'Program_None', 'unknown': 'MainMenu',
											'localization': 'Program_Localization',
											'shutdown': 'Program_Shutdown',
											'setup': 'Program_Setup',
											'scenario': 'Program_Scenario',
											'scenario_operation': 'Program_Scenario_Operation',
											'scenario_patrol': 'Program_Scenario_Patrol',
											
											'loc_start': 'Intern_Localization_Start',
											'loc_kill': 'Intern_Localization_Kill'
											})
			
        smach.StateMachine.add('Program_None', states.initialization.ROSKill(None, [LOC_PKG+' '+LOC_BIN] ), 
						   transitions={'success':'MainMenu'})
        smach.StateMachine.add('Program_Setup', states.initialization.ROSLaunch('serodi_smach','mapping.launch'), 
						   transitions={'success':'MainMenu',  'failed':'failure'})
        smach.StateMachine.add('Program_Localization', states.initialization.ROSLaunch('serodi_smach','localization.launch'), 
						   transitions={'success':'MainMenu',  'failed':'failure'})
        smach.StateMachine.add('Program_Scenario', states.initialization.ROSLaunch('serodi_smach','scenario.launch'), 
						   transitions={'success':'MainMenu',  'failed':'failure'})
        smach.StateMachine.add('Program_Scenario_Operation', states.initialization.ROSLaunch('serodi_smach','scenario_operation.launch'), 
						   transitions={'success':'MainMenu',  'failed':'failure'})
        smach.StateMachine.add('Program_Scenario_Patrol', states.initialization.ROSLaunch('serodi_smach','scenario_patrol.launch'), 
						   transitions={'success':'MainMenu',  'failed':'failure'})
        smach.StateMachine.add('Program_Shutdown', states.initialization.System('halt'), 
						   transitions={'success':'MainMenu',  'failed':'MainMenu'})
			
        smach.StateMachine.add('Intern_Localization_Start', states.initialization.ROSLaunch(LOC_PKG,LOC_BIN), 
						   transitions={'success':'MainMenu',  'failed':'MainMenu'})
        smach.StateMachine.add('Intern_Localization_Kill', states.initialization.ROSKill(LOC_PKG,LOC_BIN), 
						   transitions={'success':'MainMenu'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main()
