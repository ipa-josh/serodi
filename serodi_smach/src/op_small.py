#!/usr/bin/python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros

from simple_script_server import script

import states.interaction
import states.movement

# main
def main():
    #rospy.init_node('serodi_smach_op_small')
    sss = script()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    
    sm.userdata.data = {'op_small':{}}

    # Open the container
    with sm:
        # Add states to the container
        
        # select random light
        # SetLight(red)
        # move to light
        # SetLight(green)
        # wait for X secs.
        # SetLight(off)
        
        smach.StateMachine.add('LoadYaml', states.interaction.LoadYaml('op_small', 'config/op_small.yaml', sm), 
                               transitions={'pass':'SelectLight'})

        #select random light
        transitions={'none':'success'}
        for light in sm.userdata.data['op_small']['lights']:
			transitions[light['name']] = 'OpSmall_'+light['name']
			
        smach.StateMachine.add('SelectLight', states.interaction.SelectRandom(transitions), transitions=transitions)
        
        for light in sm.userdata.data['op_small']['lights']:
			l = light['name']
			
			sq = smach.Sequence(
					outcomes = ['success','failed'],
					connector_outcome = 'success')
			with sq:
				smach.Sequence.add('SetLight_'+l+"_RED1", states.interaction.SetLight(sss,'red'))
				smach.Sequence.add('SetLight_'+l+"_RED2", states.interaction.SetLight(sss,'red', light))
				smach.Sequence.add('MoveTo_'+l, states.movement.MoveToPose(sss,light['pose']))
				smach.Sequence.add('SetLight_'+l+"_GREEN1", states.interaction.SetLight(sss,'green'))
				smach.Sequence.add('SetLight_'+l+"_GREEN2", states.interaction.SetLight(sss,'green', light))
				
			smach.StateMachine.add('OpSmall_'+l, sq, 
                               transitions={'success':'Wait1_'+l,  'failed':'failure'})
 			
 			smach.StateMachine.add('Wait1_'+l, states.interaction.WaitForChoice(["wait"], light['waiting_time']), 
                               transitions={'wait': 'Wait2_'+l, 'unknown': 'Wait1_'+l, 'canceled': 'OpSmall2_'+l})
 			smach.StateMachine.add('Wait2_'+l, states.interaction.WaitForChoice(["wait"]), 
                               transitions={'wait': 'OpSmall2_'+l, 'unknown': 'Wait2_'+l})
                               
			sq = smach.Sequence(
					outcomes = ['success','failed'],
					connector_outcome = 'success')
					
			with sq:
				#smach.Sequence.add('Wait_'+l, states.interaction.Wait(light['waiting_time']))
				smach.Sequence.add('MoveToHome_'+l, states.movement.MoveToPose(sss,'home'))
				smach.Sequence.add('SetLight_'+l+"_OFF1", states.interaction.SetLight(sss,[0.,0.,0.]))
				smach.Sequence.add('SetLight_'+l+"_OFF2", states.interaction.SetLight(sss,'off', light))
				
			smach.StateMachine.add('OpSmall2_'+l, sq, 
                               transitions={'success':'success',  'failed':'failure'})
                                            
    # Execute SMACH plan
    outcome = sm.execute()
    print "SMACH resulted in ",outcome


if __name__ == '__main__':
    main()
