ns=40
short_stance_duration=5
stance_duration=15
flight_duration=15

T=2.

c_timelines: contact_1_timeline; contact_2_timeline; contact_3_timeline; contact_4_timeline;
	contact_1_timeline: stance_phase(node number - 15, task - contact_1), stance_phase_short(node number - 5, task - contact_1), flight_phase(node number - 15, task - z_contact_1)
	contact_2_timeline: stance_phase(node number - 15, task - contact_2), stance_phase_short(node number - 5, task - contact_2), flight_phase(node number - 15, task - z_contact_2)
	contact_3_timeline: stance_phase(node number - 15, task - contact_3), stance_phase_short(node number - 5, task - contact_3), flight_phase(node number - 15, task - z_contact_3)
	contact_4_timeline: stance_phase(node number - 15, task - contact_4), stance_phase_short(node number - 5, task - contact_4), flight_phase(node number - 15, task - z_contact_4)

initial timeline





~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~original~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

==================== ALL PHASES: ====================

- 1: stance_contact_2: <phase_manager.pyphase.PhaseToken object at 0x7f1fc8bf3c30>

  |     Phase position: 0

  |     Phase duration: 15

  |     Active nodes of phase: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

  |_____________________________________________________________|

- 2: stance_contact_2: <phase_manager.pyphase.PhaseToken object at 0x7f1f4778f7b0>

  |     Phase position: 15

  |     Phase duration: 15

  |     Active nodes of phase: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

  |_____________________________________________________________|

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~shift~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

==================== ALL PHASES: ====================

- 1: stance_contact_2: <phase_manager.pyphase.PhaseToken object at 0x7f1f4778fdb0>

  |     Phase position: -1

  |     Phase duration: 15

  |     Active nodes of phase: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

  |_____________________________________________________________|

- 2: stance_contact_2: <phase_manager.pyphase.PhaseToken object at 0x7f1f4778f7b0>

  |     Phase position: 14

  |     Phase duration: 15

  |     Active nodes of phase: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

  |_____________________________________________________________|
