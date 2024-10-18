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
