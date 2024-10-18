ns=40
short_stance_duration=5
stance_duration=15
flight_duration=15



T=2.

c_timelines: contact_1_timeline; contact_2_timeline; contact_3_timeline; contact_4_timeline;
	contact_1_timeline: stance_phase(node number - 15, task - contact_1), flight_phase(node number - 15, task - flight_contact_1)
	contact_2_timeline: stance_phase(node number - 15, task - contact_1)
	contact_3_timeline: stance_phase(node number - 15, task - contact_1)
	contact_4_timeline: stance_phase(node number - 15, task - contact_1)
