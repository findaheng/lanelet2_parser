import lgsvl
import math

"""
	@author Francis Indaheng
	
	Developed with VeHICaL and all associated 
	groups, projects, and persons.
"""

sim = lgsvl.Simulator(address="localhost", port=8181)

scenic_lgsvl_dict = {}  # maps Scenic objects to LGSVL agents' unique ids  

def use_sample(sample):
	printf(f'USE_SAMPLE called on sample={sample} ...\n')

	for obj in sample.objects:
		state = lgsvl.AgentState()  # NOTE: equivalent to Wilson's SPAWN variable
		state.transform.position = lgsvl.Vector(obj.position[0], obj.position[1], 0)
		state.transform.rotation.y = obj.heading * 180 / math.pi + 90  # NOTE: Wilson's calculations

		if obj.type == 'Car':
			name = 'Sedan'  # alt. names = ['Sedan', SUV', 'Jeep', 'Hatchback', 'SchoolBus', 'BoxTruck']
			type_ = lgsvl.AgentType.NPC
		elif obj.type == 'Pedestrian':
			name = 'Bob'  # alt. names = ['Bob', 'EntrepreneurFemale', 'Howard', 'Johny', 'Pamela', 'Presley', 'Red', 'Robin', 'Stephen', 'Zoe']
			type_ = lgsvl.AgentType.PEDESTRIAN
		else:
			print(f'Unsupported agent with type={obj.type}')
			continue

		agent = sim.add_agent(name, type_, state)
		scenic_lgsvl_dict[obj] = agent.uid
	
	print(f'... All agents in sample added.\n')


def get_state(obj):
	''' Returns the position and heading of a Scenic object's corresponding LGSVL agent at runtime '''

	uid = scenic_lgsvl_dict[scenic_obj]
	agent_lst = filter(lambda agent: agent.uid == uid, lgsvl.get_agents())

	assert agent_lst, f'Agent with id={uid} not currently available'

	agent = agent_lst[0]
	return agent.state.position, agent.state.rotation.y


def __test_run(sample):
	use_sample(sample)
	num_iter = 0
	for obj in sample.objects:
		pos, heading = get_state(obj)
		print(f'Current state of object {num_iter}: position={pos}, heading={heading}') 
		num_iter += 1
