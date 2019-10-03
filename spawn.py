import lgsvl
import math

"""
	@author Francis Indaheng
	
	Developed with VeHICaL and all associated 
	groups, projects, and persons.
"""

sim = lgsvl.Simulator(address="localhost", port=8181)  

def use_sample(sample):
	printf(f'USE_SAMPLE called on sample={sample} ...\n')

	for obj in sample.objects:

		state = lgsvl.AgentState()  # equivalent to Wilson's SPAWN variable
		state.transform.position = lgsvl.Vector(obj.position[0], obj.position[1], 0)  # no z-coordinate in Scenic
		state.transform.rotation.y = obj.heading * 180 / math.pi + 90  # calculation from Wilson's Carla simulator

		if obj.type == 'Car':
			name = 'Sedan'  # alt. names = ['Sedan', SUV', 'Jeep', 'Hatchback', 'SchoolBus', 'BoxTruck']
			type_ = lgsvl.AgentType.NPC
		elif obj.type == 'Pedestrian'
			name = 'Bob'  # alt. names = ['Bob', 'EntrepreneurFemale', 'Howard', 'Johny', 'Pamela', 'Presley', 'Red', 'Robin', 'Stephen', 'Zoe']
			type_ = lgsvl.AgentType.PEDESTRIAN
		else:
			print(f'Unsupported agent with type={obj.type}')
			continue

		sim.add_agent(name, type_, state)
	
	print(f'... All agents in sample added.\n')


def get_state():
	''' Looks up the current positions and headings of all agents at runtime '''

	print('Retrieving current state of all agents ...\n')

	agents = lgsvl.get_agents()  # method that returns a list of currently available agent objects added with add_agent

	print(f'... Found {len(agents)} agents.')

	for agent in agents:
		print(f'name={agent.name}, type={'car' if agent.agent_type == lgsvl.AgentType.NPC else 'pedestrian'}, position={agent.state.transform.position}, heading={agent.state.transform.rotation.y}')

	return agents
