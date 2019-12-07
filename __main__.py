import lanelet2_parser as parser
import matplotlib.pyplot as plt

""" 
	Testing site:
	
	Runs results of Lanelet2 Parser on 
	example map provided in Lanelet2 repo.
	-----
	@author Francis Indaheng
	
	Developed with VeHICaL and all associated 
	groups, projects, and persons.
"""

if __name__ == '__main__':
	path = './maps/example_map.osm'
	data = parser.MapData(buffer_=0)
	data.parse(path, align_range=0.5)

	print(f'\nParsing file with path \'{path}\' ...\n')
	print(f'Points: {len(data.points)}')
	print(f'Linestrings: {len(data.linestrings)}')
	print(f'Polygons: {len(data.polygons)}')
	print(f'Areas: {len(data.areas)}')
	print(f'Lanelets: {len(data.lanelets)}')
	print(f'Regulatory elements: {len(data.regulatory_elements)}\n')
	print('... Parsing Complete\n')

	data.plot(type_='line')
	