import lanelet2_parser as parser

""" 
	High-level representation of a map:

	Processes Lanelet2 data into objects that
	Scenic can understand and use.
	-----
	@author Francis Indaheng
	
	Developed with VeHICaL and all associated 
	groups, projects, and persons.
"""

class Road:
	def __init__(self, drive_on_right=True):
		self.lanelets = []
		self.drive_on_right = drive_on_right


class MapRepresentation:
	''' Uses map data extracted from an OSM-XML file to construct 
	an higher level representation of the map layout '''

	def __init__(self):
		self.roads = {}
		self.junctions = {}
