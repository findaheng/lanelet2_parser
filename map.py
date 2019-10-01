import lanelet2_parser as parser  #NOTE: Must be in same directory as this file
from shapely.geometry import Point, LineString, Polygon

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
	# NOTE: No name/id info for roads in l2 format like in xodr
	def __init__(self):
		self.lanelets = []  # ordered from left to right
		self.regulatory_elements = []

	def calculate_polygon(self):
		assert lanelets, 'Road contains no lanelets'
		if len(lanelets) == 1:
			return lanelets[0].calculate_polygon()
		else:
			## TODO: LEFT OFF HERE

class RoadLink:
	''' Indicates two Roads are connected such that
	Road with id=id_a leads to Road with id=id_b '''

	def __init__(self, id_a, id_b, ):
		self.id_a = id_a
		self.id_b = id_b


class Junction:
	''' Junction class from Wilson Wu's Open Drive parser '''

	class Connection:
		def __init__(self, incoming_id, connecting_id, connecting_contact):
			self.incoming_id = incoming_id
			self.connecting_id = connecting_id  # id of conecting road
			self.connecting_contact = connecting_contact  # contact point ('start' or end') on connecting road

	def __init__(self, id_):
		self.id_ = id_
		self.connections = []

	def add_connection(self, incoming_id, connection_id, connecting_contact):
		self.connections.append(
			Junciton.Connection(incoming_id, 
								connecting_id, 
								connecting_contact))


class MapRepresentation:
	''' Uses map data extracted from an OSM-XML file to construct 
	an higher level representation of the map layout '''

	def __init__(self, path):
		self.__map_data = parser.MapData()
		__map_data.parse(path)

		self.roads = {}
		self.junctions = {}

	def construct(self):
		# NOTE: How to go about constructing an organized map from data with no indication of placement in relation to one another -- where to start?
