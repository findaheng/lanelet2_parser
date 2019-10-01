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
	def __init__(self, drive_on_right=True):
		self.lanelets = []
		self.drive_on_right = drive_on_right


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

