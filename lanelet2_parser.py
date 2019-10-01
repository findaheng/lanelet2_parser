import xml.etree.ElementTree as ET
from shapely.geometry import Point, LineString, Polygon

""" 
	Lanelet2 parser for LGSVL Simulator:

	Parses an OSM-XML file that uses the Lanelet2 framework 
	and stores its data as fields of a MapData class.
	-----
	@author Francis Indaheng
	
	Developed with VeHICaL and all associated 
	groups, projects, and persons.
"""

class L2_Point:
	''' Point representation of Lanelet2 Point primitive type
	using Shapely's Point class '''

	def __init__(self, id_, point=None):
		self.id_ = id_
		self.point = point  # Shapely Point


class L2_Linestring:
	''' Linestring representation of Lanelet2 Linestring primitive type
	using Shapely's LineString class '''

	def __init__(self, id_, linestring, type_, subtype=None):
		self.id_ = id_
		self.linestring = linestring  # Shapely LineString
		self.type_ = type_ 
		self.subtype = subtype


class L2_Polygon:
	''' Polygon representation of Lanelet2 Polygon primitive type
	using Shapely's Polygon class '''

	def __init__(self, id_, polygon, type_, subtype=None):
		self.id_ = id_
		self.polygon = polygon  # Shapely Polygon
		self.type_ = type_
		self.subtype = subtype


class Lanelet():
	''' Atomic lane defined by exactly one left and one right linestrings
	that represents directed traffic from entry to exit '''

	def __init__(self, id_, subtype, region, location, one_way, pedestrian_participant, bicycle_participant, left_bound=None, right_bound=None, centerline=None, regulatory_elements=[]):
		self.id_ = id_
		self.subtype = subtype
		self.region = region
		self.location = location
		self.one_way = one_way
		self.pedestrian_participant = pedestrian_participant
		self.bicycle_participant = bicycle_participant
		self.left_bound = left_bound
		self.right_bound = right_bound 
		self.centerline = centerline
		self.regulatory_elements = regulatory_elements

	def calculate_polygon():
		left_bound_shapely_coords = list(left_bound.linestring.coords)
		right_bound_shapely_coords = list(right_bound.linestring.coords)
		polygon_coords = left_bound_shapely_coords + right_bound_shapely_coords.reverse()
		return Polygon(polygon_coords)


class Area():
	''' Ordered list of linestrings representing undirected traffic 
	that can have multiple entry and exit points '''

	def __init__(self, id_, outer_linestrings, inner_linestrings):
		self.id_ = id_
		self.outer_linestrings = outer_linestrings
		self.inner_linestrings = inner_linestrings


class RegulatoryElement():
	''' Defines traffic rules, such as speed 
	limits, priority rules, or traffic lights '''

	def __init__(self, id_, subtype):
		self.id_ = id_
		self.subtype = subtype


class MapData:
	''' Parses an OSM-XML file to extract primitive 
	data types of the Lanelet2 framework'''

	def __init__(self):
		self.points = {}  # L2_Points
		self.linestrings = {}  #L2_Linestrings
		self.polygons = {}  # L2_Polygons
		self.lanelets = {}
		self.areas = {}
		self.regulatory_elements = {}

		# store id's of regulatory elements to add to a lanelet objects after parsing completes (such that the regulatory elements have been processed)
		self.__todo_lanelets_regelems = []  # list of tuples in the form: (lanelet id, regulatory_element id)

	def parse(self, path):

		# MARK: - HELPER METHODS

		def __extract_point(id_, x, y):
			shapely_point = Point(x, y)
			self.points[id_] = L2_Point(id_, shapely_point)

		def __extract_polygon(id_, polygon_coords, type_, subtype):
			shapely_polygon = Polygon(polygon_coords)
			self.polygons[id_] = L2_Polygon(id_, shapely_polygon, type_, subtype)

		def __extract_linestring(id_, linestring_coords, type_, subtype):
			shapely_linestring = LineString(linestring_coords)
			self.linestrings[id_] = L2_Linestring(id_, shapely_linestring, type_, subtype)

		def __extract_lanelet(id_, subtype, region, location, one_way, pedestrian, bicycle, relation_element):
			lanelet = Lanelet(id_, subtype, region, location, one_way, pedestrian, bicycle)

			for member in relation_element.iter('member'):
				member_role = member.get('role')
				ref_id = int(member.get('ref'))

				if member_role == 'left':
					lanelet.left_bound = self.linestrings[ref_id]
				elif member_role == 'right':
					lanelet.right_bound = self.linestrings[ref_id]
				elif member_role == 'centerline':
					lanelet.centerline = self.linestrings[ref_id]
				elif member_role == 'regulatory_element':
					try:
						reg_elem = self.regulatory_elements[ref_id]
						lanelet.regulatory_elements.append(reg_elem)
					except:
					 	self.__todo_lanelets_regelems.append((id_, ref_id))  # regulatory element not yet parsed -> add after parsing complete
				else:
					raise RuntimeError(f'Unknown member role in lanelet with id={id_}')

			assert lanelet.left_bound and lanelet.right_bound, f'Lanelet with id={id_} missing bound(s)'  
			self.lanelets[id_] = lanelet

		def __extract_area(id_, relation_element):
			for member in relation_element.iter('member'):
				member_role = member.get('role')
				ref_id = int(member.get('ref'))

				outer = []  # id's of linestrings forming outer bound
				inner = []  # id's of linestrings forming inner hole

				if member_role == 'outer':
					outer.append(self.linestrings[ref_id])
				elif member_role == 'inner':
					inner.append(self.linestrings[ref_id])
				else:
					raise RuntimeError(f'Unknown member role={member_role} in area with id={id_}')

			self.areas[id_] = Area(id_, outer, inner)

		def __extract_regulatory_element(id_, subtype):
			self.regulatory_elements[id_] = RegulatoryElement(id_, subtype)

		def __execute_todo():
			for lanelet_id, reg_elem_id in self.__todo_lanelets_regelems:
				try:
					lanelet = self.lanelets[lanelet_id]
					reg_elem = self.regulatory_elements[reg_elem_id]
					lanelet.regulatory_elements.append(reg_elem)
				except:
					raise RuntimeError(f'Unknown regulatory element with id={reg_elem_id} referenced in lanelet with id={lanelet_id}')

		# MARK: - PARSING

		tree = ET.parse(path)
		root = tree.getroot()

		if root.tag != 'osm':
			raise RuntimeError(f'{path} does not appear to be an OSM-XML file')

		for node in root.iter('node'):
			node_id = int(node.get('id'))
			node_lat = float(node.get('lat'))
			node_lon = float(node.get('lon'))
			__extract_point(node_id, node_lat, node_lon)

		for way in root.iter('way'):
			way_id = int(way.get('id'))
			__ref_point_ids = [int(point.get('ref')) for point in way.findall('nd')]
			__ref_points = [self.points[id_] for id_ in __ref_point_ids]
			ref_point_coords = [(L2_point.point.x, L2_point.point.y) for L2_point in __ref_points]
			
			area_tag = False  # area='yes' tag indicates polygon
			type_tag = None
			subtype_tag = None
			for tag in way.iter('tag'):
				key = tag.get('k')
				value = tag.get('v')

				if key == 'area':
					area_tag = True if value == 'yes' else False
				elif key == 'type':
					type_tag = value
				elif key == 'subtype':
					subtype_tag = value
				else:
					print(f'Unhandled way tag with key={key}')

			if area_tag:  # polygon
				__extract_polygon(way_id, ref_point_coords, type_tag, subtype_tag)
			else:  # linestring
				__extract_linestring(way_id, ref_point_coords, type_tag, subtype_tag)

		for relation in root.iter('relation'):
			relation_id = int(relation.get('id'))

			type_tag = None
			subtype_tag = None
			region_tag = None
			location_tag = None
			one_way_tag = False
			pedestrian_tag = False
			bicycle_tag = False
			for tag in relation.iter('tag'):
				key = tag.get('k')
				value = tag.get('v')

				if key == 'type':
					type_tag = value
				elif key == 'subtype':
					subtype_tag = value
				elif key == 'region':
					region_tag = value
				elif key == 'location':
					location_tag = value
				elif key == 'one_way':
					one_way_tag = True if value == 'true' else False
				elif key == 'participant:pedestrian':
					pedestrian_tag = True if value == 'yes' else False
				elif key == 'participant:bicycle':
					bicycle_tag = True if value == 'yes' else False
				else:
					print(f'Unhandled relation tag with key={key}')

			if type_tag == 'lanelet':
				__extract_lanelet(relation_id, subtype_tag, region_tag, location_tag, one_way_tag, pedestrian_tag, bicycle_tag, relation)
			elif type_tag == 'multipolygon':  # area
				__extract_area(relation_id, relation)
			elif type_tag == 'regulatory_element':
				__extract_regulatory_element(relation_id, subtype_tag)
			else:
				raise RuntimeError(f'Unknown relation type with id={relation_id}')

		__execute_todo()  # add stored unparsed regulatory elements to corresponding lanelets
