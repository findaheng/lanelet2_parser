import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import math
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import cascaded_union

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

	def __init__(self, id_, point, type_, subtype):
		self.id_ = id_
		self.point = point  # Shapely Point
		self.type_ = type_
		self.subtype = subtype


class L2_Linestring:
	''' Linestring representation of Lanelet2 Linestring primitive type
	using Shapely's LineString class '''

	def __init__(self, id_, linestring, type_, subtype):
		self.id_ = id_
		self.linestring = linestring  # Shapely LineString
		self.type_ = type_ 
		self.subtype = subtype


class L2_Polygon:
	''' Polygon representation of Lanelet2 Polygon primitive type
	using Shapely's Polygon class '''

	def __init__(self, id_, polygon, type_, subtype):
		self.id_ = id_
		self.polygon = polygon  # Shapely Polygon
		self.type_ = type_
		self.subtype = subtype


class Lanelet():
	''' Atomic lane defined by exactly one left and one right linestrings
	that represents directed traffic from entry to exit '''

	class Cell():
		''' Section of a lane, represented as a Shapely polygon, with a defined heading '''

		def __init__(self, polygon, heading):
			self.polygon = polygon  # Shapely polygon
			self.heading = heading  # radians clockwise from y-axis

		def contains_point(self, point):
			point = Point(point.x, point.y) if not isinstance(point, Point) else point  # convert to Shapely point if necessary
			return polygon.contains(point)

	def __init__(self, id_, subtype, 
					region, location, one_way, turn_direction,
					vehicle_participant, pedestrian_participant, bicycle_participant, 
					left_bound=None, right_bound=None, centerline=None, regulatory_elements=[]):
		self.id_ = id_
		self.subtype = subtype
		self.region = region
		self.location = location
		self.one_way = one_way
		self.turn_direction = turn_direction
		self.pedestrian_participant = pedestrian_participant
		self.bicycle_participant = bicycle_participant
		self.left_bound = left_bound  # L2_Linestring
		self.right_bound = right_bound  # L2_Linestring
		self.centerline = centerline  # L2_Linestring
		self.regulatory_elements = regulatory_elements

		# calculated fields to store property methods
		self.__polygon = None
		self.__cells = []

	@property
	def polygon(self):
		if self.__polygon:
			return self.__polygon

		left_bound_coords = list(self.left_bound.linestring.coords)
		right_bound_coords = list(self.right_bound.linestring.coords)

		''' NOTE: 
		(*)	Noticed that when always reversed the right bound, some of the
		lanelet polygons were Z-shaped instead of rectangular.
		(*)	Determined that there must not be a defined convention in the Lanelet2 format
		for the order in which points were given.
		(*)	Reversal will occur if bound "vectors" are not currently oriented head-to-tail,
		which can be determined by comparing the distance of the left bound vector-head
		to the head and tail of the right bound vector. '''
		left_head = Point(left_bound_coords[-1])  # last point of the left bound 
		right_tail = Point(right_bound_coords[0])  # first point of the right bound
		right_head = Point(right_bound_coords[-1])  # first point of the right bound
		if left_head.distance(right_head) < left_head.distance(right_tail):
			right_bound_coords.reverse()

		left_bound_coords.extend(right_bound_coords)
		self.__polygon = Polygon(left_bound_coords)
		return self.__polygon

	@property
	def cells(self):
		if self.__cells:
			return self.__cells

		# determine linestring with more points		
		num_right_pts = len(self.right_bound.linestring.coords)  # number of points in right bound linestring
		num_left_pts = len(self.left_bound.linestring.coords)  # number of points in left bound linestring
		if num_right_pts > num_left_pts:
			more_pts_linestr = self.right_bound.linestring
			less_pts_linestr = self.left_bound.linestring 
		else:
			more_pts_linestr = self.left_bound.linestring
			less_pts_linestr = self.right_bound.linestring

		# connect points from linestring (with more points) to other linestring (one with less points)
		# NOTE: heading defined by slope of line segements of linestring with more points
		more_pts_coords = more_pts_linestr.coords
		for i in range(len(more_pts_coords) - 1):
			curr_point = Point(more_pts_coords[i][0], more_pts_coords[i][1])  # convert to Shapely point
			next_point = Point(more_pts_coords[i+1][0], more_pts_coords[i+1][1])  # to compute second bound and heading

			# compute closes point (not necessarily a coordinate of) on other linestring
			bound_pt_1 = less_pts_linestr.interpolate(less_pts_linestr.project(next_point))
			bound_pt_2 = less_pts_linestr.interpolate(less_pts_linestr.project(curr_point)) 

			cell_polygon = Polygon([(p.x, p.y) for p in [curr_point, next_point, bound_pt_1, bound_pt_2]])

			# FIXME: should not be able to define heading based on linestring, since linestring might be used for multiple lanes
			cell_heading = math.atan((next_point.y - curr_point.y) / (next_point.x - curr_point.x)) + math.pi / 2  # since headings in radians clockwise from y-axis

			cell = self.Cell(cell_polygon, cell_heading)
			self.__cells.append(cell)
		return self.__cells

	def contains_point(self, point):
		point = Point(point.x, point.y) if not isinstance(point, Point) else point  # convert to Shapely point if necessary
		return self.polygon.contains(point)

	def heading_at(self, point):
		point = Point(point.x, point.y) if not isinstance(point, Point) else point  # convert to Shapely point if necessary
		cells_with_point = [cell for cell in self.cells if cell.contains_point(point)]
		cell = cells_with_point[0] if cells_with_point else None
		assert cell, f'Point with coordinates x={point.x}, y={point.y} not in lanelet with id={self.id_}'
		return cell.heading  # radians clockwise from y-axis


class Area():
	''' Ordered list of linestrings representing undirected traffic 
	that can have multiple entry and exit points '''

	def __init__(self, id_, outer_linestrings, inner_linestrings):
		self.id_ = id_
		self.outer_linestrings = outer_linestrings
		self.inner_linestrings = inner_linestrings

		self.__polygon = None  # store calculated polygon to avoid redundant calculations

	@property
	def polygon(self):
		if self.__polygon:
			return self.__polygon

		outer_bound_coords = []
		for l2_linestring in self.outer_linestrings:
			shapely_linestring = l2_linestring.linestring
			outer_bound_coords.extend(shapely_linestring.coords)

		inner_bound_coords = []
		for l2_linestring in self.inner_linestrings:
			shapely_linestring = l2_linestring.linestring
			inner_bound_coords.extend(shapely_linestring.coords)

		# minimum 3 coordinates needed to define a polygon
		if len(outer_bound_coords) < 3 or (inner_bound_coords and len(inner_bound_coords) < 3):
			print(f'Area with id={self.id_} does not have at least 3 coordinate tuples')
			self.__polygon = Polygon()

		self.__polygon = Polygon(outer_bound_coords, inner_bound_coords)
		return self.__polygon


class RegulatoryElement():
	''' Defines traffic rules, such as speed 
	limits, priority rules, or traffic lights '''

	def __init__(self, id_, subtype, fallback):
		self.id_ = id_
		self.subtype = subtype


class MapData:
	''' Parses an OSM-XML file to extract primitive 
	data types of the Lanelet2 framework'''

	def __init__(self):

		# low-level data
		self.points = {}  # L2_Points
		self.linestrings = {}  #L2_Linestrings
		self.polygons = {}  # L2_Polygons
		self.lanelets = {}
		self.areas = {}
		self.regulatory_elements = {}

		# single Shapely Polygon of drivable region
		self.__drivable_polygon = None  # store calculated polygon to avoid redundant calculations

		# store id's of regulatory elements to add to a lanelet objects after parsing completes (such that the regulatory elements have been processed)
		self.__todo_lanelets_regelems = []  # list of tuples in the form: (lanelet id, regulatory_element id)

	@property
	def drivable_polygon(self):
		if self.__drivable_polygon:
			return self.__drivable_polygon

		lanelet_polygons = [lanelet.polygon for lanelet in self.lanelets.values()]
		self.__drivable_polygon = cascaded_union(lanelet_polygons)
		return self.__drivable_polygon

	def heading_at(self, point):
		point = Point(point.x, point.y) if not isinstance(point, Point) else point # convert to Shapely Point if necessary
		for lanelet in self.lanelets:
			if lanelet.contains_point(point):
				for cell in lanelet.cells:
					if cell.contains_point(point):
						return cell.heading
				raise RuntimeError(f'Error finding point with coordinates x={point.x}, y={point.y} in cells of lanelet with id={lanelet.id_}')
		raise RuntimeError(f'Heading not defined at point with coordinates x={point.x}, y={point.y}')

	def plot(self, c='r'):
		''' Plot polygon representations of data fields on Matplotlib '''

		def __plot_polygon(polygon):
			''' Code from Wilson Wu's OpenDrive parser '''
			if not polygon.exterior:
				return
			x, y = polygon.exterior.xy
			plt.plot(x, y, c=c)
			for interior in polygon.interiors:
				x, y = interior.xy
				plt.plot(x, y, c=c)

		for poly in self.polygons.values():
			__plot_polygon(poly.polygon)

		# NOTE: uncomment to see drivable region
		#__plot_polygon(self.drivable_polygon)

		for lanelet in self.lanelets.values():
			
			# NOTE: uncomment to see cells
			for cell in lanelet.cells:
				__plot_polygon(cell.polygon)

			#__plot_polygon(lanelet.polygon)

		for area in self.areas.values():
			__plot_polygon(area.polygon)

		plt.show()

	def parse(self, path):

		# MARK: - HELPER METHODS

		def __extract_point(id_, x, y, z, type_, subtype):
			shapely_point = Point(x, y, z) if z else Point(x, y)
			self.points[id_] = L2_Point(id_, shapely_point, type_, subtype)

		def __extract_polygon(id_, polygon_coords, type_, subtype):
			shapely_polygon = Polygon(polygon_coords)
			self.polygons[id_] = L2_Polygon(id_, shapely_polygon, type_, subtype)

		def __extract_linestring(id_, linestring_coords, type_, subtype):
			shapely_linestring = LineString(linestring_coords)
			self.linestrings[id_] = L2_Linestring(id_, shapely_linestring, type_, subtype)

		def __extract_lanelet(id_, subtype, region, location, one_way, turn_dir, vehicle, pedestrian, bicycle, relation_element):
			lanelet = Lanelet(id_, subtype, region, location, one_way, turn_dir, vehicle, pedestrian, bicycle)

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

		def __extract_regulatory_element(id_, subtype, fallback):
			self.regulatory_elements[id_] = RegulatoryElement(id_, subtype, fallback)

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

			type_tag = None
			subtype_tag = None
			ele_tag = None
			for tag in node.iter('tag'):
				key = tag.get('k')
				value = tag.get('v')

				if key == 'type':
					type_tag = value
				elif key == 'subtype':
					subtype_tag = value
				elif key == 'ele':
					ele_tag = float(value)
				else:
					print(f'Unhandled node tag with key={key}')

			__extract_point(node_id, node_lat, node_lon, ele_tag, type_tag, subtype_tag)

		for way in root.iter('way'):
			# TODO: Check if nodes defined in way, and if they're distinct from nodes defined in root
			
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
			turn_direction_tag = None  # for lanelets
			one_way_tag = False  # for lanelets
			vehicle_tag = False  # for lanelets
			pedestrian_tag = False  # for lanelets
			bicycle_tag = False  # for lanelets
			fallback_tag = False  # for regulatory elements
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
				elif key == 'turn_direction':
					turn_direction_tag = value
				elif key == 'one_way':
					one_way_tag = True if value == 'true' else False
				elif key == 'participant:vehicle':
					vehicle_tag = True if value == 'true' else False
				elif key == 'participant:pedestrian':
					pedestrian_tag = True if value == 'yes' else False
				elif key == 'participant:bicycle':
					bicycle_tag = True if value == 'yes' else False
				elif key == 'fallback':
					fallback_tag = True if value == 'yes' else False
				else:
					print(f'Unhandled relation tag with key={key}')

			if type_tag == 'lanelet':
				__extract_lanelet(relation_id, subtype_tag, region_tag, location_tag, one_way_tag, turn_direction_tag, vehicle_tag, pedestrian_tag, bicycle_tag, relation)
			elif type_tag == 'multipolygon':  # area
				__extract_area(relation_id, relation)
			elif type_tag == 'regulatory_element':
				__extract_regulatory_element(relation_id, subtype_tag, fallback_tag)
			else:
				raise RuntimeError(f'Unknown relation type with id={relation_id}')

		__execute_todo()  # add stored unparsed regulatory elements to corresponding lanelets
