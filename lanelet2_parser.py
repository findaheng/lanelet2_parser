import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import math
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
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

	def __init__(self, id_, metric_point, geo_point, type_, subtype):
		self.id_ = id_
		self.point = metric_point  # (Shapely Point) in meters
		self.geo_point = geo_point  # (Shapely Point) store x=longitude, y=latitude data
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

		# calculated fields for property methods
		self._polygon = None
		self._cells = []

	@property
	def polygon(self):
		if self._polygon:
			return self._polygon

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
		self._polygon = Polygon(left_bound_coords)#.buffer(1e-6)  # NOTE: Buffer to account for misaligned lanelets

		return self._polygon

	@property
	def cells(self):
		if self._cells:
			return self._cells

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
			curr_pt = Point(more_pts_coords[i][0], more_pts_coords[i][1])  # convert to Shapely point
			next_pt = Point(more_pts_coords[i+1][0], more_pts_coords[i+1][1])  # to compute second bound and heading

			# compute closes point (not necessarily a coordinate of) on other linestring
			bound_pt_1 = less_pts_linestr.interpolate(less_pts_linestr.project(next_pt))
			bound_pt_2 = less_pts_linestr.interpolate(less_pts_linestr.project(curr_pt)) 

			cell_polygon = Polygon([(p.x, p.y) for p in [curr_pt, next_pt, bound_pt_1, bound_pt_2]])

			# FIXME: (assuming) can define heading based on lanelet's right bound
			delta_x = next_pt.x - curr_pt.x
			cell_heading = math.atan((next_pt.y - curr_pt.y) / delta_x) + math.pi / 2 if delta_x else 0 # since headings in radians clockwise from y-axis

			cell = self.Cell(cell_polygon, cell_heading)
			self._cells.append(cell)

		return self._cells


class Area():
	''' Ordered list of linestrings representing undirected traffic 
	that can have multiple entry and exit points '''

	def __init__(self, id_, outer_linestrings, inner_linestrings):
		self.id_ = id_
		self.outer_linestrings = outer_linestrings
		self.inner_linestrings = inner_linestrings

		self._polygon = None  # store calculated polygon to avoid redundant calculations

	@property
	def polygon(self):
		if self._polygon:
			return self._polygon

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
			self._polygon = Polygon()
		else:
			self._polygon = Polygon(outer_bound_coords, inner_bound_coords)

		return self._polygon


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

		# store (lon, lat) as origin for reference when converting to meters
		self._origin = None

		# calculate fields for property methods
		self._drivable_polygon = None  # for interface's drivable polygonal region 
		self._cells = []    # for interface's polygonal vector field

		# store id's of regulatory elements to add to a lanelet objects after parsing completes (such that the regulatory elements have been processed)
		self._todo_lanelets_regelems = []  # list of tuples in the form: (lanelet id, regulatory_element id)

	@property
	def drivable_polygon(self, buffer_=1e-6):
		if self._drivable_polygon:
			return self._drivable_polygon

		# NOTE: buffer to account for misaligned lanelets
		lanelet_polygons = [lanelet.polygon.buffer(buffer_) for lanelet in self.lanelets.values() if lanelet.subtype != 'crosswalk']
		self._drivable_polygon = cascaded_union(lanelet_polygons)  # returns either a Shapely Polygon or MultiPolygon

		return self._drivable_polygon

	@property
	def cells(self):
		if self._cells:
			return self._cells

		for lanelet in self.lanelets.values():
			for cell in lanelet.cells:
				cell_heading = (cell.polygon, cell.heading)  # polygonal vector field takes a list of (polygon, heading) tuples
				self._cells.append(cell_heading)

		return self._cells
				
	def plot(self, c='r'):
		''' Plot polygon representations of data fields on Matplotlib '''

		# # # # # # # # # # # # # # 
		# MARK : - HELPER METHODS #
		# # # # # # # # # # # # # #

		def __plot_polygon(polygon, just_points=False, c=c):
			if just_points:
				__plot_polygon_points(polygon, c=c)
				return

			if not polygon.exterior:
				return

			x, y = polygon.exterior.coords.xy
			plt.plot(x, y, c=c)

			for interior in polygon.interiors:
				x, y = interior.coords.xy
				plt.plot(x, y, c=c)

		def __plot_multipolygon(multipolygon, just_points=False):
			for i, polygon in enumerate(multipolygon):
				__plot_polygon(polygon, just_points, c=['r', 'b', 'g', 'c', 'm', 'y', 'k'][i % 7])  # to differentiate polygons by color

		def __plot_drivable_polygon(just_points=False):
			# NOTE: checking type since cascaded union, which was used to compute drivable polygon, can return Polygon or MultiPolygon
			if isinstance(self.drivable_polygon, MultiPolygon):
				__plot_multipolygon(self.drivable_polygon, just_points)
			elif isinstance(self.drivable_polygon, Polygon):
				__plot_polygon(self.drivable_polygon, just_points)
			else:
				raise RuntimeError(f'Drivable polygon has unhandled type={type(self.drivable_polygon)}')

		def __plot_lanelet_cells(lanelet, just_points=False):
			for cell in lanelet.cells:
				__plot_polygon(cell.polygon, just_points)

		# NOTE: for testing purposes
		def __plot_polygon_points(polygon, c=c):
			''' Plots polygon's exterior points in red and interior points in blue '''

			if not polygon.exterior:
				return

			x, y = polygon.exterior.coords.xy
			coords_map = zip(x, y)
			for coord in coords_map:
				plt.plot(coord[0], coord[1], marker='x', c='r')

			for interior in polygon.interiors:
				x, y = interior.coords.xy
				coords_map = zip(x, y)
				for coord in coords_map:
					plt.plot(coord[0], coord[1], marker='x', c='b')

		# # # # # # # # # # # 
		# MARK: - PLOTTING  #
		# # # # # # # # # # # 

		for poly in self.polygons.values():
			__plot_polygon(poly.polygon)

		# NOTE: uncomment to see drivable region
		#__plot_drivable_polygon()

		for lanelet in self.lanelets.values():
			# NOTE: comment when trying see only drivable region
			__plot_polygon(lanelet.polygon)
			
			# NOTE: uncomment to see lanelet cells
			#__plot_lanelet_cells(lanelet)

			continue  # to avoid error if empty for-loop

		for area in self.areas.values():
			__plot_polygon(area.polygon)

		plt.show()

	def parse(self, path):
		''' Parse OSM-XML file that fulfills the Lanelet2 framework '''

		# # # # # # # # # # # # # # 
		# MARK : - HELPER METHODS #
		# # # # # # # # # # # # # #

		def __extract_point(id_, lon, lat, x, y, z, type_, subtype):
			''' Converts longitude and latitude to meters if x and y are unspecified'''

			if x and y:
				shapely_metric_point = Point(x, y, z) if z else Point(x, y)
			elif not self._origin:
				self._origin = (lon, lat)
				shapely_metric_point = Point(0, 0, 0) if z else Point(0, 0)
			else:
				# NOTE: lon/lat to meters calculation from https://stackoverflow.com/questions/3024404/transform-longitude-latitude-into-meters
				deltaLon = lon - self._origin[0]
				deltaLat = lat - self._origin[1]
				x = deltaLon * 40075160 * math.cos(self._origin[1] * math.pi / 180) / 360
				y = deltaLat * 40008000 / 360
				shapely_metric_point = Point(x, y, z) if z else Point(x, y)

			shapely_geo_point = Point(lon, lat)
			self.points[id_] = L2_Point(id_, shapely_metric_point, shapely_geo_point, type_, subtype)

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
					 	self._todo_lanelets_regelems.append((id_, ref_id))  # regulatory element not yet parsed -> add after parsing complete
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
			for lanelet_id, reg_elem_id in self._todo_lanelets_regelems:
				try:
					lanelet = self.lanelets[lanelet_id]
					reg_elem = self.regulatory_elements[reg_elem_id]
					lanelet.regulatory_elements.append(reg_elem)
				except:
					raise RuntimeError(f'Unknown regulatory element with id={reg_elem_id} referenced in lanelet with id={lanelet_id}')

		def __align_lanelets(percent_err=1):
			''' Align lanelet bounds to overlap exactly '''

			bounds = self.drivable_polygon.bounds  # (minx, miny, maxx, maxy)
			greater_dim = max(bounds[2] - bounds[0], bounds[3] - bounds[1])
			min_dist = greater_dim * percent_err / 100  # minimum distance two points before being combined

			lanelets = [v for v in self.lanelets.values()]
			for i in range(len(lanelets) - 1):
				curr = lanelets[i]
				curr_left = list(curr.left_bound.linestring.coords)
				curr_right = list(curr.right_bound.linestring.coords)
				curr_bound_pts = tuple(Point(coords) for coords in (curr_left[0], curr_left[-1], curr_right[0], curr_right[-1]))

				for j in range(i + 1, len(lanelets)):
					other = lanelets[j]
					other_left = list(other.left_bound.linestring.coords)
					other_right = list(other.right_bound.linestring.coords)
					other_bound_pts = tuple(Point(coords) for coords in (other_left[0], other_left[-1], other_right[0], other_right[-1]))

					for u in range(len(curr_bound_pts)):
						curr_pt = curr_bound_pts[u]

						for k in range(len(other_bound_pts)):
							other_pt = other_bound_pts[k]
							dist = curr_pt.distance(other_pt)

							if dist != 0 and dist < min_dist:
								avg_x = (curr_pt.x + other_pt.x) / 2
								avg_y = (curr_pt.y + other_pt.y) / 2

								if u == 0 or u == 1:  # replace left bound coordinates
									new_coords = curr_left
									new_coords[0 if u == 0 else -1] = (avg_x, avg_y)
									curr.left_bound.linestring = LineString(new_coords)
								else:  # replace right bound coordinates
									new_coords = curr_right
									new_coords[0 if u == 2 else -1] = (avg_x, avg_y)
									curr.right_bound.linestring = LineString(new_coords)

								if k == 0 or k == 1:  # replace left bound coordinates
									new_coords = other_left
									new_coords[0 if k == 0 else -1] = (avg_x, avg_y)
									other.left_bound.linestring = LineString(new_coords)
								else:  # replace right bound coordinates
									new_coords = other_right
									new_coords[0 if k == 2 else -1] = (avg_x, avg_y)
									other.right_bound.linestring = LineString(new_coords)

			# re-compute polygons
			for lanelet in self.lanelets.values():
				#lanelet._polygon = None  # FIXME: calculation creates self-intersection error with Shapely polygons for lanelets
				assert lanelet.polygon
			self._drivable_polygon = None
			assert self.drivable_polygon

		# # # # # # # # # # #
		# MARK : - PARSING  #
		# # # # # # # # # # #

		tree = ET.parse(path)
		root = tree.getroot()

		if root.tag != 'osm':
			raise RuntimeError(f'{path} does not appear to be an OSM-XML file')

		for node in root.iter('node'):
			node_id = int(node.get('id'))
			node_lon = float(node.get('lon'))
			node_lat = float(node.get('lat'))

			type_tag = None
			subtype_tag = None
			ele_tag = None
			x_tag = None
			y_tag = None 
			for tag in node.iter('tag'):
				key = tag.get('k')
				value = tag.get('v')

				if key == 'type':
					type_tag = value
				elif key == 'subtype':
					subtype_tag = value
				elif key == 'ele':
					ele_tag = float(value)
				elif key == 'x':
					x_tag	= float(value)
				elif key == 'y':
					y_tag = float(value)
				else:
					print(f'Unhandled node tag with key={key}')

			__extract_point(node_id, node_lat, node_lon, x_tag, y_tag, ele_tag, type_tag, subtype_tag)

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
		__align_lanelets()  # ensure lanelet endpoints overlap exactly
