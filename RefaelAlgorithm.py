# we are going to make a reduction to the Dijkstra algorithm.


import itertools
from math import sqrt
import heapq
from ast import literal_eval

obstacles = [((3, 4), (8, 6)), ((2, 5), (8, 15)), ((1, 3), (7, 4))]


class Vertice:
    """
    This class represents a vertice. It is very convinient for our dijkstra use.
    """

    def __init__(self, x1, y1):
        self.x = x1
        self.y = y1
        self.d = 'inf'
        self.path = []

    def __lt__(self, other):  # for integrating with heapq
        if other.d != 'inf' and self.d != 'inf':
            return self.d < other.d
        else:
            if other.d == 'inf':
                return True
            else:
                return False

    def __str__(self):
        return "({},{}), distance : {}, path : {}".format(self.x, self.y, self.d, str([(i.x, i.y) for i in self.path]))


class Obstacle:
    """
    This class gets 4 coordinates that represents a rectangle obstacle.
    """

    def __init__(self, x1, y1, x2, y2):
        self.left_bottom = (x1, y1)
        self.left_top = (x1, y2)
        self.right_bottom = (x2, y1)
        self.right_top = (x2, y2)

    @property
    def get_vertices(self):
        return {self.left_top, self.left_bottom, self.right_bottom, self.right_top}


class Graph:
    """
    The graph's vertices is all of the corners of the rectangles, and the two points given (source, target) .
    The edges of the graph are between every two vertices that doesn't cross the obstacles.
    """

    def make_edges(self, vertices):
        """
        In this function , we try to make all of the possible edges between all of the vertices of the graph, if the
        path is clear (and not blocked by an obstacle).
        :param vertices: A list of all of the coordinates (Not Vertice Objects)
        """
        for element in itertools.product(vertices, vertices):
            if self.collided_path(*element) is False and (
                    len(self.adjacency_list[element[0]]['group_id']) == 0 or len(
                self.adjacency_list[element[1]]['group_id']) == 0 or
                    not set(self.adjacency_list[element[0]]['group_id']).isdisjoint(
                        self.adjacency_list[element[1]]['group_id']) is False):
                self.add_edge_and_update_adjacency_list(element[0], element[1])

    def collided_path(self, vertice1, vertice2):
        """
        This function returns a boolean if the way from vertice1 to vertice2 is blocked.
        :param vertice1: tuple represents a coordinate
        :param vertice2: tuple represents a coordinate
        :return: True if the way is blocked, False if the way is clear.
        """
        if vertice1 == vertice2:
            return True
        return self.check_if_collide_rectangle(vertice1, vertice2)

    def check_if_collide_rectangle(self, vertice1, vertice2):
        """
        This function returns a boolean if the way from vertice1 to vertice2 is blocked, by any edge of the graph.
        :param vertice1: tuple represents a coordinate
        :param vertice2: tuple represents a coordinate
        :return: True if the way is blocked by any edge of the graph, False if the way is clear.
        """
        return any(
            [self.check_if_collide_vertices(vertice1, vertice2, edge.start, edge.end) for edge in self.edges])

    def check_if_collide_vertices(self, vertice1, vertice2, vertice3, vertice4):
        """
        This method returns a boolean if the edge from v1 to v2 collides with the edge from v3 to v4.
        :param vertice1: tuple represents a coordinate
        :param vertice2: tuple represents a coordinate
        :param vertice3: tuple represents a coordinate
        :param vertice4: tuple represents a coordinate
        :return: True if the edges collide, False if they are not.
        """
        return self.check_if_collide(vertice1[0], vertice1[1], vertice2[0], vertice2[1], vertice3[0], vertice3[1],
                                     vertice4[0], vertice4[1])

    def check_if_collide(self, x1, y1, x2, y2, x3, y3, x4, y4):
        """
        This method checks if the line between the points (x1,y1) , (x2,y2) and the line between the point (x3,y3)(x4,y4)
         intersects.  If so, return True.
        :return: True if collide, False if not.
        """
        try:
            vertices_set = {(x1, y1), (x2, y2), (x3, y3), (x4, y4)}
            if 4 == len(vertices_set):
                ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
                ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
                if 0 <= ua <= 1 and 0 <= ub <= 1:
                    return True
            else:
                return False
        except ZeroDivisionError:
            return False

    def __init__(self, obstacles_list, source, target):
        self.source = source
        self.target = target
        vertices = self.make_vertices_from_obstacles(obstacles_list)
        vertices.update([source, target])
        self.vertices = vertices
        self.vertices_objects = set()
        self.adjacency_list = {el: {'neighbors': [], 'group_id': []} for el in self.vertices}
        self.obstacle_groups = [obstacle.get_vertices for obstacle in obstacles_list]
        self.make_edges_obstacles(obstacles_list)
        self.make_edges(self.vertices)

    def relax(self, neighbor_vertice, original_vertice, distance):
        """
        Relax function from Dijstkra algorithm .
        :return:
        """
        if neighbor_vertice.d == 'inf':
            return
        if original_vertice.d == 'inf' and neighbor_vertice != 'inf' or original_vertice.d > neighbor_vertice.d + distance:
            original_vertice.d = neighbor_vertice.d + distance
            original_vertice.path = neighbor_vertice.path.copy()
            original_vertice.path.append(original_vertice)

    def dijkstra(self):
        # DIJKSTRA #
        coordinates_to_vertices_dict = {}
        q = []
        target_vertice = None
        s = self.vertices_objects.copy()
        for i in self.vertices:
            vertice_object = Vertice(*i)
            if i == self.target:
                target_vertice = vertice_object
            if i == self.source:
                vertice_object.d = 0
                vertice_object.path.append(vertice_object)
            self.vertices_objects.add(vertice_object)
            coordinates_to_vertices_dict[i] = vertice_object
            heapq.heappush(q, vertice_object)
        while s != self.vertices_objects:
            obj = heapq.heappop(q)
            s.add(obj)
            for neighbor in self.adjacency_list[(obj.x, obj.y)]['neighbors']:
                self.relax(coordinates_to_vertices_dict[neighbor[0]], obj,
                           neighbor[1].distance)  # Vertice Object , Vertice Object, distance
                # Python heapq libary does not support decrease key option, so we have no choice but to do this action that cost us O(n)
                heapq.heapify(q)
        for i in s:
            if i == target_vertice:
                return i

    @staticmethod
    def make_vertices_from_obstacles(obstacles_l):
        """
        Build the rectangles from the obstacles.
        :param obstacles_l:
        :return:
        """
        vertices_list = []
        for obstacle in obstacles_l:
            vertices_list = set().union(obstacle.get_vertices, vertices_list)
        return vertices_list

    def make_edges_obstacles(self, obstacles_l):
        self.edges = []
        i = 0
        for obstacle in obstacles_l:
            self.add_edge_and_update_adjacency_list(obstacle.left_bottom, obstacle.left_top)
            self.add_edge_and_update_adjacency_list(obstacle.left_bottom, obstacle.right_bottom)
            self.add_edge_and_update_adjacency_list(obstacle.right_bottom, obstacle.right_top)
            self.add_edge_and_update_adjacency_list(obstacle.left_top, obstacle.right_top)
            self.adjacency_list[obstacle.left_bottom]['group_id'].append(i)
            self.adjacency_list[obstacle.left_top]['group_id'].append(i)
            self.adjacency_list[obstacle.right_bottom]['group_id'].append(i)
            self.adjacency_list[obstacle.right_top]['group_id'].append(i)

    def add_edge_and_update_adjacency_list(self, v1, v2):
        if v2 not in self.adjacency_list[v1]['neighbors']:
            e1 = Edge(v1, v2)
            self.edges.append(e1)
            self.adjacency_list[v1]['neighbors'].append((v2, e1))
            self.adjacency_list[v2]['neighbors'].append((v1, e1))


class Edge:

    @staticmethod
    def compute_distance(vertice1, vertice2):
        return sqrt((vertice2[0] - vertice1[0]) ** 2 + (vertice2[1] - vertice1[1]) ** 2)

    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.distance = self.compute_distance(start, end)


with open("input.txt", "r") as file:
    string = file.readline()[:-1]
    obstacle_l = literal_eval(string)
    source_target = literal_eval(file.readline())
obstacles_object_list = []
for obs in obstacle_l:
    obstacles_object_list.append(Obstacle(*obs[0], *obs[1]))
graph = Graph(obstacles_object_list, source_target[0], source_target[1])
s = graph.dijkstra()
print(s)
