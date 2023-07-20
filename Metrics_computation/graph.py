import numpy as np
from numpy import linalg as la


def angular_dist(points_normals, centers_normals):
    points_normals = np.reshape(points_normals, (-1, 3))
    centers_normals = np.reshape(centers_normals, (-1, 3))
    p_norm = np.tile(la.norm(points_normals, ord=2,
                             axis=1).transpose(), (3, 1))
    c_norm = np.tile(la.norm(centers_normals, ord=2,
                             axis=1).transpose(), (3, 1))
    p_normals_norm = points_normals/p_norm.transpose()
    c_normals_norm = centers_normals/c_norm.transpose()
    product = p_normals_norm @ c_normals_norm.transpose()
    angles = np.arccos(np.around(product.flatten(), 8))
    return float(angles)


class Graph:

    def __init__(self, num_of_vertices):
        self.v = num_of_vertices
        self.edges = [[-1 for i in range(num_of_vertices)] for j in range(num_of_vertices)]
        self.ang_edge = [[-1 for i in range(num_of_vertices)] for j in range(num_of_vertices)]
        self.visited = []

    def add_edge(self, u, v, weight, ang_dst):
        if ang_dst > np.pi/2:
            weight = -1
        self.edges[u][v] = weight
        self.edges[v][u] = weight
        self.ang_edge[u][v] = ang_dst
        self.ang_edge[v][u] = ang_dst

    def fill(self, normals, knn, indices):
        for current_point in indices:
            neighborhood = knn[current_point]
            for neighbor in neighborhood:
                ang_dst = angular_dist(normals[current_point, :], normals[neighbor, :])
                self.add_edge(int(current_point), neighbor, neighborhood[neighbor], ang_dst)
