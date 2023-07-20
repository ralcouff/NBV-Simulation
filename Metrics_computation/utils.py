import argparse
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from queue import PriorityQueue
from graph import Graph
from matplotlib import cm


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def make_pca(data: np.ndarray, n_components: int):
    """
    Compute the PCA on the data provided
    :param n_components: n_components for the PCA
    :param data: Array containing data
    :return:
    """
    x = pd.DataFrame(data)
    x_mean = x.mean()
    xc = x - x_mean
    xc = xc.to_numpy()
    pca = PCA(n_components=n_components)
    pca.fit(xc)
    d = pca.singular_values_
    w = pca.components_
    new_coord = xc @ np.transpose(w)
    pct = pca.explained_variance_
    return w, d, new_coord, pct


def compute_bbox(points: np.ndarray):
    """
    Computes the Bounding Box of a point cloud
    :param points: The points of the point cloud
    :return:
    """
    mini = np.min(points, axis=0)
    maxi = np.max(points, axis=0)
    return np.array([[a] + [b] for a, b in zip(mini, maxi)]).flatten()


def compute_diagonal(vertices: np.ndarray):
    """
    Compute the diagonal of a set of data represented by an array
    :param vertices: Array containing the points
    :return: the diagonal of the object represented by the points
    """
    if len(vertices):
        (min_x, max_x, min_y, max_y, min_z, max_z) = compute_bbox(vertices)
        diag = np.sqrt(np.power(max_x - min_x, 2) + np.power(max_y - min_y, 2) + np.power(max_z - min_z, 2))
    else:
        diag = 0
    return diag


def dijkstra(graph: Graph, start_vertex: int):
    """
    Dijkstra algorithm to compute a geodesic distance and an angular one between points of a graph
    :param graph: A graph defining distances and connections between points
    :param start_vertex: The start vertex of the graph
    :return: the geodesic distance and the angular distance
    """
    d = {v: float('inf') for v in range(graph.v)}
    a = {v: float('inf') for v in range(graph.v)}
    d[start_vertex] = 0
    a[start_vertex] = 0

    pq = PriorityQueue()
    pq.put((0, int(start_vertex)))

    while not pq.empty():
        (dist, current_vertex) = pq.get()
        graph.visited.append(current_vertex)

        for neighbor in range(graph.v):
            # if type(neighbor) != int and type(neighbor) != np.int64:
            #     print('Ploup')
            #     print(type(neighbor))
            # if type(current_vertex) != np.int64 and type(current_vertex) != int:
            #     print("ABDHFZBFZU")
            #     print(type(current_vertex))
            if graph.edges[current_vertex][neighbor] != -1:
                distance = graph.edges[current_vertex][neighbor]
                ang_distance = graph.ang_edge[current_vertex][neighbor]
                if neighbor not in graph.visited:
                    old_cost = d[neighbor]
                    new_cost = d[current_vertex] + distance
                    old_angle = a[current_vertex]
                    new_angle = old_angle + ang_distance
                    if new_cost < old_cost:
                        pq.put((new_cost, int(neighbor)))
                        d[neighbor] = new_cost
                        a[neighbor] = new_angle
    return d, a


def lstsq_quadrics_fitting(pos_xyz):
    """
    Fit a given set of 3D points (x, y, z) to a quadrics of equation ax^2 + by^2 + cxy + dx + ey + f = z
    :param pos_xyz: A two-dimensional numpy array, containing the coordinates of the points
    :return:
    """
    row_num = pos_xyz.shape[0]
    A = np.ones((row_num, 6))
    A[:, 0:2] = np.square(pos_xyz[:, 0:2], pos_xyz[:, 0:2])
    A[:, 2] = np.multiply(pos_xyz[:, 0], pos_xyz[:, 1])
    A[:, 3:5] = pos_xyz[:, 0:2]

    f = pos_xyz[:, 2]

    sol, _, rank, singular_values = np.linalg.lstsq(A, f, rcond=None)

    residuals = f - A @ sol

    return sol, residuals


def lstsq_plane_fitting(pos_xyz):
    """
    Fit a given set of 3D points (x, y, z) to a plane of equation ax + by + cz = d
    :param pos_xyz: A two-dimensional numpy array, containing the coordinates of the points
    :return:
    """
    row_num = pos_xyz.shape[0]
    A = np.ones((row_num, 4))
    A[:, 0:3] = pos_xyz

    _, _, vh = np.linalg.svd(A)
    sol = vh[-1, :]

    return sol


def compute_orth_dist(pos_xyz, plane):
    """
    Compute the orthogonal distance between the points and the plane
    :param pos_xyz: A two-dimensional numpy array, containing the coordinates of the points
    :param plane: The parameters of the plane
    :return:
    """
    plane_norm = plane / np.linalg.norm(plane[0:3])
    row_num = pos_xyz.shape[0]
    A = np.ones((row_num, 4))
    A[:, :3] = pos_xyz
    orth_dist = A @ plane_norm
    return orth_dist


def generate_colormap(distances: np.ndarray, c_type: str = 'seismic'):
    """
    Generates a colormap according to the values contained in distances
    :param distances: a numpy.ndarray containing data from which create a colormap
    :param c_type: the type of colormap, it's a matplotlib colormap type
    :return:
    """
    colormap = []
    q25 = np.percentile(distances, 25)
    q75 = np.percentile(distances, 75)
    iqr = q75-q25

    min_dist = np.min(distances)
    max_dist = np.max(distances)
    normed_dist = distances / (q75 + 1.5*iqr)
    col = cm.get_cmap(c_type, 256)
    if min_dist != max_dist:
        for dist in normed_dist:
            if dist < (q25-1.5*iqr)/(q75 + 1.5*iqr) or dist > 1:
                colormap.append([255, 0, 255])
            else:
                colormap.append([255 * i for i in col(float(dist))])
    else:
        print("Error")
        n = len(distances)
        colormap = n * [[0, 255, 0]]
    return colormap
