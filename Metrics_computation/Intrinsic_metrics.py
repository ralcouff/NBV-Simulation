import pandas as pd
import numpy as np
from sklearn.neighbors import NearestNeighbors
import utils
import time
import os
import errno


class IntrinsicMetrics:

    def __init__(self, obj_name: str, points: np.ndarray, normals: np.ndarray, n_neighbors: int = 20,
                 edges: np.ndarray = []):
        self._n_neighbors = n_neighbors
        self._coords = pd.DataFrame(data=points, columns=['x', 'y', 'z'])
        self._normals = pd.DataFrame(data=normals, columns=['x', 'y', 'z'])
        self._points = pd.DataFrame(columns=['plane_roughness', 'quad_roughness', 'curvature'])
        self._params = pd.DataFrame(columns=['obj_name', 'n_vertex', 'diagonal', 'n_neighbors', 'ctime'])
        self._quadrics = pd.DataFrame(columns=['a', 'b', 'c', 'd', 'e', 'f'])
        self.set_params_param('obj_name', obj_name)
        self.set_params_param('n_vertex', self.npoints)
        self.set_params_param('diagonal', utils.compute_diagonal(self.coords))
        self.set_params_param('n_neighbors', n_neighbors)
        self.set_params_param('ctime', time.time())
        self._edges = edges
        # self._knn = self.compute_knn(n_neighbors)
        self._neighbors, self._knn = self.compute_knn_from_edges(n_neighbors)

    @property
    def coords(self):
        return self._coords.to_numpy()

    @property
    def obj_name(self):
        return str(self.get_params_param('obj_name').to_numpy()[0])

    @property
    def normals(self):
        return self._normals.to_numpy()

    @property
    def npoints(self):
        return int(self._coords.shape[0])

    @property
    def knn(self):
        return self._knn

    @knn.setter
    def knn(self, new_knn: dict):
        self._knn = new_knn

    @property
    def n_neighbors(self):
        return int(self._params['n_neighbors'].to_numpy())

    @property
    def points(self):
        return self._points

    @property
    def ctime(self):
        return float(self.get_params_param('ctime'))

    @property
    def edges(self):
        return self._edges

    @ctime.setter
    def ctime(self, ctime: float):
        self.set_params_param('ctime', ctime)

    def set_params_param(self, param: str, value):
        self._params.at[0, param] = value

    def get_params_param(self, param: str):
        return self._params[param]

    def set_points_param(self, index: int, param: str, value):
        self._points.at[index, param] = value

    def get_points_param(self, index, param: str):
        return self._points.at[index, param]

    def get_points_column(self, param: str):
        return self._points[param].to_numpy()

    def compute_knn(self, n_neighbors: int):
        """
        Computes the k-nearest neighbors of each point of the cloud
        :param n_neighbors: the number of neighbors to consider
        :return knn:
        """
        x_data = self.coords
        knn = NearestNeighbors(n_neighbors=n_neighbors + 1, algorithm='auto').fit(x_data)
        distances, indices = knn.kneighbors(x_data)
        knn_dict = {i: {ind: dst for ind, dst in zip(indices[i][1:], distances[i][1:])} for i in range(self.npoints)}
        return knn_dict

    def compute_knn_from_edges(self, n_ring: int):
        neighbors = {i: set() for i in range(self.npoints)}
        for e in self.edges:
            neighbors[e[0]].add(e[1])
            neighbors[e[1]].add(e[0])
        knn_dict = {i: {ind: 0 for ind in self.get_n_neighbors(neighbors, n_ring, i, set())}
                    for i in range(self.npoints)}
        return neighbors, knn_dict

    def get_n_neighbors(self, neighbors: dict, depth: int, point_index: int, n_set: set):
        if depth == 0:
            n_set = neighbors[point_index]
        else:
            while depth > 0:
                depth = depth - 1
                for v in neighbors[point_index]:
                    n_set = set.union(n_set, self.get_n_neighbors(neighbors, depth, v, n_set))
        return n_set

    def compute_metrics(self):
        self.compute_mean_curvature()
        self.compute_plane_roughness()
        self.compute_mean_metrics('plane_roughness')
        self.compute_mean_metrics('curvature')
        self.compute_mean_metrics('quad_roughness')
        self.ctime = time.time() - self.ctime

    def compute_mean_curvature(self):
        """
        Computes the best quadrics fitting the surface of each SuperPoint and the associated curvature
        :return:
        """
        points = self.coords
        for i in self.knn:
            current_point = points[i, :]
            neighbors = [points[j, :] for j in list(self.knn[i].keys())]
            if len(neighbors) < 6:
                print("Error")
            neighbors.append(current_point)
            _, _, new_coord, _ = utils.make_pca(np.array(neighbors), n_components=3)
            quadrics, residuals = utils.lstsq_quadrics_fitting(np.array(new_coord))
            curv = self.curvature(quadrics)
            self.set_points_param(i, 'curvature', curv)
            self.set_points_param(i, 'quad_roughness', residuals[-1])

    @staticmethod
    def curvature(quadrics):
        """
        Computes the curvature of a quadrics defined by its 6 parameters
        :param quadrics: an array containing the parameters of the quadric
        :return:
        """
        if np.array(quadrics).any() != -1:
            a = quadrics[0]
            b = quadrics[1]
            c = quadrics[2]
            d = quadrics[3]
            e = quadrics[4]
            curv = ((1 + pow(d, 2)) * a + (1 + pow(e, 2)) * b - 4 * a * b * c) / (pow(1 + pow(e, 2) + pow(d, 2), 3 / 2))
        else:
            print('Error Computing Curvature')
            curv = -1
        return curv

    def compute_plane_roughness(self):
        """
        Computes the local roughness for each point and its mean for each SuperPoint
        :return:
        """
        points = self.coords
        for i in self.knn:
            current_point = points[i, :]
            neighbors = [points[j, :] for j in list(self.knn[i].keys())]
            neighbors.append(current_point)
            plane = utils.lstsq_plane_fitting(np.array(neighbors))
            local_roughness = utils.compute_orth_dist(np.reshape(current_point, (-1, 3)), plane)[0]
            self.set_points_param(i, 'plane_roughness', local_roughness)

    def compute_mean_metrics(self, metric: str):
        metric_val = self.get_points_column(metric)
        for i in self.knn:
            neighbors = list(self.knn[i].keys())
            neighbors.append(i)
            neighbor_metric = [metric_val[i] for i in neighbors]
            self.set_points_param(i, f"mean_{metric}", np.mean(neighbor_metric))
            self.set_points_param(i, f"med_{metric}", np.median(neighbor_metric))

    def save(self, output_path):
        self.save_metric(output_path, metric='plane_roughness')
        self.save_metric(output_path, metric='quad_roughness')
        self.save_metric(output_path, metric='curvature')
        self.save_metric(output_path, metric='mean_plane_roughness')
        self.save_metric(output_path, metric='mean_quad_roughness')
        self.save_metric(output_path, metric='mean_curvature')
        self.save_metric(output_path, metric='med_plane_roughness')
        self.save_metric(output_path, metric='med_quad_roughness')
        self.save_metric(output_path, metric='med_curvature')
        self.save_csv(output_path)
        pass

    def save_metric(self, output_path: str, metric: str, colormap: str = 'jet'):
        """
        Save OBj file with colormap according to the metric
        :param colormap: the matplotlib colormap
        :param output_path: the folder name on where to save data
        :param metric: the metric to save
        :return:
        """
        filename = os.path.join(output_path,
                                f"{metric}_{self.obj_name}.obj")
        if not os.path.exists(os.path.dirname(output_path)):
            try:
                os.makedirs(os.path.dirname(output_path))
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    raise
        with open(filename, 'w+') as f:
            distances = np.abs(self.get_points_column(metric))
            colormap = utils.generate_colormap(distances, colormap)
            for index, row in self._coords.iterrows():
                x = row['x']
                y = row['y']
                z = row['z']
                color = colormap[index]
                f.writelines(f"v {x} {y} {z} {color[0]} {color[1]} {color[2]}\n")

    def save_csv(self, output_path):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        self._points.to_csv(f"{os.path.join(output_path,self.obj_name)}_points.csv")
        self._params.to_csv(f"{os.path.join(output_path,self.obj_name)}_params.csv")
