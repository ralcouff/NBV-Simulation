# System modules
from queue import Queue
from threading import Thread

import pymeshlab
import os
from Intrinsic_metrics import IntrinsicMetrics
import argparse

# Set up some global variables
enclosure_queue = Queue()

n_neighbors = [1, 2, 3, 4]


def ComputeMetrics(q):
    while True:
        param = q.get()

        filename = param['filename']
        save_folder = param['output_folder']
        n_neighbors = param['n_neighbors']

        print(f"{n_neighbors} : {filename}")
        model_name = os.path.basename(filename).split('.')[0]
        f_name = f"{n_neighbors}_{model_name}"
        folder_name = os.path.join(model_name, str(f_name))

        # Loading the mesh with pymeshlab
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(filename)
        faces = ms.current_mesh().face_matrix()

        mesh = ms.current_mesh()

        # Computing normals
        # ms.apply_filter('compute_normals_for_point_sets')
        # ms.apply_filter('re_compute_face_normals')
        # ms.apply_filter('re_compute_vertex_normals')
        normals = mesh.vertex_normal_matrix()

        edges = set()
        for f in faces:
            f.sort()
            edges.add((f[0], f[1]))
            edges.add((f[1], f[2]))
            edges.add((f[0], f[2]))

        # Mesh information
        points = mesh.vertex_matrix()
        print(f"# of points : {points.shape}")
        diagonal = mesh.bounding_box().diagonal()
        print(f"Diagonal length : {diagonal}")

        int_met = IntrinsicMetrics(model_name, points, normals, n_neighbors, edges)
        int_met.compute_metrics()

        print(f" Time elapsed: {int_met.ctime}")

        if not os.path.exists(os.path.join(save_folder, 'test', folder_name)):
            os.makedirs(os.path.join(save_folder, 'test', folder_name))

        int_met.save(output_path=os.path.join(save_folder, 'test', folder_name))
        q.task_done()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Compute SuperPointClouds of a PointCloud.')
    parser.add_argument('path_to_file', metavar='filename', type=str,
                        help='Path to the file containing the PointCloud, it can be either .ply or .obj.')
    parser.add_argument('output_folder', metavar='save_folder', type=str, default="results/",
                        help="Path to the output folder where the results files will be saved (default = 'results/'")
    parser.add_argument('--threads', nargs='?', type=int, default=1,
                        help="Number of threads (default = 2)")

    args = parser.parse_args()

    dirname = args.path_to_file
    save_folder = args.output_folder
    num_fetch_threads = args.threads

    # Set up some threads to fetch the enclosures
    for i in range(num_fetch_threads):
        worker = Thread(target=ComputeMetrics, args=(enclosure_queue,))
        worker.setDaemon(True)
        worker.start()

    for root, dirs, files in os.walk(dirname):
        for file in files:
            filename = os.path.join(dirname, file)
            for neighbor in n_neighbors:
                param = {'filename': filename, 'output_folder': save_folder, 'n_neighbors': neighbor}
                enclosure_queue.put(param)

    print('Main thread waiting')
    enclosure_queue.join()
    print('Done')
