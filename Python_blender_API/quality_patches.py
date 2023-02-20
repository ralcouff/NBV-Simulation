from obj_parser import parse_file, Model
import random
import os


def make_quality_patch(save_folder: str, model: Model, n_ppv: list[int]):
    quality_model = model.copy()
    quality_model.faces = model.faces
    quality_model.vertices = model.vertices
    initial_vertex = random.randint(0, len(quality_model.vertices))
    if not os.path.exists(save_folder):
        os.mkdir(save_folder)
    for n_ppv_i in n_ppv:
        indices = compute_n_neighbors(quality_model, initial_vertex, n_ppv_i)
        save_filename = os.path.join(save_folder, f"{quality_model.name}_quality_colored_{n_ppv_i}.obj")
        quality_model.save_quality(save_filename, indices)


def compute_neighbors(faces, vertices):
    print("Computing the neighbors")
    neighbors = {i: set() for i in range(len(vertices))}
    for f in faces:
        neighbors[f.a].add(f.b)
        neighbors[f.a].add(f.c)
        neighbors[f.b].add(f.a)
        neighbors[f.b].add(f.c)
        neighbors[f.c].add(f.a)
        neighbors[f.c].add(f.b)
    return neighbors


def n_ring_neighbors(neighbors, init_vertex, n_ring):
    voisins = neighbors[init_vertex]
    for j in range(1, n_ring):
        vois = voisins.copy()
        for i in vois:
            voisins.update(neighbors[i])
    return voisins


def compute_n_neighbors(model: Model, init_vertex: int, n_neighbor):
    neighbors = compute_neighbors(model.faces, model.vertices)
    voisins = set()
    n_ring = 1
    while len(voisins) < n_neighbor:
        voisins = n_ring_neighbors(neighbors, init_vertex, n_ring)
        n_ring += 1
    return voisins


if __name__ == "__main__":
    print("Hello World")
    n_test = 5
    save_folder = 'Arma_qlt'
    reference_file = "armadillo.obj"
    reference_model = parse_file(reference_file)
    percentage = [0.01, 0.025, 0.05, 0.1]
    toto = [int(x * len(reference_model.vertices)) for x in percentage]
    for i in range(n_test):
        print()
        make_quality_patch(f"{i}_{save_folder}", reference_model, toto)
