import bpy
import os
import numpy as np
import mathutils
import argparse

# Parsing command line arguments
argParser = argparse.ArgumentParser()
argParser.add_argument("-f_abc", "--file_abc", help="The abc file")
argParser.add_argument("-f_obj", "--file_obj", help="The rescaled obj model")
argParser.add_argument("-t", "--type", help="Generate all images or last")
argParser.add_argument("-s", "--save", help="Save path for the images")

args = argParser.parse_args()


filename = args.file_abc
file_loc = args.file_obj
save_type = int(args.type)
# unit = args.unit
# scale = args.scale


# Clean the scene
context = bpy.context
scene = context.scene
for c in scene.collection.children:
    scene.collection.children.unlink(c)

# Import the Alembic file
# filename = "/home/alcoufr/dev/NBV-Simulation/cmake-build-debug-cmake-kitware/plop.abc"
bpy.ops.wm.alembic_import(filepath=filename)

# Loading the obj file
# file_loc = "/home/alcoufr/dev/NBV-Simulation/3d_models/Armadillo.obj"
imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
obj_object = bpy.context.selected_objects[0]

# # Rescaling the object
# obj_object.data.transform(mathutils.Matrix((
#     (scale * unit, 0, 0, 0),
#     (0, -scale * unit, 0, 0),
#     (0, 0, -scale * unit, 0),
#     (0, 0, 0, 1))))
# obj_object.data.update()

light_data = bpy.data.lights.new('light', type='SUN')
light = bpy.data.objects.new('light', light_data)
bpy.context.collection.objects.link(light)
light.rotation_euler = [-np.pi / 2, 0.0, (45 * np.pi) / 180]
light.data.energy = 10
light.data.angle = (130 * np.pi) / 180

cameras = bpy.data.cameras
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
scene = bpy.context.scene
scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.cycles.device = 'GPU'
bpy.data.worlds["World"].node_tree.nodes["Background"].inputs[0].default_value = (0.01, 0.01, 0.01, 1)

if save_type == 0: # LAST
    cam_max = -1
    for cam in cameras:
        try:
            cam_num = int(cam.name.split("__")[-1])
            if cam_num > cam_max:
                print(cam_num)
                cam_max = cam_num
                my_cam = cam
        except ValueError:
            print(cam.name)
            print("Wrong Camera")
    print("Camera parameters:")
    print(f"Camera Name: {my_cam.name}")
    print(f"Focal Length: {my_cam.lens}")
    print(f"Sensor Height: {my_cam.sensor_height}")
    print(f"Sensor Width: {my_cam.sensor_width}")

    scene.camera = bpy.data.objects.get(f"{'_'.join(my_cam.name.split('_')[1:])}")
    scene.render.filepath = args.save
    bpy.ops.render.render(write_still=1)

elif save_type == 1: #ALL
    cameras = bpy.data.cameras
    for i, cam in enumerate(cameras):
        print(i)
        if i != 0:
            print(f"{'_'.join(cam.name.split('_')[1:])}")
            scene.camera = bpy.data.objects.get(f"{'_'.join(cam.name.split('_')[1:])}")
            scene.render.filepath = f"{i}_{args.save}"
            bpy.ops.render.render(write_still=1)
