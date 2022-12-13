import argparse
import bpy
import mathutils

# Parsing command line arguments
argParser = argparse.ArgumentParser()
argParser.add_argument("-f_obj", "--file_obj", help="The original obj model")
argParser.add_argument("-u", "--unit", type=float, help="Unit to rescale the model", default=1.0)
argParser.add_argument("-sc", "--scale", type=float, help="Coefficient to scale the model", default=1.0)
argParser.add_argument("-s", "--save", help="Save path for the images")

args = argParser.parse_args()

file_loc = args.file_obj
unit = args.unit
scale = args.scale

# Clean the scene
context = bpy.context
scene = context.scene
for c in scene.collection.children:
    scene.collection.children.unlink(c)

# Loading the obj file
imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
obj_object = bpy.context.selected_objects[0]

# Rescaling the object
obj_object.data.transform(mathutils.Matrix((
    (scale * unit, 0, 0, 0),
    (0, -scale * unit, 0, 0),
    (0, 0, -scale * unit, 0),
    (0, 0, 0, 1))))
obj_object.data.update()

target_file = args.save

bpy.ops.export_scene.obj(filepath=target_file)
