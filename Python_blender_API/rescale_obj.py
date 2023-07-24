import argparse
import bpy
import mathutils
import os

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
obj_object = bpy.ops.wm.obj_import(filepath=file_loc,
                                   directory=os.path.dirname(file_loc)+"/",
                                    files=[{"name":os.path.basename(file_loc), "name":os.path.basename(file_loc)}])
# imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
obj_object = bpy.context.selected_objects[0]

material_object_texture = bpy.data.materials.new(name="object_texture")
material_object_texture.use_nodes = True
material_object_texture.node_tree.nodes.remove(material_object_texture.node_tree.nodes.get('Principled BSDF'))
NodeBSDF = material_object_texture.node_tree.nodes.new('ShaderNodeBsdfDiffuse')
NodeBSDF.inputs["Roughness"].default_value = 0.0
Node_Vert = material_object_texture.node_tree.nodes.new('ShaderNodeVertexColor')
Node_Vert.layer_name = "Color"
Node_output = material_object_texture.node_tree.nodes.get('Material Output')
links = material_object_texture.node_tree.links
links.new(Node_Vert.outputs["Color"], NodeBSDF.inputs["Color"])
links.new(NodeBSDF.outputs["BSDF"], Node_output.inputs["Surface"])


ob = bpy.context.active_object
# Get material
mat = bpy.data.materials.get("Material")
if mat is None:
    # create material
    mat = material_object_texture
# Assign it to object
if ob.data.materials:
    # assign to 1st material slot
    ob.data.materials[0] = material_object_texture
else:
    # no slots
    ob.data.materials.append(material_object_texture)

# Rescaling the object
obj_object.data.transform(mathutils.Matrix((
    (scale * unit, 0, 0, 0),
    (0, -scale * unit, 0, 0),
    (0, 0, -scale * unit, 0),
    (0, 0, 0, 1))))
obj_object.data.update()

target_file = args.save

bpy.ops.export_scene.obj(filepath=target_file)
