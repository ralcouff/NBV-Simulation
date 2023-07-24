import bpy
import numpy as np
import argparse
import math
import os

def createLight(name, type, radius):
    light = bpy.data.lights.new(name, type)
    light.shadow_soft_size = radius * 0.25

    return light


def createLightObj(name, light, loc=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0)):
    radiansRot = tuple([math.radians(a) for a in rot])  # Convert angles to radians

    obj = bpy.data.objects.new(name, light)  # Set object settings
    obj.location = loc
    obj.rotation_euler = radiansRot
    obj.data.energy = 15

    bpy.context.collection.objects.link(obj)

    return obj


def createLedLight(name):
    return createLight(name, 'AREA', 3)


def createLedLights(ledDistance, ledAngle):
    # ---------- Create Led light ----------#
    ledLight = createLedLight("LedLight")
    LedFront = createLightObj("LedFront", ledLight, (0, -ledDistance, 0), (90, 0, 0))
    LedBack = createLightObj("LedBack", ledLight, (0, ledDistance, 0), (-90, 0, 0))
    LedLeft = createLightObj("LedLeft", ledLight, (ledDistance, 0, 0), (0, 90, 0))
    LedRight = createLightObj("LedRight", ledLight, (-ledDistance, 0, 0), (0, -90, 0))
    LedTop = createLightObj("LedTop", ledLight, (0, 0, ledDistance), (0, 0, 0))
    LedBottom = createLightObj("LedBottom", ledLight, (0, 0, -ledDistance), (180, 0, 0))

    # ---------- Rotate Led light by 37.5 degrees ----------#
    ledLights = bpy.data.objects.new('LedLights', None)  # None for empty object
    ledLights.location = (0, 0, 0)
    ledLights.empty_display_type = 'PLAIN_AXES'

    # Relinking
    LedFront.parent = LedBack.parent = LedLeft.parent = LedRight.parent = LedTop.parent = LedBottom.parent = ledLights
    ledLights.rotation_euler[2] = math.radians(ledAngle)

    bpy.context.collection.objects.link(ledLights)


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
ledDistance = 2.0

# Clean the scene
context = bpy.context
scene = context.scene
for c in scene.collection.children:
    scene.collection.children.unlink(c)

# Import the Alembic file
bpy.ops.wm.alembic_import(filepath=filename)

# Loading the obj file
obj_object = bpy.ops.wm.obj_import(filepath=file_loc,
                                   directory=os.path.dirname(file_loc)+"/",
                                    files=[{"name":os.path.basename(file_loc), "name":os.path.basename(file_loc)}])
obj_object = bpy.context.selected_objects[0]

createLedLights(0.5, 37.5)
# light_data = bpy.data.lights.new('light', type='SUN')
# light = bpy.data.objects.new('light', light_data)
# bpy.context.collection.objects.link(light)
# light.rotation_euler = [-np.pi / 2, 0.0, (45 * np.pi) / 180]
# light.data.energy = 10
# light.data.angle = (130 * np.pi) / 180

cameras = bpy.data.cameras
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
scene = bpy.context.scene
scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.cycles.device = 'GPU'
bpy.data.worlds["World"].node_tree.nodes["Background"].inputs[0].default_value = (0.01, 0.01, 0.01, 1)

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

if save_type == 0:  # LAST
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

elif save_type == 1:  # ALL
    cameras = bpy.data.cameras
    for i, cam in enumerate(cameras):
        print(i)
        if i != 0:
            print(f"{'_'.join(cam.name.split('_')[1:])}")
            scene.camera = bpy.data.objects.get(f"{'_'.join(cam.name.split('_')[1:])}")
            scene.render.filepath = f"{i}_{args.save}"
            bpy.ops.render.render(write_still=1)
