import bpy
import math
import mathutils

# Input data
file_loc = "/home/alcoufr/dev/NBV-Simulation/3d_models/rescaled_model.obj"
input_location = (-0.11872799999999993, 0.282775, -0.25120199999999993)
camera_location = (input_location[0], input_location[2], -input_location[1])

# Import object
imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
obj_object = bpy.context.selected_objects[0]

# Set some lights
light_data = bpy.data.lights.new('light', type='SUN')
light = bpy.data.objects.new('light', light_data)
light.location = (3, 4, -5)
light.data.energy = 200.0
bpy.context.collection.objects.link(light)

cam_data = bpy.data.cameras.new('camera')
cam = bpy.data.objects.new('camera', cam_data)
cam.location = camera_location

rot_mat = mathutils.Matrix([[0.93384826779126118, -0.17799940535976502, 0.31023156582596723],
                            [0.34749177460295833, 0.65696119345020232, -0.66906760262605269],
                            [-0.084716464318031509, 0.73261053909431084, 0.67535540175628694]])

mat = mathutils.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, -1]])

corr_rot_mat = (mat @ rot_mat.transposed())

full_mat = mathutils.Matrix([[corr_rot_mat[0][0], corr_rot_mat[0][1], corr_rot_mat[0][2],input_location[0]],
                            [corr_rot_mat[1][0], corr_rot_mat[1][1], corr_rot_mat[1][2],input_location[2]],
                            [corr_rot_mat[2][0], corr_rot_mat[2][1], corr_rot_mat[2][2],-input_location[1]],
                            [0,0,0,1]])

# cam.matrix_world = full_mat
rot_euler = (rot_mat @ mat).to_euler("XYZ")
cam.rotation_euler[0] = rot_euler[0]
cam.rotation_euler[1]= rot_euler[2]
cam.rotation_euler[2] = -rot_euler[1]               
# cam.rotation_euler = rot_euler
print(cam.rotation_euler[0]*180/3.1415,cam.rotation_euler[1]*180/3.1415,cam.rotation_euler[2]*180/3.1415)


bpy.context.collection.objects.link(cam)  # add camera to scenescene = bpy.context.scenescene.camera=cam

# led_distance = 0.1
# led_light = __createLedLight("Led Light")
# LedFront = __createLightObj("LedFront", led_light, (0, -led_distance, 0), (90, 0, 0))
# LedBack = __createLightObj("LedBack", led_light, (0, led_distance, 0), (-90, 0, 0))
# LedLeft = __createLightObj("LedLeft", led_light, (0, led_distance, 0), (0, 90, 0))
# LedRight = __createLightObj("LedRight", led_light, (0, -led_distance, 0), (0, -90, 0))
# LedTop = __createLightObj("LedTop", led_light, (0, led_distance, 0), (0, 0, 0))
# LedBottom = __createLightObj("LedBottom", led_light, (0, -led_distance, 0), (180, 0, 0))

view_layer = bpy.context.view_layer
