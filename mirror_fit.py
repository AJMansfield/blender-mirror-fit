import bpy
import bmesh
import mathutils
import math
import numpy as np
import random


def flatten(mat):
    dim = len(mat)
    return [mat[j][i] for i in range(dim) 
                      for j in range(dim)]
def unflatten(mat):
    dim = int(math.sqrt(len(mat)))
    return mathutils.Matrix([mat[i*dim:(i+1)*dim] for i in range(dim)])

class MirrorErrorEstimatePanel(bpy.types.Panel):
    bl_label = "Mirror Error Estimator"
    bl_idname = "VIEW3D_PT_mirror_error"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Mirror'
    
    def __init__(self, *a, **kw):
        super(*a, **kw)

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        obj = scene.my_obj
        mirror = scene.my_mirror
        maxdist = scene.my_maxdist

        row = layout.row()
        row.prop(scene, "my_obj")
        row = layout.row()
        row.prop(scene, "my_mirror", icon='EMPTY_AXIS')
        row = layout.row()
        row.operator("object.reset_mirror")
        row = layout.row()
        row.prop(scene, "my_maxdist")
        row = layout.row()
        row.prop(scene, "my_maxiter")
        row = layout.row()
        row.label(text="Result: ?") # + str(compute_error(obj,mirror,maxdist)))
        row = layout.row()
        row.prop(scene, "my_transform")
        row = layout.row()
        row.operator("object.refine_mirror")
        

class SetTransformFromMirrorObjectOperator(bpy.types.Operator):
    bl_idname = "object.reset_mirror"
    bl_label = "Reset Mirror Tuning"

    def execute(self, context):
        scene = context.scene
        
        obj = scene.my_obj
        mirror = scene.my_mirror
        maxdist = scene.my_maxdist
        
        print(scene.my_transform)
        
        return {'FINISHED'}
    
class RefineMirrorOperator(bpy.types.Operator):
    """Tweak object settings to Tooltip"""
    bl_idname = "object.refine_mirror"
    bl_label = "Refine Mirror"

    def execute(self, context):
        scene = context.scene
        
        obj = scene.my_obj
        mirror = scene.my_mirror
        maxdist = scene.my_maxdist
        maxiter = scene.my_maxiter
        
        obj_size = max(obj.dimensions)
        
        step_factor = 2
        
        
        for i in range(maxiter):
            cur_err = compute_error(obj, mirror, maxdist=maxdist)
            print("init error: " + str(cur_err))
            
            dist = math.sqrt(cur_err)*step_factor
            ang = dist/obj_size
            
            deltas = [
#                mathutils.Matrix.Rotation(ang, 4, 'X'),
#                mathutils.Matrix.Rotation(-ang, 4, 'X'),
                mathutils.Matrix.Rotation(ang, 4, 'Y'),
                mathutils.Matrix.Rotation(-ang, 4, 'Y'),
                mathutils.Matrix.Rotation(ang, 4, 'Z'),
                mathutils.Matrix.Rotation(-ang, 4, 'Z'),
                mathutils.Matrix.Translation([dist,0,0]),
                mathutils.Matrix.Translation([-dist,0,0]),
#                mathutils.Matrix.Translation([0,dist,0]),
#                mathutils.Matrix.Translation([0,-dist,0]),
#                mathutils.Matrix.Translation([0,0,dist]),
#                mathutils.Matrix.Translation([0,0,-dist]),
            ]
            
#            print(deltas)
#            print(dist, ang)
#            deltas = [ mirror.matrix_world @ d for d in deltas ]
            
            eps_delta = [
                (compute_error(obj, mirror, d, maxdist=maxdist), d) for d in deltas
            ]
            
            e, d = min(eps_delta)
            if e < cur_err:
                print("improved to: " + str(e))
                obj.matrix_world =  d @ obj.matrix_world
            else:
                step_factor /= 2
                print("worsened: " + str(e))
                print("Trying smaller step: " + str(step_factor))

#        print("Result: " + str(result_delta))
        
#        obj.matrix_world = obj.matrix_world @ result_delta
        
        return {'FINISHED'}

def compute_error(obj, mirror, delta = mathutils.Matrix.Identity(4), maxdist = 1e19, sample_size = 10000, myrand = None):
    if myrand is None: myrand = random.Random(1)
    
    verts = myrand.sample(list(obj.data.vertices), sample_size)
    obj_err = 0
    world_err = 0
    count = 0
    
    world_mat = delta @ obj.matrix_world
    obj_mat = world_mat.inverted() 
#    mirrored_world_mat = mirror.matrix_world @ world_mat
#    mirrored_obj_mat = world_mat.inverted() @ mirrored_world_mat 
    
#    print("mirror.matrix_world", mirror.matrix_world)
#    print("world_mat", world_mat)
#    print("obj_mat", obj_mat)
#    print("mirrored_world_mat", mirrored_world_mat)
#    print("mirrored_obj_mat", mirrored_obj_mat)
    
#    first = True
    
    for v in verts:
        obj_point = v.co
        world_point = world_mat @ obj_point
        mirror_world_point = mirror.matrix_world @ world_point
        mirror_obj_point = obj_mat @ mirror_world_point
        
        result, closest_obj_point, normal, index = obj.closest_point_on_mesh(mirror_obj_point, distance=maxdist)
        
        if result:
#            closest_world_point = world_mat @ closest_obj_point
            
#            if first:
#                print("obj_point", obj_point)
#                print("world_point", world_point)
#                print("mirror_world_point", mirror_world_point)
#                print("mirror_obj_point", mirror_obj_point)
#                print("closest_obj_point", closest_obj_point)
#                print("closest_world_point", closest_world_point)
#                first = False
            
            obj_d = (mirror_obj_point - closest_obj_point)
#            world_d = (mirror_world_point - closest_world_point)
            obj_err += obj_d.dot(obj_d)
#            world_err += world_d.dot(world_d)
            count += 1
        else:
#            print("obj point:", v.co)
#            print("mirrored:", mirrored_obj_mat @ v.co)
            pass
    
    if count == 0:
        return float('inf')
    else:
        return obj_err / count
        

def register():
    bpy.utils.register_class(MirrorErrorEstimatePanel)
    bpy.utils.register_class(RefineMirrorOperator)
    bpy.utils.register_class(SetTransformFromMirrorObjectOperator)
    bpy.types.Scene.my_mirror = bpy.props.PointerProperty(type=bpy.types.Object, name="Mirror")
    bpy.types.Scene.my_obj = bpy.props.PointerProperty(type=bpy.types.Object, name="Object")
    bpy.types.Scene.my_maxdist = bpy.props.FloatProperty(name="dist")
    bpy.types.Scene.my_maxiter = bpy.props.IntProperty(name="maxiter")
    bpy.types.Scene.my_transform = bpy.props.FloatVectorProperty(name="Matrix", size=16, subtype="MATRIX")

def unregister():
    bpy.utils.unregister_class(MirrorErrorEstimatePanel)
    bpy.utils.unregister_class(RefineMirrorOperator)
    bpy.utils.unregister_class(SetTransformFromMirrorObjectOperator)
    del bpy.types.Object.my_maxdist
    del bpy.types.Object.my_maxiter
    del bpy.types.Object.my_mirror
    del bpy.types.Object.my_obj
    del bpy.types.Object.my_transform


if __name__ == "__main__":
    register()