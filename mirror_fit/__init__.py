import bpy
import bmesh
import mathutils
import math
import numpy as np
import random


def is_mirror(obj):
    return np.isclose(obj.matrix_world.det(), -1)

def has_mesh_vertices(obj):
    try:
        return isinstance(obj.data.vertices, bpy.types.MeshVertices) and len(obj.data.vertices) > 0
    except AttributeError:
        return False

class MirrorFitSettings(bpy.types.PropertyGroup):
    mirror = bpy.props.PointerProperty(type=bpy.types.Object, name="Mirror", description="Object to use as the mirror transformation.", poll=is_mirror)
    obj = bpy.props.PointerProperty(type=bpy.types.Object, name="Object", description="Object to tune the position of.", poll=has_mesh_vertices)
    max_dist = bpy.props.FloatProperty(name="Max Distance", description="Ignore matches that are further than this value; 0 to ignore.", default=0, min=0, subtype="DISTANCE")
    iter_count = bpy.props.IntProperty(name="Iterations", description="Number of iterations to perform when refining the position.", default=20, min=1, soft_max=100, subtype="UNSIGNED")
    samp_count = bpy.props.IntProperty(name="Samples", description="Number of points to sample to compute the error term; 0 to use all points.", default=20, min=0, soft_max=65536, subtype="UNSIGNED")



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

        sampled_verts = np.empty(count*3, dtype=np.float64)
        
        
        for i in range(maxiter):
            cur_err = compute_error(obj, mirror, maxdist=maxdist)
            print("init error: " + str(cur_err))
            
            dist = math.sqrt(cur_err)*step_factor
            ang = dist/obj_size
            
            deltas = [
                mathutils.Matrix.Rotation(ang, 4, 'Y'),
                mathutils.Matrix.Rotation(-ang, 4, 'Y'),
                mathutils.Matrix.Rotation(ang, 4, 'Z'),
                mathutils.Matrix.Rotation(-ang, 4, 'Z'),
                mathutils.Matrix.Translation([dist,0,0]),
                mathutils.Matrix.Translation([-dist,0,0]),
            ]
            
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
        
        return {'FINISHED'}

def compute_error(obj, mirror, delta = mathutils.Matrix.Identity(4), maxdist = 1e19, sample_size = 10000, myrand = None):
    if myrand is None: myrand = random.Random(1)
    
    verts = myrand.sample(list(obj.data.vertices), sample_size)
    obj_err = 0
    world_err = 0
    count = 0
    
    world_mat = delta @ obj.matrix_world
    obj_mat = world_mat.inverted() 
    
    for v in verts:
        obj_point = v.co
        world_point = world_mat @ obj_point
        mirror_world_point = mirror.matrix_world @ world_point
        mirror_obj_point = obj_mat @ mirror_world_point
        
        result, closest_obj_point, normal, index = obj.closest_point_on_mesh(mirror_obj_point, distance=maxdist)
        
        if result:
            obj_d = (mirror_obj_point - closest_obj_point)
            obj_err += obj_d.dot(obj_d)
            count += 1
    
    if count == 0:
        return float('inf')
    else:
        return obj_err / count
        

def register():
    bpy.utils.register_class(MirrorErrorEstimatePanel)
    bpy.utils.register_class(RefineMirrorOperator)
    bpy.utils.register_class(SetTransformFromMirrorObjectOperator)
    bpy.types.Scene.mirror_mirror = bpy.props.PointerProperty(type=bpy.types.Object, name="Mirror")
    bpy.types.Scene.mirror_obj = bpy.props.PointerProperty(type=bpy.types.Object, name="Object")
    bpy.types.Scene.mirror_maxdist = bpy.props.FloatProperty(name="dist")
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