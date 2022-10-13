import bpy
import mathutils
import numpy as np
import sys

class MirrorComputation:
    def __init__(self, mirror, normal = mathutils.Vector([0,0,1,0])):
        self.mirror = mirror
        self.t_axis = self.mirror.matrix_world @ normal
        self.t_axis.normalize()

        self.r1_axis = self.t_axis.orthogonal()
        self.r1_axis.normalize()
        self.r2_axis = self.t_axis.cross(self.r1_axis)
        self.r2_axis.normalize()

        reflect_over_xy = mathutils.Matrix([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        self.world_reflect = self.mirror.matrix_world.inverted @ reflect_over_xy @ self.mirror.matrix_world

    def make_delta_distance(self, obj, dist):
        v = obj.matrix_world.inverted() @ self.t_axis
        v.normalize() # all the weirdness with this is to ensure object space units
        v *= dist
        return mathutils.Matrix.Translation(v)

    def make_delta_rotation(self, obj, ang, axis):
        r1 = mathutils.Matrix.Rotation(ang, 4, axis)
        r2 = obj.matrix_world.inverted() @ r1 @ obj.matrix_world
        q = r2.to_quaternion()
        r3 = q.to_matrix().to_4x4()
        return r3

    def make_reflection(self, obj):
        return obj.matrix_world.inverted() @ self.world_reflect @ obj.matrix_world
    
    def make_deltas(self, obj, avg_error, speed):
        radius = max(obj.dimensions)
        dist = np.sqrt(avg_error) * speed
        angle = dist / radius

        return [
            self.make_delta_distance(obj, dist),
            self.make_delta_distance(obj, -dist),
            self.make_delta_rotation(obj, self.r1_axis, angle),
            self.make_delta_rotation(obj, self.r1_axis, -angle),
            self.make_delta_rotation(obj, self.r2_axis, angle),
            self.make_delta_rotation(obj, self.r2_axis, -angle),
        ]

class OBJECT_OT_mirror_fit(bpy.types.Operator):
    """Tweak object trasform to minimize mirroring error."""
    bl_idname = "object.mirror_fit"
    bl_label = "Fit To Mirror"
    bl_options = {'REGISTER', 'UNDO'}
    
    mirror: bpy.props.PointerProperty(type=bpy.types.Object, name="Mirror", description="Object to use as the mirror transformation.")
    max_dist: bpy.props.FloatProperty(name="Max Distance", description="Ignore matches that are further than this value.", default=1.84467e+19, min=sys.float_info.min, subtype="DISTANCE", unit="LENGTH")
    iter_count: bpy.props.IntProperty(name="Iterations", description="Number of iterations to perform when refining the position.", default=20, min=1, soft_max=100, subtype="UNSIGNED")
    samp_count: bpy.props.IntProperty(name="Samples", description="Number of points to sample to compute the error term; 0 to use all points.", default=20, min=0, soft_max=65536, subtype="UNSIGNED")
    samp_seed: bpy.props.IntProperty(name="Random Seed", description="Seed for sampling points.", default=0)
    speed: bpy.props.FloatProperty(name="Step Factor", description="Scaling factor for gradients.", default=1, min=sys.float_info.min, soft_min=1, soft_max=1)

    
    @classmethod
    def poll(cls, context):
        try:
            return isinstance(context.active_object.data.vertices, bpy.types.MeshVertices) and len(context.active_object.data.vertices) > 0
        except AttributeError:
            return False

    def execute(self, context):
        scene = context.scene
        
        obj = context.active_object
        me = obj.data
        radius = max(obj.dimensions)
        verts = np.empty(len(me.vertices)*3, dtype=np.float64)
        me.vertices.foreach_get('co', verts)
        verts.shape = (len(me.vertices), 3) 

        if self.samp_count > 0:
            rng = np.random.default_rng(self.samp_seed)
            sample = rng.choice(verts, self.samp_count)
        else:
            sample = verts

        mirrorizer = MirrorComputation(self.mirror)

        def matrix_total(delta = mathutils.Matrix.Identity(4)):
            dm = obj.matrix_world @ delta
            return dm.inverted() @ mirrorizer.world_reflect @ dm

        avg_error = self.calculate_error(matrix_total() @ sample, obj)
        print("error starting:", avg_error)
        
        for i in range(self.iter_count):
            err_delta = [
                (
                    self.calculate_error(matrix_total(delta=d) @ sample, obj), d
                ) for d in mirrorizer.make_deltas(obj, avg_error, self.speed)
            ]
            
            new_error, delta = min(err_delta)
            if new_error < avg_error:
                avg_error = new_error
                print("error accepted:", avg_error)
                obj.matrix_world = obj.matrix_world @ delta
            else:
                self.speed /= 2
                print("error rejected:", new_error)
                print("reduced speed to:" + str(self.speed))
        
        return {'FINISHED'}
    

        
    def calculate_error(self, points, obj):
        error = 0
        count = 0

        for point in points:
            result, closest_point, normal, index = obj.closest_point_on_mesh(point, distance=self.max_dist)
            if result:
                error_vec = point - closest_point
                error += error_vec.dot(error_vec)
                count += 1
        
        if count == 0:
            return np.inf
        else:
            return error / count


class OBJECT_PT_mirror_fit(bpy.types.Panel):
    bl_idname = "OBJECT_PT_mirror_fit"
    bl_label = "Fit to Mirror"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Mirror"

    def draw(self, context):
        # You can set the property values that should be used when the user
        # presses the button in the UI.
        layout = self.layout
        scene = context.scene

        row = layout.row()
        props = row.operator('object.mirror_fit')
        
        row = layout.row()
        row.prop(props, 'mirror', icon='EMPTY_AXIS')
        row = layout.row()
        row.prop(props, 'max_dist')
        row = layout.row()
        row.prop(props, 'iter_count')
        row = layout.row()
        row.prop(props, 'samp_count')
        row = layout.row()
        row.prop(props, 'samp_seed')
        row = layout.row()
        row.prop(props, 'speed')


def register():
    bpy.utils.register_class(OBJECT_OT_mirror_fit)
    bpy.utils.register_class(OBJECT_PT_mirror_fit)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_mirror_fit)
    bpy.utils.unregister_class(OBJECT_PT_mirror_fit)

if __name__ == "__main__":
    register()