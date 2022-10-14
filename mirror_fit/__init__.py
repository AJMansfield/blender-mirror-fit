from argparse import ArgumentError
import bpy
import mathutils
import numpy as np
import sys

# bl_info = {
#     "name": "Fit to Mirror",
#     "blender": (3,3,0),
#     "category": "Object",
# }

def is_good_obj(obj: bpy.types.Object) -> bool:
    return isinstance(obj.data, bpy.types.Mesh) and len(obj.data.vertices) >= 1

def is_good_mirror(mirror: bpy.types.Object) -> bool:
    return (mirror.type == 'EMPTY') or (isinstance(mirror.data, bpy.types.Mesh) and len(mirror.data.polygons) == 1)

def _get_object_and_mirror(context: bpy.types.Context) -> tuple[bpy.types.Object, bpy.types.Object]:
    """returns (object, mirror)"""
    if not len(context.selected_objects) == 2: raise ArgumentError("Must have two selected objects.")
    a,b = tuple(context.selected_objects)

    # if one of them is an empty, it's definitely the mirror
    if a.type == 'EMPTY':
        return b,a
    elif b.type == 'EMPTY':
        return a,b
    
    # if one of them isn't a mesh, then the one with a mesh is definitly the object
    if not a.type == 'MESH':
        return b,a
    elif not b.type == 'MESH':
        return a,b
    
    assert isinstance(a.data, bpy.types.Mesh)
    assert isinstance(b.data, bpy.types.Mesh)
    
    # if one of them is a mesh with a single face, and the other one isn't, then the single-face mesh is definitely the mirror
    if len(a.data.polygons) == 1 and len(b.data.polygons) != 1:
        return b,a
    elif len(b.data.polygons) == 1 and len(a.data.polygons) != 1:
        return a,b

    # otherwise, the active object is the object and the other one is the mirror
    if a == context.active_object:
        return a,b
    else:
        return b,a

def get_object_and_mirror(context: bpy.types.Context) -> tuple[bpy.types.Object, bpy.types.Object]:
    """picks out the object and the mirror to manipulate from the current selected objects, and validates them"""
    obj, mirror = cls._get_object_and_mirror(context)
    if not is_good_obj(obj):
        raise ArgumentError("Active Object must contain a mesh.")
    if not is_good_mirror(mirror):
        raise ArgumentError("Mirror must be empty or have a mesh with a single polygon.")
    return obj, mirror

def get_mirror_normal(mirror: bpy.types.Object) -> mathutils.Vector:
    """returns the reflection vector of the mirror in its local coordinate space"""
    if mirror.type == 'MESH':
        assert isinstance(mirror.data, bpy.types.Mesh)
        return mirror.data.polygons[0].normal
    elif mirror.type == 'EMPTY':
        if mirror.empty_display_type == 'CIRCLE': # circles display as a disc perpendicular to the y axis!
            return mathutils.Vector([0,1,0])
        elif mirror.empty_display_type == 'SINGLE_ARROW': # single arrows display as an arrow pointing along the z axis
            return mathutils.Vector([0,0,1])
        else: # z axis default
            return mathutils.Vector([0,0,1])

def get_mirror_point(mirror: bpy.types.Object) -> mathutils.Vector:
    """returns the reflection centerpoint of the mirror in its local coordinate space"""
    if mirror.type == 'MESH':
        assert isinstance(mirror.data, bpy.types.Mesh)
        return mirror.data.polygons[0].center
    elif mirror.type == 'EMPTY':
        return mathutils.Vector([0,0,0])

def get_mirror_matrix(mirror: bpy.types.Object) -> mathutils.Matrix:
    """returns the mirroring transformation of the mirror in its local coordinate space"""
    point = get_mirror_point(mirror)
    norm = get_mirror_normal(mirror)
    translation = mathutils.Matrix.Translation(point)
    reflection = mathutils.Matrix.Scale(-1, 4, norm)
    return translation @ reflection @ translation.inverted()

class Mirror:
    def __init__(self, mirror_object: bpy.types.Object):
        self.mirror = mirror_object
        
        self.t_axis = self.mirror.matrix_world @ get_mirror_normal(self.mirror)
        """Translation axis that moves closer or furthre from the reflection."""
        self.t_axis.normalize()

        self.r1_axis = self.t_axis.orthogonal()
        """One rotation axis that rotates perpendicular to the reflection."""
        self.r1_axis.normalize()
        self.r2_axis = self.t_axis.cross(self.r1_axis)
        """Another rotation axis that rotates perpendicular to the reflection."""
        self.r2_axis.normalize()

        self.matrix_reflect = self.mirror.matrix_world @ get_mirror_matrix(self.mirror) @ self.mirror.matrix_world.inverted()
        """Matrix representing this mirror's reflection transformation in world-space coordinates."""

    def make_delta_distance(self, obj, dist):
        v = obj.matrix_world.inverted() @ self.t_axis # transform translation axis into object space
        v.normalize() # normalize and multiply by distance to get it in object-space units
        v *= dist
        return mathutils.Matrix.Translation(v)

    def make_delta_rotation(self, obj, ang, axis):
        r1 = mathutils.Matrix.Rotation(ang, 4, axis) # rotation in world-space coordinates
        r2 = obj.matrix_world.inverted() @ r1 @ obj.matrix_world # transform into object-space coordinates
        r3 = r2.to_quaternion().to_matrix().to_4x4() # drop all but rotational component to rotate about object origin rather than world origin
        return r3

    def make_reflection(self, obj):
        return obj.matrix_world.inverted() @ self.matrix_reflect @ obj.matrix_world
    
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
    
    # mirror: bpy.props.PointerProperty(type=bpy.types.Object, name="Mirror", description="Object to use as the mirror transformation.")
    max_dist: bpy.props.FloatProperty(name="Max Distance", description="Ignore matches that are further than this value.", default=1.84467e+19, min=sys.float_info.min, subtype="DISTANCE", unit="LENGTH")
    iter_count: bpy.props.IntProperty(name="Iterations", description="Number of iterations to perform when refining the position.", default=20, min=1, soft_max=100, subtype="UNSIGNED")
    samp_count: bpy.props.IntProperty(name="Samples", description="Number of points to sample to compute the error term; 0 to use all points.", default=20, min=0, soft_max=65536, subtype="UNSIGNED")
    samp_seed: bpy.props.IntProperty(name="Random Seed", description="Seed for sampling points.", default=0)
    speed: bpy.props.FloatProperty(name="Step Factor", description="Scaling factor for gradients.", default=1, min=sys.float_info.min, soft_min=1, soft_max=1)

    @classmethod
    def poll(cls, context):
        get_object_and_mirror(context) # and trigger any exceptions that this might cause!
        return True
    
    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    def execute(self, context):
        obj, mirror_obj = get_object_and_mirror(context)

        me = obj.data
        assert isinstance(me, bpy.types.Mesh)

        verts = np.empty(len(me.vertices)*3, dtype=np.float64)
        me.vertices.foreach_get('co', verts) # fastest way to get vertices in a numpy array
        verts.shape = (len(me.vertices), 3) 

        if self.samp_count > 0:
            rng = np.random.default_rng(self.samp_seed)
            sample = rng.choice(verts, self.samp_count)
        else:
            sample = verts

        mirror = Mirror(mirror_obj)

        def matrix_total(delta = mathutils.Matrix.Identity(4)):
            dm = obj.matrix_world @ delta
            return dm.inverted() @ mirror.matrix_reflect @ dm

        avg_error = self.calculate_error(matrix_total() @ sample, obj)
        print("error starting:", avg_error)
        
        for i in range(self.iter_count):
            err_delta = [
                (
                    self.calculate_error(matrix_total(delta=d) @ sample, obj), d
                ) for d in mirror.make_deltas(obj, avg_error, self.speed)
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


# class OBJECT_PT_mirror_fit(bpy.types.Panel):
#     bl_idname = "OBJECT_PT_mirror_fit"
#     bl_label = "Fit to Mirror"
#     bl_space_type = 'VIEW_3D'
#     bl_region_type = 'UI'
#     bl_category = "Mirror"

#     def draw(self, context):
#         # You can set the property values that should be used when the user
#         # presses the button in the UI.
#         layout = self.layout
#         scene = context.scene

#         row = layout.row()
#         props = row.operator('object.mirror_fit')
        
#         row = layout.row()
#         row.prop(props, 'mirror', icon='EMPTY_AXIS')
#         row = layout.row()
#         row.prop(props, 'max_dist')
#         row = layout.row()
#         row.prop(props, 'iter_count')
#         row = layout.row()
#         row.prop(props, 'samp_count')
#         row = layout.row()
#         row.prop(props, 'samp_seed')
#         row = layout.row()
#         row.prop(props, 'speed')

# Only needed if you want to add into a dynamic menu.
def menu_func(self, context):
    self.layout.operator(OBJECT_OT_mirror_fit.bl_idname)

def register():
    bpy.utils.register_class(OBJECT_OT_mirror_fit)
    # bpy.utils.register_class(OBJECT_PT_mirror_fit)
    bpy.types.VIEW3D_MT_view.append(menu_func)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_mirror_fit)
    # bpy.utils.unregister_class(OBJECT_PT_mirror_fit)
    bpy.types.VIEW3D_MT_view.remove(menu_func)

if __name__ == "__main__":
    register()