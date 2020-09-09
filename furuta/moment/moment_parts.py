import moment_primitives as mp
import moment_assemblies as ma
import moment_const as mc

def bracket(radius, offset):
    return ma.bracket_installed(mc.TUBE_SIDE, mc.TUBE_HOLE_DIA/2.0, radius, offset, mc.TUBE_MASS)
    
# reference the near side at rotor 
def hall(radius):
    moment = mp.cuboid_solid_face_centered(mc.HALL_L, mc.HALL_D, mc.HALL_MASS)
    distance = ((radius + mc.HALL_D/2.0)**2 + (mc.HALL_L/2.0 - mc.HALL_AXIS_CENTER)**2)**0.5
    return mp.parallel_axis(moment, distance, mc.HALL_MASS)

# reference near side
def bearing_orthogonal_axis_center(radius):
    moment = mp.cylinder_hollow_orthogonal_axis_center(mc.BEARING_ID, mc.BEARING_OD, mc.BEARING_LENGTH, mc.BEARING_MASS)
    return mp.parallel_axis(moment, radius + mc.BEARING_LENGTH/2.0, mc.BEARING_MASS)



def m3_nut_on_axis(radius):
    moment = ma.hex_nut_on_axis(mc.M3_NUT_OD/2.0, mc.M3_NUT_OD/2.0, mc.M3_NUT_ID, mc.M3_NUT_MASS)
    return mp.parallel_axis(moment, radius, mc.M3_NUT_MASS)

# hole axis parallel to radius
# measured to near face
def m3_nut_orthogonal_axis_end(radius, offset):
    moment = ma.hex_nut_orthogonal_axis_end(mc.M3_NUT_OD/2.0, mc.M3_NUT_OD/2.0, mc.M3_NUT_ID/2.0, mc.M3_NUT_LENGTH, mc.M3_NUT_MASS)
    return mp.parallel_axis(moment, (radius**2 + offset**2)**0.5, mc.M3_NUT_MASS)

def m3_bolt_on_axis(length, mass, radius, offset):
    return ma.bolt_installed_on_axis(mc.M3_BOLT_HEAD_ID/2.0, mc.M3_BOLT_HEAD_ID/2.0, mc.M3_BOLT_HEAD_OD/2.0, mc.M3_BOLT_HEAD_LENGTH, mc.M3_BOLT_PLATE_LENGTH, mc.M3_BOLT_SHAFT_DIA/2.0, length, radius, offset, mass)

def m3_bolt_orthogonal_axis_center(length, mass, radius, offset, head_out):
    return ma.bolt_installed_orthogonal_axis_center(mc.M3_BOLT_HEAD_ID/2.0, mc.M3_BOLT_HEAD_ID/2.0, mc.M3_BOLT_HEAD_OD/2.0, mc.M3_BOLT_HEAD_LENGTH, mc.M3_BOLT_PLATE_LENGTH, mc.M3_BOLT_SHAFT_DIA/2.0, length, radius, offset, mass, head_out)



def m4_nut_on_axis(radius):
    moment = ma.hex_nut_on_axis(mc.M4_NUT_OD/2.0, mc.M4_NUT_OD/2.0, mc.M4_NUT_ID, mc.M4_NUT_MASS)
    return mp.parallel_axis(moment, radius, mc.M4_NUT_MASS)

# hole axis parallel to radius
# measured to near face
def m4_nut_orthogonal_axis_end(radius, offset):
    moment = ma.hex_nut_orthogonal_axis_end(mc.M4_NUT_OD/2.0, mc.M4_NUT_OD/2.0, mc.M4_NUT_ID/2.0, mc.M4_NUT_LENGTH, mc.M4_NUT_MASS)
    return mp.parallel_axis(moment, (radius**2 + offset**2)**0.5, mc.M4_NUT_MASS)

def m4_bolt_on_axis(length, mass, radius, offset):
    return ma.bolt_installed_on_axis(mc.M4_BOLT_HEAD_ID/2.0, mc.M4_BOLT_HEAD_ID/2.0, mc.M4_BOLT_HEAD_OD/2.0, mc.M4_BOLT_HEAD_LENGTH, mc.M4_BOLT_PLATE_LENGTH, mc.M4_BOLT_SHAFT_DIA/2.0, length, radius, offset, mass)

def m4_bolt_orthogonal_axis_center(length, mass, radius, offset, head_out):
    return ma.bolt_installed_orthogonal_axis_center(mc.M4_BOLT_HEAD_ID/2.0, mc.M4_BOLT_HEAD_ID/2.0, mc.M4_BOLT_HEAD_OD/2.0, mc.M4_BOLT_HEAD_LENGTH, mc.M4_BOLT_PLATE_LENGTH, mc.M4_BOLT_SHAFT_DIA/2.0, length, radius, offset, mass, head_out)
