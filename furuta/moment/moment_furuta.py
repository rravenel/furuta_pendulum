import moment_assemblies as ma
import moment_parts as mpa
import moment_primitives as mpr
import moment_const as mc

def axle_on_axis():
    return mpr.cylinder_solid_on_axis(mc.ROD_DIA/2.0, mc.AXLE_MASS)

def axle_orthogonal_axis_center():
    return mpr.cylinder_solid_orthogonal_axis_center(mc.ROD_DIA/2.0, mc.AXLE_LENGTH, mc.AXLE_MASS)


def axle_hardware_on_axis():
    return 5.0 * mpa.m4_nut_on_axis(0)    

def axle_hardware_orthogonal_axis_center():
    nuts = 5.0 * mpa.m4_nut_orthogonal_axis_end(mc.AXLE_LENGTH/2.0, 0)
    bearings = 2.0 * mpa.bearing_orthogonal_axis_center(mc.AXLE_LENGTH/2.0)
    return nuts + bearings

def axle_mass():
    mass = mc.AXLE_MASS
    mass += 2.0 * mc.BEARING_MASS
    mass += 5.0 * mc.M4_NUT_MASS
    return mass

def pendulum_on_axis():
    return ma.square_tube_on_axis(mc.TUBE_SIDE, mc.TUBE_WALL, mc.POLE_MASS)

def pendulum_orthogonal_axis_end():
    return ma.square_tube_orthogonal_axis_end(mc.TUBE_SIDE, mc.TUBE_WALL, mc.POLE_LENGTH, mc.POLE_MASS)

def pendulum_mass():
    mass = mc.POLE_MASS
    return mass

def rotor_orthogonal_axis_center():
    tube = ma.square_tube_orthogonal_axis_center(mc.TUBE_SIDE, mc.TUBE_WALL, mc.ARM_LENGTH, mc.ARM_MASS)

    hall = mpa.hall(mc.LINK_LENGTH/2.0)

    hall_mount = ma.square_tube_orthogonal_axis_center(mc.TUBE_SIDE, mc.TUBE_WALL, mc.HALL_MOUNT_L, mc.HALL_MOUNT_MASS)
    hall_mount = mpr.parallel_axis(hall_mount, mc.ARM_LENGTH/2.0 + mc.TUBE_SIDE, mc.HALL_MOUNT_MASS)
    h_nut = 2.0 * mpa.m4_nut_orthogonal_axis_end(mc.ARM_LENGTH/2.0 + mc.TUBE_SIDE, mc.HALL_MOUNT_L/2.0)
    h_bolt = 2.0 * mpa.m4_bolt_orthogonal_axis_center(mc.M4_BOLT_SHAFT_L_30, mc.M4_BOLT_MASS_30, mc.ARM_LENGTH/2.0 + mc.TUBE_SIDE, mc.HALL_MOUNT_L/2.0, False)

    brackets = 2.0 * mpa.bracket(mc.ARM_LENGTH/2.0, mc.TUBE_SIDE/2.0)
    # in to side of rotor arm
    b_nut_1 = 2.0 * mpa.m3_nut_orthogonal_axis_end(mc.TUBE_SIDE/2.0 + mc.TUBE_WALL, mc.ARM_LENGTH/2.0 - mc.TUBE_SIDE/2.0)
    b_bolt_1 = 2.0 * mpa.m3_bolt_orthogonal_axis_center(mc.M3_BOLT_SHAFT_L_6, mc.M3_BOLT_MASS_6, mc.TUBE_SIDE/2.0 - mc.TUBE_WALL, mc.ARM_LENGTH/2.0 - mc.TUBE_SIDE/2.0, False)
    # in to hall mount
    b_nut_2 = 2.0 * mpa.m3_nut_orthogonal_axis_end(mc.ARM_LENGTH/2.0 + mc.TUBE_WALL, mc.TUBE_SIDE)
    b_bolt_2 = 2.0 * mpa.m3_bolt_orthogonal_axis_center(mc.M3_BOLT_SHAFT_L_6, mc.M3_BOLT_MASS_6, mc.ARM_LENGTH/2.0 - mc.TUBE_WALL, mc.TUBE_SIDE, False)

    # mount to threaded hole in rotor - no nut
    m_bolt = 2.0 * mpa.m3_bolt_on_axis(mc.M3_BOLT_SHAFT_L_6, mc.M3_BOLT_MASS_6, mpr.mm2m(12), 0)
    
    return hall_mount + h_nut + h_bolt + brackets + b_nut_1 + b_nut_2 + b_bolt_1 + b_bolt_2 + m_bolt

def rotor_mass():
    mass = mc.ARM_MASS
    mass += mc.HALL_MASS
    mass += mc.HALL_MOUNT_MASS
    mass += 2.0 * mc.M4_NUT_MASS
    mass += 2.0 * mc.M4_BOLT_SHAFT_L_30
    mass += 2.0 * mc.BRACKET_MASS
    mass += 4.0 * mc.M3_NUT_MASS
    mass += 4.0 * mc.M3_BOLT_MASS_6
    return mass

print("axle_on_axis: ", ( axle_on_axis() + axle_hardware_on_axis() ) )
print("axle_orthogonal_axis_center: ", ( axle_orthogonal_axis_center() + axle_hardware_orthogonal_axis_center() ) )
print("axle_mass: ", (axle_mass()))
print("pendulum_on_axis: ", ( pendulum_on_axis() ) )
print("pendulum_orthogonal_axis_end: ", ( pendulum_orthogonal_axis_end() ) )
print("pendulum_mass: ", (pendulum_mass()))
print("rotor_orthogonal_axis_center: ", ( rotor_orthogonal_axis_center() ) )
print("rotor_mass: ", (rotor_mass()))
