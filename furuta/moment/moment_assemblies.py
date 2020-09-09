import math
import moment_primitives as mp


# mounting bracket consists of two square faces, joined on a common edge,
#  perpendicular to eachother, each with a round hole, of the same size,
#  centered in the face

# hole is centered; rotates about axis parallel to face, through center of face
def _plate_with_hole(length, radius, mass):
    length = float(length)
    radius = float(radius)
    mass = float(mass)
    
    density = (length**2 - math.pi*radius**2) / mass
    mass_plate_full = length**2 * density
    mass_hole = math.pi*radius**2 * density
    cube_moment = mp.cuboid_solid_face_centered(length, 0, mass_plate_full)
    hole_moment = mp.cylinder_solid_orthogonal_axis_center(radius, 0, mass_hole)
    return cube_moment - hole_moment

# moment installed bracket bracket; mount_radius to nearest square point of bracket; mount_offset from mount_radius
def bracket_installed(face_length, hole_radius, mount_radius, mount_offset, mass):
    face_moment = _plate_with_hole(face_length, hole_radius, mass/2.0)
    r1 = (mount_radius**2 + (mount_offset + face_length/2.0)**2)**0.5
    r2 = ((mount_radius + face_length/2.0)**2 + mount_offset**2)**0.5
    m1 = mp.parallel_axis(face_moment, r1, mass/2.0)
    m2 = mp.parallel_axis(face_moment, r2, mass/2.0)
    return m1 + m2


# approximation using average radius of hexagon
def bolt_head_on_axis(min_radius, max_radius, head_radius, mass):
    inner_radius = (min_radius + max_radius) / 2.0
    return mp.cylinder_hollow_on_axis(inner_radius, head_radius, mass)
    
# approximation using average radius of hexagon
def bolt_head_orthogonal_axis_center(min_radius, max_radius, head_radius, height, mass):
    inner_radius = (min_radius + max_radius) / 2.0
    return mp.cylinder_hollow_orthogonal_axis_center(inner_radius, head_radius, height, mass)

def bolt_head_volume(min_radius, max_radius, head_radius, head_height):
    inner_radius = (min_radius + max_radius) / 2.0
    return (math.pi*head_radius**2 - math.pi*inner_radius**2) * head_height

def bolt_density(min_radius, max_radius, head_radius, head_height, plate_height, shaft_radius, shaft_length, mass):
    head_volume = bolt_head_volume(min_radius, max_radius, head_radius, head_height) #head_area * head_height
    plate_volume = math.pi*head_radius**2 * plate_height
    shaft_volume = math.pi*shaft_radius**2 * shaft_length
    total_volume = head_volume + plate_volume + shaft_volume
    return mass / total_volume

def bolt_installed_on_axis(min_radius, max_radius, head_radius, head_height, plate_height, shaft_radius, shaft_length, mount_radius, mount_offset, mass):
    density = bolt_density(min_radius, max_radius, head_radius, head_height, plate_height, shaft_radius, shaft_length, mass)
    
    head_volume = bolt_head_volume(min_radius, max_radius, head_radius, head_height)
    head_mass = head_volume * density
    head_moment = bolt_head_on_axis(min_radius, max_radius, head_radius, head_mass)

    plate_volume = math.pi*head_radius**2 * plate_height
    plate_mass = plate_volume * density
    plate_moment = mp.cylinder_solid_on_axis(head_radius, plate_mass)
    
    shaft_volume = math.pi*shaft_radius**2 * shaft_length
    shaft_mass = shaft_volume * density
    shaft_moment = mp.cylinder_solid_on_axis(shaft_radius, shaft_mass)
    
    total_moment = head_moment + plate_moment + shaft_moment

    distance = (mount_radius**2 + mount_offset**2)**0.5

    return mp.parallel_axis(total_moment, distance, mass)

# bolt axis is parallel to mount_radius
# mount_radius measured from shaft end at the head (bottom side of plate)
def bolt_installed_orthogonal_axis_center(min_radius, max_radius, head_radius, head_height, plate_height, shaft_radius, shaft_length, mount_radius, mount_offset, mass, head_out):
    density = bolt_density(min_radius, max_radius, head_radius, head_height, plate_height, shaft_radius, shaft_length, mass)
    
    head_volume = bolt_head_volume(min_radius, max_radius, head_radius, head_height)
    head_mass = head_volume * density
    head_moment = bolt_head_orthogonal_axis_center(min_radius, max_radius, head_radius, head_height, head_mass)
    head_distance = ((mount_radius + plate_height + head_height/2.0)**2 + mount_offset**2)**0.5
    if not head_out:
        head_distance = ((mount_radius - plate_height - head_height/2.0)**2 + mount_offset**2)**0.5
    head_moment = mp.parallel_axis(head_moment, head_distance, head_mass)
    
    plate_volume = math.pi*head_radius**2 * plate_height
    plate_mass = plate_volume * density
    plate_moment = mp.cylinder_solid_orthogonal_axis_center(head_radius, plate_height, plate_mass)
    plate_distance = ((mount_radius + plate_height/2.0)**2 + mount_offset**2)**0.5
    if not head_out:
        plate_distance = ((mount_radius - plate_height/2.0)**2 + mount_offset**2)**0.5
    plate_moment = mp.parallel_axis(plate_moment, plate_distance, plate_mass)
    
    shaft_volume = math.pi*shaft_radius**2 * shaft_length
    shaft_mass = shaft_volume * density
    shaft_moment = mp.cylinder_solid_orthogonal_axis_center(shaft_radius, shaft_length, shaft_mass)
    shaft_distance = ((mount_radius - shaft_length/2.0)**2 + mount_offset**2)**0.5
    if not head_out:
        shaft_distance = ((mount_radius + shaft_length/2.0)**2 + mount_offset**2)**0.5
    shaft_moment = mp.parallel_axis(shaft_moment, shaft_distance, shaft_mass)
    
    return head_moment + plate_moment + shaft_moment
    
# approximation using average radius of hexagon
def hex_nut_on_axis(min_radius, max_radius, hole_radius, mass):
    outer_radius = (min_radius + max_radius) / 2.0
    return mp.cylinder_hollow_on_axis(hole_radius, outer_radius, mass)

# approximation using average radius of hexagon
def hex_nut_orthogonal_axis_center(min_radius, max_radius, hole_radius, height, mass):
    outer_radius = (min_radius + max_radius) / 2.0
    return mp.cylinder_hollow_orthogonal_axis_center(hole_radius, outer_radius, height, mass)

# approximation using average radius of hexagon
def hex_nut_orthogonal_axis_end(min_radius, max_radius, hole_radius, height, mass):
    outer_radius = (min_radius + max_radius) / 2.0
    return mp.cylinder_hollow_orthogonal_axis_end(hole_radius, outer_radius, height, mass)

def square_tube_on_axis(side, thickness, mass):
    radius = (side - thickness) / 2.0
    
    side_1 = side
    side_2 = side - 2.0 * thickness
    
    mass_1 = (mass / 2.0) * (side_1/side_2)
    mass_2 = (mass / 2.0) * (side_2/side_1)
    
    moment_1 = mp.cuboid_solid_face_centered(side_1, thickness, mass_1)
    moment_2 = mp.cuboid_solid_face_centered(side_2, thickness, mass_2)
    
    moment_1 = mp.parallel_axis(moment_1, radius, mass_1)
    moment_2 = mp.parallel_axis(moment_2, radius, mass_2)
    
    return 2.0 * (moment_1 + moment_2)

def square_tube_orthogonal_axis_center(side, thickness, length, mass):
    radius = (side - thickness) / 2.0
    
    side_1 = side
    side_2 = side - 2.0 * thickness
    
    mass_1 = (mass / 2.0) * (side_1/side_2)
    mass_2 = (mass / 2.0) * (side_2/side_1)
    
    moment_1 = mp.cuboid_solid_face_centered(length, side_1, mass_1)
    moment_2 = mp.cuboid_solid_face_centered(length, thickness, mass_2)
    
    moment_2 = mp.parallel_axis(moment_2, radius, mass_2)
    
    return 2 * (moment_1 + moment_2)

def square_tube_orthogonal_axis_end(side, thickness, length, mass):
    radius = (side - thickness) / 2.0
    
    side_1 = side
    side_2 = side - 2.0 * thickness
    
    mass_1 = (mass / 2.0) * (side_1/side_2)
    mass_2 = (mass / 2.0) * (side_2/side_1)
    
    moment_1 = mp.cuboid_solid_face_major(length, side_1, mass_1)
    moment_2 = mp.cuboid_solid_face_major(length, thickness, mass_2)
        
    moment_2 = mp.parallel_axis(moment_2, radius, mass_2)
    
    return 2.0 * (moment_1 + moment_2)