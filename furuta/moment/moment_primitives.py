'''
Functions for computing moments of inertia.

z - up (height)
x - right (width)
y - towards the viewer (depth)

'''


def g2kg(arg):
    return arg / 1000.0

def mm2m(arg):
    return arg / 1000.0

def cylinder_solid_on_axis(radius, mass):
    radius = float(radius)
    mass = float(mass)
    return mass/2.0 * radius**2
    
def test_cylinder_solid_on_axis():
    return 4 == cylinder_solid_on_axis(2, 2)
    
def cylinder_solid_orthogonal_axis_center(radius, length, mass):
    radius = float(radius)
    length = float(length)
    mass = float(mass)
    return mass * (radius**2 / 4.0 + length**2 / 12.0)

def test_cylinder_solid_orthogonal_axis_center():
    return 2 * (1 + 1/3.0) == cylinder_solid_orthogonal_axis_center(2, 2, 2)
    
def cylinder_solid_orthogonal_axis_end(radius, length, mass):
    radius = float(radius)
    length = float(length)
    mass = float(mass)
    return mass * (radius**2 / 4.0 + length**2 / 3.0)

def test_cylinder_solid_orthogonal_axis_end():
    return 2 * (1 + 4/3.0) == cylinder_solid_orthogonal_axis_end(2, 2, 2)
    
def cylinder_hollow_on_axis(radius_inner, radius_outer, mass):
    radius_inner = float(radius_inner)
    radius_outer = float(radius_outer)
    mass = float(mass)
    return mass/2.0 * (radius_inner**2 + radius_outer**2)

def test_cylinder_hollow_on_axis():
    return 20 == cylinder_hollow_on_axis(2, 4, 2)

def cylinder_hollow_orthogonal_axis_center(radius_inner, radius_outer, length, mass):
    radius_inner = float(radius_inner)
    radius_outer = float(radius_outer)
    length = float(length)
    mass = float(mass)
    return mass * (radius_inner**2 / 4.0 + radius_outer**2 / 4.0 + length**2 / 12.0)

def test_cylinder_hollow_orthogonal_axis_center():
    return 2 * (5 + 1/3.0) == cylinder_hollow_orthogonal_axis_center(2, 4, 2, 2)

def cylinder_hollow_orthogonal_axis_end(radius_inner, radius_outer, length, mass):
    radius_inner = float(radius_inner)
    radius_outer = float(radius_outer)
    length = float(length)
    mass = float(mass)
    return mass * (radius_inner**2 / 4.0 + radius_outer**2 / 4.0 + length**2 / 3.0)

def test_cylinder_hollow_orthogonal_axis_end():
    return 2 * (5 + 4/3.0) == cylinder_hollow_orthogonal_axis_end(2, 4, 2, 2)

def cuboid_solid_face_centered(length_major, length_minor, mass):
    length_major = float(length_major)
    length_minor = float(length_minor)
    mass = float(mass)
    return mass/12.0 * (length_major**2 + length_minor**2)

def test_cuboid_solid_face_centered():
    return 8 == cuboid_solid_face_centered(2, 2, 12)

# rotate about center point of length_minor; length_major is the rotating radius
def cuboid_solid_face_major(length_major, length_minor, mass):
    length_major = float(length_major)
    length_minor = float(length_minor)
    mass = float(mass)
    return mass * (length_major**2 / 3.0 + length_minor**2 / 12.0)

def test_cuboid_solid_face_major():
    return 2 * (4/3.0 + 16/12.0) == cuboid_solid_face_major(2, 4, 2)

def parallel_axis(moment, distance, mass):
    moment = float(moment)
    distance = float(distance)
    mass = float(mass)
    return moment + mass * distance**2

def test_parallel_axis():
    return 10 == parallel_axis(2, 2, 2)

def test():
    assert test_cylinder_solid_on_axis()
    assert test_cylinder_solid_orthogonal_axis_center()
    assert test_cylinder_solid_orthogonal_axis_end()
    assert test_cylinder_hollow_on_axis()
    assert test_cylinder_hollow_orthogonal_axis_center()
    assert test_cylinder_hollow_orthogonal_axis_end()
    assert test_cuboid_solid_face_centered()
    assert test_cuboid_solid_face_major()
    assert test_parallel_axis()
    print("moment_primitives.test(): Pass")
