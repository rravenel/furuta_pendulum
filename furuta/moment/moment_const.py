import moment_primitives as mp

LINK_LENGTH = mp.mm2m(200.0)
ARM_LENGTH = mp.mm2m(90)
AXLE_LENGTH = mp.mm2m(150)
POLE_LENGTH = mp.mm2m(125)


TUBE_SIDE = mp.mm2m(12.0)
TUBE_WALL = mp.mm2m(1.0)
TUBE_HOLE_DIA = mp.mm2m(3.0)
TUBE_DENSITY_KG_M = mp.g2kg(0.12) / mp.mm2m(1.0)
TUBE_MASS = TUBE_DENSITY_KG_M * LINK_LENGTH
ARM_MASS = TUBE_DENSITY_KG_M * ARM_LENGTH
POLE_MASS = TUBE_DENSITY_KG_M * POLE_LENGTH

ROD_DIA = mp.mm2m(3.8)
ROD_DENSITY_KG_M = mp.g2kg(0.076) / mp.mm2m(1.0)
ROD_MASS = ROD_DENSITY_KG_M * LINK_LENGTH
AXLE_MASS = ROD_DENSITY_KG_M * AXLE_LENGTH

BEARING_OD = mp.mm2m(10.0)
BEARING_ID = mp.mm2m(4.0)
BEARING_LENGTH = mp.mm2m(4.0)
BEARING_MASS = mp.g2kg(1.4)

HALL_L = mp.mm2m(43.0)
HALL_W = mp.mm2m(29.0)
HALL_D = mp.mm2m(9.0)
HALL_MOUNT_L = mp.mm2m(50)
# distance from end, on largest face
HALL_AXIS_CENTER = mp.mm2m(15.0)
HALL_MASS = mp.g2kg(22.0)
HALL_MOUNT_MASS = TUBE_DENSITY_KG_M * HALL_MOUNT_L

BRACKET_SIDE = mp.mm2m(12.0)
BRACKET_DIA = mp.mm2m(3.0)
BRACKET_MASS = mp.g2kg(1.2)

M4_NUT_OD = mp.mm2m(7.3)
M4_NUT_ID = mp.mm2m(3.5)
M4_NUT_LENGTH = mp.mm2m(3.0)
M4_NUT_MASS = mp.g2kg(0.6)

M4_BOLT_HEAD_OD = mp.mm2m(6.9)
M4_BOLT_HEAD_ID = mp.mm2m(3.2)
M4_BOLT_HEAD_LENGTH = mp.mm2m(2.5)
M4_BOLT_PLATE_LENGTH = mp.mm2m(1.5)
M4_BOLT_SHAFT_DIA = mp.mm2m(3.9)
M4_BOLT_MASS_30 = mp.g2kg(3.1)
M4_BOLT_MASS_35 = mp.g2kg(3.5)
M4_BOLT_MASS_40 = mp.g2kg(3.9)
M4_BOLT_MASS_45 = mp.g2kg(4.2)
M4_BOLT_MASS_50 = mp.g2kg(4.6)
M4_BOLT_SHAFT_L_30 = mp.mm2m(30.0)
M4_BOLT_SHAFT_L_35 = mp.mm2m(35.0)
M4_BOLT_SHAFT_L_40 = mp.mm2m(40.0)
M4_BOLT_SHAFT_L_45 = mp.mm2m(45.0)
M4_BOLT_SHAFT_L_50 = mp.mm2m(50.0)


M3_NUT_OD = mp.mm2m(6.34)
M3_NUT_ID = mp.mm2m(2.6)
M3_NUT_LENGTH = mp.mm2m(2.3)
M3_NUT_MASS = mp.g2kg(0.3)

M3_BOLT_HEAD_OD = mp.mm2m(5.4)
M3_BOLT_HEAD_ID = mp.mm2m(2.6)
M3_BOLT_HEAD_LENGTH = mp.mm2m(1.9)
M3_BOLT_PLATE_LENGTH = mp.mm2m(1.0)
M3_BOLT_SHAFT_DIA = mp.mm2m(2.9)
M3_BOLT_MASS_6 = mp.g2kg(0.6)
M3_BOLT_MASS_8 = mp.g2kg(0.7)
M3_BOLT_MASS_10 = mp.g2kg(0.8)
M3_BOLT_MASS_12 = mp.g2kg(0.9)
M3_BOLT_MASS_16 = mp.g2kg(1.1)
M3_BOLT_MASS_20 = mp.g2kg(1.2)
M3_BOLT_SHAFT_L_6 = mp.mm2m(6.0)
M3_BOLT_SHAFT_L_8 = mp.mm2m(8.0)
M3_BOLT_SHAFT_L_10 = mp.mm2m(10.0)
M3_BOLT_SHAFT_L_12 = mp.mm2m(12.0)
M3_BOLT_SHAFT_L_16 = mp.mm2m(16.0)
M3_BOLT_SHAFT_L_20 = mp.mm2m(20.0)


