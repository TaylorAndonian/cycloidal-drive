"""
Cycloidal reducer -- 30:1, NEMA 17, Ø80 x ~51 mm envelope.
Companion unit to cycloidal-50-d100 (the big joint).

All dimensions in mm. Edit here; every other script imports from this file.
"""

# ---------------------------------------------------------------- ratio / ring
N_PINS   = 31          # ring pins.  Reduction = N_PINS - 1 with ring fixed.
N_LOBES  = N_PINS - 1  # 30 lobes  ->  30:1
R_PIN_CIRCLE = 31.0    # pin circle radius
R_PIN        = 2.0     # ring pin radius (Ø4 dowel)
ECC          = 0.79    # eccentricity

# curtate / undercut ratio.  Must be > 1.0;  1.15..1.5 is the sane band.
CURTATE = R_PIN_CIRCLE / (ECC * N_PINS)

# profile is offset inward by this much so the printed disc has real
# backlash against the steel pins.  0.06-0.10 works on a Bambu with PA-CF.
DISC_CLEARANCE = 0.08

RING_PIN_D    = 4.0
RING_PIN_LEN  = 30.0    # stock Ø4 x 30 dowel, 1.5 into each end plate
PIN_POCKET_CLR = 0.20

# ---------------------------------------------------------------- housing
HOUSING_OD   = 80.0
SLEEVE_T     = 32.0     # ring sleeve length (disc stack + carrier)
TIE_D        = 3.4      # M3 clearance
TIE_INSERT_D = 4.0      # M3 heat-set insert
TIE_PCD      = 73.0
N_TIE        = 6
TIE_LEN      = 45.0     # M3 x 45 SHCS

# ---------------------------------------------------------------- discs
DISC_T       = 12.5     # each disc
N_DISCS      = 2
DISC_SHIM    = 0.5      # washer between the two discs
DISC_BORE    = 21.0     # 6801 OD (12x21x5), 2 per disc
DISC_SHOULDER_BORE = 17.0
DISC_SHOULDER_T    = 1.5
ECC_BRG      = ("6801", 12.0, 21.0, 5.0)   # name, id, od, w
ECC_BRG_PER_DISC = 2

# ---------------------------------------------------------------- output pins
N_OUT_PINS   = 8
OUT_PIN_D    = 6.0      # Ø6 hardened dowel
OUT_PCD_R    = 19.0     # output pin circle radius
OUT_PIN_LEN  = 32.0
OUT_HOLE_D   = OUT_PIN_D + 2 * ECC

# ---------------------------------------------------------------- shaft
MOTOR_SHAFT_D  = 5.0
SHAFT_BORE_D   = 5.05
CAM_JOURNAL_D  = 12.0   # 6801 bore
NOSE_JOURNAL_D = 8.0    # 608 bore  (you have a pile of 608s)
COLLAR_D       = 15.0
REAR_BRG  = ("6801", 12.0, 21.0, 5.0)
NOSE_BRG  = ("608",   8.0, 22.0, 7.0)
GRUB_D    = 3.0         # M3 grub screw onto the motor shaft flat

# ---------------------------------------------------------------- output brg
OUT_BRG   = ("6808", 40.0, 52.0, 7.0)

# ---------------------------------------------------------------- plates
MOTOR_PLATE_T = 7.0
CARRIER_T     = 6.0
CARRIER_OD    = 60.0
BOSS_D        = 40.0
BOSS_LEN      = 8.5
RING_PLATE_T  = 8.0
FACE_PLATE_T  = 4.0
FACE_PLATE_OD = 70.0
FACE_BOLT_PCD = 50.0
N_FACE_BOLTS  = 6
FACE_BOLT_D   = 3.4     # M3 clearance
BOSS_BOLT_PCD = 30.0    # face plate -> boss
N_BOSS_BOLTS  = 4
BOSS_INSERT_D = 5.6     # M4 heat-set insert

NEMA17_BOLT_PCD = 31.0
NEMA17_PILOT_D  = 22.4
NEMA17_BOLT_D   = 3.4

# ---------------------------------------------------------------- fits
CLR_RUNNING = 0.30      # printed-part running clearance
CLR_SLEEVE  = 1.0       # radial, disc crest -> sleeve bore

# ---------------------------------------------------------------- axial datum
# z = 0 is the motor mounting face.
Z_SLEEVE   = MOTOR_PLATE_T                       # 6.0
Z_DISC_A   = Z_SLEEVE                            # 6.0
Z_DISC_B   = Z_DISC_A + DISC_T + DISC_SHIM       # 19.5
Z_CARRIER  = Z_SLEEVE + SLEEVE_T - CARRIER_T     # 33.0
Z_RINGPL   = Z_SLEEVE + SLEEVE_T                 # 39.0
Z_FACE     = Z_RINGPL + BOSS_LEN                 # 47.5
OVERALL_L  = Z_FACE + FACE_PLATE_T               # 51.5
