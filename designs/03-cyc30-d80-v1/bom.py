"""Builds BOM-d80.csv and BOM-d80.xlsx from one table."""
import csv
import os

from openpyxl import Workbook
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

from params import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

HEAD = ["category", "item", "spec", "qty", "have/buy", "cm3", "link_or_note"]

ROWS = [
    ("printed", "coupon", "same material + profile as the real parts; "
     "stl/coupon.stl", 1, "print", 12.5,
     "PRINT FIRST - carries every press fit and clearance in the box"),
    ("printed", "motor_plate", "PA-CF or PC; stl/motor_plate.stl", 1, "print",
     31.4, "NEMA 17 face, pilot recess, rear 6801 seat, 31 pin pockets"),
    ("printed", "sleeve", "PA-CF or PC; stl/sleeve.stl", 1, "print", 55.3,
     "ring body; all 31 pins bear against its bore"),
    ("printed", "disc_a", "PA-CF only; stl/disc_a.stl", 1, "print", 24.2,
     "NOT interchangeable with disc_b - print both, do not print 2 of one"),
    ("printed", "disc_b", "PA-CF only; stl/disc_b.stl", 1, "print", 24.2,
     "lobes phased 6 deg (half a lobe pitch) because its cam is 180 deg round"),
    ("printed", "carrier", "PA-CF; stl/carrier.stl", 1, "print", 21.0,
     "output plate + boss; 8 dowel press fits, 608 seat, 4 x M4 inserts"),
    ("printed", "ring_plate", "PA-CF or PC; stl/ring_plate.stl", 1, "print",
     22.5, "captures the 31 pin tops and holds the 6808"),
    ("printed", "face_plate", "PA-CF or PC; stl/face_plate.stl", 1, "print",
     14.8, "arm interface: 6 x M3 clearance on PCD 50"),
    ("printed", "shaft", "PA-CF; stl/shaft.stl", 1, "print", 3.4,
     "print VERTICAL, 100% infill; or turn from Ø15 steel if you can"),

    ("bearing", "6801-2RS (12x21x5)",
     "2 per disc on the cams + 1 rear support in the motor plate", 5, "buy",
     "", "https://www.amazon.com/uxcell-6801-2RS-Bearing-12x21x5mm-Bearings/"
     "dp/B07TJDVQ3D  (10-pack ~$10, gives spares)"),
    ("bearing", "6808-2RS (40x52x7)",
     "output bearing in the ring plate - carries the arm load", 1, "buy", "",
     "https://bearingsdirect.com/6808-2rs-ball-bearing-40x52x7-sealed-6808ddu/"
     "  or https://www.amazon.com/6808-2RS-61808RS-Bearing-40x52x7-Bearings/"
     "dp/B0DPZC1TFP"),
    ("bearing", "608ZZ (8x22x7)", "shaft nose journal, sits in the carrier boss",
     1, "HAVE", "", "you have a pile of these"),

    ("pin", "dowel Ø4 x 30, DIN 6325 m6, hardened + ground", "ring pins",
     "31 (+4 spare)", "buy", "",
     "https://www.aspenfasteners.com/m4-x-30mm-din-6325-metric-solid-dowel-"
     "pins-steel/  -- do NOT substitute soft/stainless: 31 soft pins will "
     "brinell and give back the backlash you designed out"),
    ("pin", "dowel Ø6 x 32, hardened", "output pins", "8 (+1 spare)", "buy", "",
     "https://www.amazon.com/Dowel-Through-Hardened-Finish-Quantity/"
     "dp/B07FMKL5NK  (100-pack)"),

    ("fastener", "M3 x 45 SHCS", "housing tie screws, ring plate -> motor plate",
     6, "buy", "", "40 mm of stack + 5 mm of insert engagement"),
    ("fastener", "M3 heat-set insert, 4.0 OD x 5.7", "motor plate, 6 places",
     "6 (+2 spare)", "buy", "", "hole is Ø4.0 x 5.0 deep"),
    ("fastener", "M4 heat-set insert, 5.6 OD x 8.0", "carrier boss, 4 places",
     "4 (+1 spare)", "buy", "", "hole is Ø5.6 x 8.0 deep"),
    ("fastener", "M4 x 10 SHCS", "face plate to carrier boss", 4, "buy", "", ""),
    ("fastener", "M3 x 8 SHCS", "motor to motor plate", 4, "buy", "",
     "check it does not bottom in your NEMA 17; M3 x 6 if it does"),
    ("fastener", "M3 x 4 grub screw", "shaft to motor shaft flat", 1, "buy", "",
     "tighten BEFORE the sleeve goes on - the collar is buried afterwards"),
    ("fastener", "M3 SHCS for the arm", "length = arm root thickness + 6 mm", 6,
     "buy", "", "6 holes on PCD 50 in the face plate"),

    ("hardware", "shim washer Ø30 x 0.5", "between disc A and disc B", 1, "buy",
     "", "any 0.5 mm shim that clears Ø21 and stays under Ø59; or print one"),
    ("consumable", "PTFE or lithium grease",
     "ring pins, output pin holes, disc thrust faces", 1, "buy", "",
     "the output pin holes are a sliding contact - do not run them dry"),
]

SPECS = [
    ("ratio", f"{N_LOBES}:1", f"{N_PINS} ring pins, {N_LOBES} lobes"),
    ("envelope", f"Ø{HOUSING_OD:.0f} x {OVERALL_L:.1f} mm",
     "motor mounting face to arm face"),
    ("motor", "NEMA 17", f"Ø{MOTOR_SHAFT_D:.0f} shaft, D-flat"),
    ("pin circle radius", f"{R_PIN_CIRCLE:.1f} mm", ""),
    ("ring pin", f"Ø{RING_PIN_D:.0f} x {RING_PIN_LEN:.0f}",
     f"pitch {2*3.14159265*R_PIN_CIRCLE/N_PINS:.2f} mm"),
    ("eccentricity", f"{ECC:.2f} mm", f"curtate ratio {CURTATE:.3f}"),
    ("designed backlash", f"{DISC_CLEARANCE:.2f} mm",
     "profile offset; raise to 0.10 if tight, drop to 0.06 if loose"),
    ("disc", f"Ø{2*(R_PIN_CIRCLE-R_PIN+ECC)-2*DISC_CLEARANCE:.2f} "
     f"x {DISC_T:.1f} mm", f"{N_DISCS} off, {180/N_LOBES:.0f} deg apart"),
    ("output pins", f"{N_OUT_PINS} x Ø{OUT_PIN_D:.0f} on PCD "
     f"Ø{2*OUT_PCD_R:.0f}", f"holes Ø{OUT_HOLE_D:.2f} (pin + 2e)"),
    ("rated torque", "~8.7 Nm", "lobe contact limited"),
    ("motor limit", "~10.1 Nm", "0.45 Nm holding x 30 x 0.75 efficiency"),
    ("arm interface", f"{N_FACE_BOLTS} x M3 on PCD "
     f"Ø{FACE_BOLT_PCD:.0f}", "same pattern as the Ø100 unit"),
]

INK = "1D1D1F"
HEAD_FILL = PatternFill("solid", fgColor="1F2937")
ALT_FILL = PatternFill("solid", fgColor="F4F4F5")
HAVE_FILL = PatternFill("solid", fgColor="D9F99D")
WARN_FILL = PatternFill("solid", fgColor="FEF3C7")
THIN = Side(style="thin", color="D4D4D8")


def write_csv(path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([h for h in HEAD if h != "cm3"])
        for r in ROWS:
            w.writerow([r[0], r[1], r[2], r[3], r[4], r[6]])


def write_xlsx(path):
    wb = Workbook()
    ws = wb.active
    ws.title = "BOM"

    ws["A1"] = f"cycloidal-30-d80  —  {N_LOBES}:1 NEMA 17 reducer, " \
               f"Ø{HOUSING_OD:.0f} x {OVERALL_L:.1f} mm"
    ws["A1"].font = Font(name="Arial", size=13, bold=True, color=INK)
    ws.merge_cells("A1:G1")
    ws["A2"] = "Printed parts: PA-CF preferred, 0.4 nozzle, 0.15 mm layers, " \
               "5 walls, 40% gyroid. Discs and shaft at 100% infill."
    ws["A2"].font = Font(name="Arial", size=9, italic=True, color="52525B")
    ws.merge_cells("A2:G2")

    hr = 4
    for c, h in enumerate(HEAD, 1):
        cell = ws.cell(hr, c, h)
        cell.font = Font(name="Arial", size=10, bold=True, color="FFFFFF")
        cell.fill = HEAD_FILL
        cell.alignment = Alignment(horizontal="left", vertical="center")

    for i, r in enumerate(ROWS):
        row = hr + 1 + i
        for c, v in enumerate(r, 1):
            cell = ws.cell(row, c, v)
            cell.font = Font(name="Arial", size=10, color=INK)
            cell.alignment = Alignment(vertical="top", wrap_text=(c in (3, 7)))
            cell.border = Border(bottom=THIN)
            if i % 2:
                cell.fill = ALT_FILL
            if c == 4:
                cell.alignment = Alignment(horizontal="center", vertical="top")
        if r[4] == "HAVE":
            ws.cell(row, 5).fill = HAVE_FILL
            ws.cell(row, 5).font = Font(name="Arial", size=10, bold=True,
                                        color="3F6212")
        if "PRINT FIRST" in r[6]:
            for c in range(1, 8):
                ws.cell(row, c).fill = WARN_FILL
                ws.cell(row, c).font = Font(name="Arial", size=10, bold=True,
                                            color="92400E")

    last = hr + len(ROWS)
    tot = last + 2
    ws.cell(tot, 2, "total printed volume").font = Font(
        name="Arial", size=10, bold=True, color=INK)
    v = ws.cell(tot, 6, f"=SUM(F{hr+1}:F{last})")
    v.font = Font(name="Arial", size=10, bold=True, color=INK)
    v.number_format = "0.0"
    ws.cell(tot, 7, "cm3 of filament for one complete gearbox "
                    "(~230 g in PA-CF)").font = Font(
        name="Arial", size=9, italic=True, color="52525B")

    for col, w in zip("ABCDEFG", (11, 34, 42, 13, 10, 8, 62)):
        ws.column_dimensions[col].width = w
    ws.freeze_panes = "A5"

    # ---- specs sheet
    sp = wb.create_sheet("Specs")
    sp["A1"] = "Design summary"
    sp["A1"].font = Font(name="Arial", size=13, bold=True, color=INK)
    for c, h in enumerate(("parameter", "value", "note"), 1):
        cell = sp.cell(3, c, h)
        cell.font = Font(name="Arial", size=10, bold=True, color="FFFFFF")
        cell.fill = HEAD_FILL
    for i, r in enumerate(SPECS):
        for c, val in enumerate(r, 1):
            cell = sp.cell(4 + i, c, val)
            cell.font = Font(name="Arial", size=10, color=INK,
                             bold=(c == 2))
            cell.border = Border(bottom=THIN)
            if i % 2:
                cell.fill = ALT_FILL
    for col, w in zip("ABC", (24, 26, 52)):
        sp.column_dimensions[col].width = w

    wb.save(path)


if __name__ == "__main__":
    write_csv(os.path.join(OUT, "BOM-d80.csv"))
    write_xlsx(os.path.join(OUT, "BOM-d80.xlsx"))
    print("wrote BOM-d80.csv and BOM-d80.xlsx")
