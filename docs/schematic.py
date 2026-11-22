"""Circuit schematic for Dose Coach (drawn by hand with matplotlib)."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, Circle, FancyBboxPatch

INK, MUTED, RED, YEL = "#111111", "#4b5563", "#d64545", "#e0b000"
plt.rcParams.update({"font.family": "DejaVu Sans"})
fig = plt.figure(figsize=(16, 8.4), dpi=170)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 16); ax.set_ylim(0, 8.4); ax.axis("off")
LW = 2.2

def wire(*pts):
    xs, ys = zip(*pts); ax.plot(xs, ys, color=INK, lw=LW, solid_capstyle="round", zorder=2)

def label(x, y, t, size=13, color=INK, **kw):
    ax.text(x, y, t, fontsize=size, color=color, zorder=5, **kw)

def ground(x, y):
    wire((x, y), (x, y - 0.25))
    for i, w in enumerate((0.36, 0.24, 0.12)):
        yy = y - 0.25 - i * 0.09
        ax.plot([x - w / 2, x + w / 2], [yy, yy], color=INK, lw=LW, zorder=2)

def plus5(x, y, t="+5 V"):
    wire((x, y), (x, y + 0.22))
    ax.plot([x - 0.15, x + 0.15], [y + 0.22, y + 0.22], color=INK, lw=LW)
    label(x, y + 0.3, t, 11, ha="center", va="bottom")

def resistor(x1, x2, y, text="220 Ω"):
    n = 6; seg = (x2 - x1) / (n + 2)
    pts = [(x1, y), (x1 + seg, y)]
    for i in range(n):
        pts.append((x1 + seg * (i + 1.5), y + (0.14 if i % 2 == 0 else -0.14)))
    pts += [(x2 - seg, y), (x2, y)]
    wire(*pts)
    label((x1 + x2) / 2, y + 0.22, text, 11, ha="center", va="bottom")

def led(x, y, color):
    w = 0.42
    ax.add_patch(Polygon([(x, y - 0.22), (x, y + 0.22), (x + w, y)], closed=True, fc=color, ec=INK, lw=LW, zorder=3))
    ax.plot([x + w, x + w], [y - 0.22, y + 0.22], color=INK, lw=LW, zorder=3)
    for dx in (0.08, 0.24):  # light arrows
        ax.annotate("", xy=(x + dx + 0.28, y + 0.5), xytext=(x + dx + 0.08, y + 0.26),
                    arrowprops=dict(arrowstyle="->", color=INK, lw=1.4))
    return x + w

def button(x, y):
    # two contacts and a lifted bar
    for cx in (x, x + 0.6):
        ax.add_patch(Circle((cx, y), 0.05, fc="white", ec=INK, lw=1.8, zorder=3))
    ax.plot([x, x + 0.6], [y + 0.22, y + 0.22], color=INK, lw=LW, zorder=3)
    ax.plot([x + 0.3, x + 0.3], [y + 0.22, y + 0.4], color=INK, lw=LW, zorder=3)
    ax.plot([x + 0.18, x + 0.42], [y + 0.4, y + 0.4], color=INK, lw=LW, zorder=3)
    return x + 0.6

def box(x, y, w, h, title, fc="#f2f3f4", tsize=14, sub=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.08", fc=fc, ec=INK, lw=LW, zorder=1))
    label(x + w / 2, y + h - 0.18, title, tsize, ha="center", va="top", weight="bold")
    if sub: label(x + w / 2, y + h - 0.52, sub, 11, MUTED, ha="center", va="top")

# ---------------- Arduino ----------------
AX1, AX2, AY1, AY2 = 5.6, 8.6, 0.7, 7.9
box(AX1, AY1 - 0.1, AX2 - AX1, AY2 - AY1 + 0.1, "Arduino UNO R4 Minima", fc="#ffffff", tsize=15, sub="+ Grove Base Shield, 5 V")
left_pins = {"5V": 7.0, "GND": 6.45, "A0": 5.7, "A1": 5.15, "A2": 4.6, "A4 / SDA": 3.2, "A5 / SCL": 2.65}
right_pins = {"D2": 7.15, "D3": 6.3, "D4": 5.45, "D5": 4.4, "D7": 3.5, "D6": 2.5, "D8": 1.45}
for n, y in left_pins.items():
    label(AX1 + 0.12, y, n, 12, va="center", weight="bold")
for n, y in right_pins.items():
    label(AX2 - 0.12, y, n, 12, va="center", ha="right", weight="bold")
label((AX1 + AX2) / 2, 0.75, "D2-D4 use the built-in\npull-up resistors (INPUT_PULLUP)", 10.5, MUTED, ha="center", va="bottom")

# ---------------- Accelerometer (left, top) ----------------
box(0.4, 4.15, 2.9, 3.55, "Wrist accelerometer", sub="MMA7361 module")
acc = [("VIN", 7.0), ("GND", 6.45), ("X", 5.7), ("Y", 5.15), ("Z", 4.6)]
for n, y in acc:
    label(3.3 - 0.12, y, n, 12, va="center", ha="right", weight="bold")
wire((3.3, 7.0), (AX1, 7.0)); wire((3.3, 6.45), (AX1, 6.45))
for (_, y) in acc[2:]:
    wire((3.3, y), (AX1, y))
label(0.55, 4.3, "SL (sleep) jumpered to the\nmodule's own 3.3 V pin", 10.5, MUTED, va="bottom")
label(4.45, 7.08, "+5 V", 10.5, MUTED, ha="center", va="bottom")
label(4.45, 6.53, "GND", 10.5, MUTED, ha="center", va="bottom")
label(4.45, 5.78, "X", 10.5, MUTED, ha="center", va="bottom")
label(4.45, 5.23, "Y", 10.5, MUTED, ha="center", va="bottom")
label(4.45, 4.68, "Z", 10.5, MUTED, ha="center", va="bottom")

# ---------------- LCD (left, bottom) ----------------
box(0.4, 1.55, 2.9, 2.2, "RGB LCD 16x2", sub="Grove, I2C")
for n, y in (("SDA", 3.2), ("SCL", 2.65)):
    label(3.3 - 0.12, y, n, 12, va="center", ha="right", weight="bold")
    wire((3.3, y), (AX1, y))
label(0.55, 1.68, "5 V and GND through\nthe Grove cable", 10.5, MUTED, va="bottom")
label(4.45, 3.28, "I2C data", 10.5, MUTED, ha="center", va="bottom")
label(4.45, 2.73, "I2C clock", 10.5, MUTED, ha="center", va="bottom")

# ---------------- Buttons ----------------
for name, pin in (("A", "D2"), ("B", "D3"), ("C", "D4")):
    y = right_pins[pin]
    wire((AX2, y), (10.0, y))
    end = button(10.0, y)
    wire((end, y), (11.2, y))
    ground(11.2, y)
    label(11.6, y, f"Button {name}", 12, va="center", weight="bold")
    label(12.95, y, {"A": "start / count / pause", "B": "next exercise", "C": "restart / mute"}[name], 11, MUTED, va="center")

# ---------------- LEDs ----------------
for pin, col, name in (("D5", RED, "Red warning LED"), ("D7", YEL, "Yellow status LED")):
    y = right_pins[pin]
    wire((AX2, y), (9.6, y))
    end = led(9.6, y, col)
    wire((end, y), (10.35, y))
    resistor(10.35, 11.55, y)
    wire((11.55, y), (11.9, y))
    ground(11.9, y)
    label(12.25, y, name, 12, va="center", weight="bold")

# ---------------- Grove modules ----------------
for pin, name, sub in (("D6", "TAP pad", "Grove touch sensor"), ("D8", "Buzzer", "Grove buzzer")):
    y = right_pins[pin]
    wire((AX2, y), (10.0, y))
    ax.add_patch(FancyBboxPatch((10.0, y - 0.42), 3.1, 0.84, boxstyle="round,pad=0,rounding_size=0.06", fc="#f2f3f4", ec=INK, lw=LW, zorder=1))
    label(10.12, y, "SIG", 11, va="center", weight="bold")
    label(11.75, y + 0.12, name, 13, ha="center", va="center", weight="bold")
    label(11.75, y - 0.2, sub, 10.5, MUTED, ha="center", va="center")
    label(13.25, y, "5 V and GND\nthrough the\nGrove cable", 10, MUTED, va="center")

label(15.6, 0.25, "All grounds are common. Laptop USB supplies 5 V.", 11, MUTED, ha="right", va="bottom")
fig.savefig(sys.argv[1], facecolor="white")
print("ok")
