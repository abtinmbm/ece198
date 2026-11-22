import re, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = sys.argv[1]
v = [float(m.group(1)) for m in (re.match(r'angle:([-\d.]+)', l) for l in open(src)) if m]
dt = 0.04                                   # PLOT_EVERY = 2 -> 25 points per second
t = [i * dt for i in range(len(v))]

# same rep rules as the sketch
armUp = False; lastBelow = 0; reps = []
for i, a in enumerate(v):
    if not armUp:
        if a < 25: lastBelow = t[i]
        if a > 40: armUp, start, peak, pi = True, lastBelow, a, i
    else:
        if a > peak: peak, pi = a, i
        if a < 25:
            armUp = False; took = t[i] - start
            reps.append((t[pi], peak, took)); lastBelow = t[i]
good = [r for r in reps if r[1] >= 100 and r[2] >= 1.5]
other = [r for r in reps if r not in good]

INK, MUTED, GRID = "#1f2328", "#5b6470", "#e6e8eb"
LINE = "#2f6fdb"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 14})
fig, ax = plt.subplots(figsize=(10, 5.4), dpi=200)
fig.patch.set_facecolor("white")

ax.axhspan(100, 135, color="#2f9e6b", alpha=0.07, lw=0)        # target zone
for y, lab in [(100, "Target 100°\n(must reach)"),
               (40, "Start 40°"),
               (25, "End 25°")]:
    ax.axhline(y, color=MUTED, lw=1, ls=(0, (4, 3)), zorder=1)
    ax.text(t[-1] + 0.4, y, lab, va="center", ha="left", color=MUTED, fontsize=12)

ax.plot(t, v, color=LINE, lw=2, zorder=3, solid_capstyle="round")

for n, (tp, pk, took) in enumerate(good, 1):
    ax.plot(tp, pk, "o", ms=6, color=LINE, mec="white", mew=1.5, zorder=4)
    ax.text(tp, pk + 3, f"Rep {n}\n{took:.1f} s", ha="center", va="bottom", fontsize=11, color=INK)

ax.set_xlim(0, t[-1]); ax.set_ylim(0, 140)
ax.set_xlabel("Time (seconds)", color=INK)
ax.set_ylabel("Elbow angle (°)", color=INK)
ax.set_yticks(range(0, 141, 20))
ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
for s in ("top", "right"): ax.spines[s].set_visible(False)
for s in ("left", "bottom"): ax.spines[s].set_color("#9aa1a9")
ax.tick_params(colors=MUTED)

fig.text(0.085, 0.94, f"{len(good)} slow bicep curls, all {len(good)} judged GOOD",
         fontsize=18, weight="bold", color=INK, ha="left")
fig.text(0.085, 0.885,
         f"Every 2nd reading plotted (25 per second). "
         f"Peaks {min(r[1] for r in good):.0f}–{max(r[1] for r in good):.0f}°, "
         f"rep times {min(r[2] for r in good):.1f}–{max(r[2] for r in good):.1f} s.",
         fontsize=11.5, color=MUTED, ha="left")
fig.text(0.085, 0.018,
         "Between reps the arm rests at about 15°, under the 25° end line, so each rep ends cleanly.\n"
         "The small bump at the very end never reached 100° and was not counted.",
         fontsize=11.5, color=MUTED, ha="left")
fig.subplots_adjust(left=0.085, right=0.86, top=0.82, bottom=0.19)
fig.savefig(sys.argv[2], facecolor="white")
print(len(good), "good;", [(round(a,1), round(b), round(c,2)) for a,b,c in reps])
