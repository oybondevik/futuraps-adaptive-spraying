import argparse
import csv
import math


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _load_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append({key: _to_float(value) for key, value in row.items()})
    return rows


def _series(rows, key):
    return [row.get(key, math.nan) for row in rows]


def main():
    parser = argparse.ArgumentParser(
        description="Plot platform PID debug CSV from /tmp/platform_pid_debug.csv"
    )
    parser.add_argument("--input", default="/tmp/platform_pid_debug.csv")
    parser.add_argument("--output", default="/tmp/platform_pid_debug.png")
    parser.add_argument("--reference-distance", type=float, default=0.9)
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="Expected canopy side in platform_link. left=+Y, right=-Y.",
    )
    args = parser.parse_args()

    rows = _load_csv(args.input)
    if not rows:
        raise RuntimeError(f"No rows found in {args.input}")

    import matplotlib.pyplot as plt

    t0 = rows[0].get("t", 0.0)
    t = [row.get("t", math.nan) - t0 for row in rows]
    ref_y = args.reference_distance if args.side == "left" else -args.reference_distance

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t, _series(rows, "y"), label="platform odom y")
    axes[0].set_ylabel("odom y [m]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, _series(rows, "closest_y"), label="detected closest_y")
    axes[1].axhline(ref_y, color="k", linestyle="--", label="reference y")
    axes[1].set_ylabel("canopy y [m]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, _series(rows, "distance"), label="abs closest_y")
    axes[2].axhline(
        args.reference_distance, color="k", linestyle="--", label="reference distance"
    )
    axes[2].plot(t, _series(rows, "distance_error"), label="distance_error")
    axes[2].set_ylabel("distance [m]")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(t, _series(rows, "v_cmd"), label="v_cmd")
    axes[3].plot(t, _series(rows, "w_cmd"), label="w_cmd")
    axes[3].plot(t, _series(rows, "w_odom"), linestyle="--", label="odom angular.z")
    axes[3].set_xlabel("time [s]")
    axes[3].set_ylabel("cmd / odom")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(args.output, dpi=140)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
