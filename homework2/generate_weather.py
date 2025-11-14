import argparse
import sys
import numpy as np
from collections import Counter
import os
# NEW: lazy import in main if plotting requested

STATES = ["sunny", "cloudy", "rainy"]
IDX = {s: i for i, s in enumerate(STATES)}
LETTER_TO_STATE = {"s": "sunny", "c": "cloudy", "r": "rainy"}

# Transition matrix P[row = today, col = tomorrow]
P = np.array([
    [0.8, 0.2, 0.0],  # today = sunny
    [0.4, 0.4, 0.2],  # today = cloudy
    [0.2, 0.6, 0.2],  # today = rainy
], dtype=float)

def sequence_probability(start_state: str, seq_states: list[str]) -> float:
    """Exact probability of a specific sequence given the start state."""
    cur = IDX[start_state]
    prob = 1.0
    for s in seq_states:
        nxt = IDX[s]
        prob *= P[cur, nxt]
        cur = nxt
    return prob

def simulate_weather(initial_state: str, length: int, rng: np.random.Generator) -> list[str]:
    """Simulate a single weather sequence (includes the initial day)."""
    seq = [initial_state]
    cur = IDX[initial_state]
    for _ in range(length):
        cur = rng.choice(3, p=P[cur])
        seq.append(STATES[cur])
    return seq

def estimate_sequence_probability_mc(start_state: str, target_seq: list[str], trials: int = 200_000, seed: int = 0) -> float:
    """Monte Carlo estimate of the probability of target_seq following start_state."""
    rng = np.random.default_rng(seed)
    match = 0
    for _ in range(trials):
        cur = IDX[start_state]
        ok = True
        for s in target_seq:
            cur = rng.choice(3, p=P[cur])
            if STATES[cur] != s:
                ok = False
                break
        if ok:
            match += 1
    return match / trials

def estimate_stationary_distribution(steps: int = 2_000_000, burn_in: int = 10_000, seed: int = 0) -> dict[str, float]:
    """Estimate the stationary distribution by long-run simulation."""
    rng = np.random.default_rng(seed)
    cur = IDX["sunny"]
    counts = np.zeros(3, dtype=np.int64)
    for t in range(steps + burn_in):
        cur = rng.choice(3, p=P[cur])
        if t >= burn_in:
            counts[cur] += 1
    freqs = counts / counts.sum()
    return {STATES[i]: float(freqs[i]) for i in range(3)}

def parse_one_state(token: str) -> str:
    """Parse a single start token: 's'/'sunny', 'c'/'cloudy', 'r'/'rainy'."""
    t = token.strip().lower()
    if t in STATES:
        return t
    if t in LETTER_TO_STATE:
        return LETTER_TO_STATE[t]
    raise ValueError(f"Invalid state token: {token!r}. Use s/c/r or sunny/cloudy/rainy.")

def parse_seq(tokens: list[str]) -> list[str]:
    """Parse sequence tokens: accepts 'scr', 's c r', 'sunny cloudy rainy', or mixed."""
    states: list[str] = []
    if not tokens:
        raise ValueError("Empty sequence.")
    for tok in tokens:
        t = tok.strip().lower().replace(",", "")
        if not t:
            continue
        # If token is a bundle of letters like 'scr'
        if all(ch in LETTER_TO_STATE for ch in t):
            states.extend(LETTER_TO_STATE[ch] for ch in t)
        elif t in STATES:
            states.append(t)
        else:
            raise ValueError(f"Invalid sequence token: {tok!r}. Use s/c/r or sunny/cloudy/rainy.")
    if not states:
        raise ValueError("Parsed empty sequence.")
    return states

def interactive() -> int:
    print("Enter start day (s/c/r or sunny/cloudy/rainy): ", end="", flush=True)
    start_in = sys.stdin.readline().strip()
    try:
        start_state = parse_one_state(start_in)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2

    print("Enter sequence (e.g., 'ccr' or 'cloudy cloudy rainy'):", end=" ", flush=True)
    seq_in = sys.stdin.readline().strip()
    try:
        # Split on whitespace; parser also accepts compact letters like 'ccr'
        seq_states = parse_seq(seq_in.split())
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2

    prob = sequence_probability(start_state, seq_states)
    seq_str = " -> ".join(seq_states)
    print(f"Exact P({start_state} -> {seq_str}) = {prob:.6f}")
    return 0

def _analytic_stationary(P: np.ndarray) -> np.ndarray:
    """Analytic stationary distribution (eigenvector of P^T)."""
    w, v = np.linalg.eig(P.T)
    idx = np.argmin(np.abs(w - 1.0))
    pi = np.real(v[:, idx])
    pi = pi / pi.sum()
    return pi

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Weather Markov chain tools")
    parser.add_argument("--start", "-s", help="Start day: s/c/r or sunny/cloudy/rainy")
    parser.add_argument("--seq", "-q", nargs="+", help="Sequence: e.g., c c r OR ccr OR 'cloudy cloudy rainy'")
    parser.add_argument("--mc", type=int, default=0, help="Optional MC trials to verify probability (0 to skip)")
    parser.add_argument("--seed", type=int, default=0, help="Seed for MC")
    # NEW histogram options
    parser.add_argument("--hist-length", type=int, default=10000,
                        help="Number of future days to simulate for histogram (default 10000, set 0 to skip)")
    parser.add_argument("--hist-file", type=str, default="weather_hist.png",
                        help="Output PNG filepath for histogram")
    parser.add_argument("--no-show", action="store_true",
                        help="Save histogram but do not display window")
    # NEW convergence plot option
    parser.add_argument("--converge", action="store_true",
                        help="Also generate cumulative frequency convergence plot")
    args = parser.parse_args(argv)

    if not args.start and not args.seq:
        return interactive()

    if not args.start or not args.seq:
        parser.error("Both --start and --seq are required when not using non-interactive mode.")

    try:
        start_state = parse_one_state(args.start)
        seq_states = parse_seq(args.seq)
    except ValueError as e:
        parser.error(str(e))
        return 2

    prob = sequence_probability(start_state, seq_states)
    seq_str = " -> ".join(seq_states)
    print(f"Exact P({start_state} -> {seq_str}) = {prob:.6f}")

    if args.mc and args.mc > 0:
        mc = estimate_sequence_probability_mc(start_state, seq_states, trials=args.mc, seed=args.seed)
        print(f"MC    P({start_state} -> {seq_str}) ≈ {mc:.6f}  (trials={args.mc}, seed={args.seed})")

    # NEW: Histogram generation
    if args.hist_length and args.hist_length > 0:
        rng = np.random.default_rng(args.seed)
        sim_seq = simulate_weather(start_state, args.hist_length, rng)
        counts = Counter(sim_seq[1:])  # exclude initial day
        total = sum(counts.values())
        print(f"Histogram over next {args.hist_length} days (proportions):")
        for s in STATES:
            print(f"  {s:6s}: {counts[s]/total:.4f}")

        import matplotlib.pyplot as plt
        freqs = [counts[s]/total for s in STATES]

        # Convergence data
        cumulative_counts = np.zeros(3, dtype=np.int64)
        cum_freqs = np.zeros((args.hist_length, 3), dtype=float)
        for i, st in enumerate(sim_seq[1:]):  # days 1..hist_length
            cumulative_counts[IDX[st]] += 1
            cum_freqs[i] = cumulative_counts / (i + 1)

        stationary = _analytic_stationary(P)

        if args.converge:
            fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), dpi=110)
            ax_bar, ax_line = axes
        else:
            fig, ax_bar = plt.subplots(1, 1, figsize=(6, 4), dpi=110)

        # Bar histogram
        bars = ax_bar.bar(STATES, freqs, color=["gold", "gray", "royalblue"],
                          edgecolor="k", alpha=0.85)
        for b, v in zip(bars, freqs):
            ax_bar.text(b.get_x()+b.get_width()/2, b.get_height()+0.002, f"{v:.3f}",
                        ha="center", va="bottom", fontsize=9)
        ax_bar.set_title(f"Histogram after {args.hist_length} days (start='{start_state}')")
        ax_bar.set_ylabel("Relative frequency")
        ax_bar.set_ylim(0, max(freqs)*1.15)
        ax_bar.grid(axis="y", linestyle=":", alpha=0.4)

        if args.converge:
            x = np.arange(1, args.hist_length + 1)
            for i, s in enumerate(STATES):
                ax_line.plot(x, cum_freqs[:, i], label=f"{s}", lw=1.4)
                ax_line.hlines(stationary[i], 1, args.hist_length,
                               colors=ax_line.lines[-1].get_color(),
                               linestyles="dashed", alpha=0.6)
            ax_line.set_xscale("log")
            ax_line.set_xlabel("Day (log scale)")
            ax_line.set_ylabel("Cumulative frequency")
            ax_line.set_title("Convergence of cumulative frequencies")
            ax_line.grid(True, linestyle=":", alpha=0.4)
            ax_line.legend(frameon=False, fontsize=9)

        # Ensure visuals directory exists at same level as this script
        visuals_dir = os.path.join(os.path.dirname(__file__), "visuals")
        os.makedirs(visuals_dir, exist_ok=True)

        # Always save inside visuals dir, use basename of requested file
        out_name = os.path.basename(args.hist_file)
        out_path = os.path.join(visuals_dir, out_name)

        fig.tight_layout()
        fig.savefig(out_path)
        print(f"Saved histogram figure: {out_path}")
        if not args.no_show:
            plt.show()
        else:
            plt.close(fig)

    return 0

if __name__ == "__main__":
    raise SystemExit(main())

# Examples

# Interactive:
# python generate_weather.py
# CLI letters:
# python generate_weather.py --start s --seq c c r
# python generate_weather.py --start s --seq ccr
# CLI names:
# python generate_weather.py --start sunny --seq cloudy cloudy rainy
# With MC check:
# python generate_weather.py --start s --seq ccr --mc 200000 --seed 0