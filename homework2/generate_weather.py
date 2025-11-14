import argparse
import sys
import numpy as np
from collections import Counter

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

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Weather Markov chain tools")
    parser.add_argument("--start", "-s", help="Start day: s/c/r or sunny/cloudy/rainy")
    parser.add_argument("--seq", "-q", nargs="+", help="Sequence: e.g., c c r OR ccr OR 'cloudy cloudy rainy'")
    parser.add_argument("--mc", type=int, default=0, help="Optional MC trials to verify probability (0 to skip)")
    parser.add_argument("--seed", type=int, default=0, help="Seed for MC")
    args = parser.parse_args(argv)

    if not args.start and not args.seq:
        return interactive()

    if not args.start or not args.seq:
        parser.error("Both --start and --seq are required when not using interactive mode.")

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