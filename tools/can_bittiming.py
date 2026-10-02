#!/usr/bin/env python3
"""
bxCAN bit-timing calculator.

Enumerates every (prescaler, BS1, BS2) combination the STM32 bxCAN peripheral
supports for a given CAN kernel clock and bit rate, and ranks the exact
matches by distance from the target sample point.

    $ python3 tools/can_bittiming.py                       # 45 MHz APB1, 500 kbit/s
    $ python3 tools/can_bittiming.py --clock 42e6 --bitrate 250e3 --sample 0.875

bxCAN limits (RM0390 sec. 30.7.7): prescaler 1..1024, BS1 1..16 tq,
BS2 1..8 tq, SJW 1..4 tq. Bit = SYNC_SEG (1 tq) + BS1 + BS2.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class Timing:
    prescaler: int
    bs1: int
    bs2: int
    clock_hz: float

    @property
    def tq_per_bit(self) -> int:
        return 1 + self.bs1 + self.bs2

    @property
    def bitrate(self) -> float:
        return self.clock_hz / (self.prescaler * self.tq_per_bit)

    @property
    def sample_point(self) -> float:
        return (1 + self.bs1) / self.tq_per_bit

    @property
    def max_sjw(self) -> int:
        return min(4, self.bs2)


def solve(clock_hz: float, bitrate: float, target_sp: float,
          sp_min: float = 0.75, sp_max: float = 0.90) -> list[Timing]:
    """Return exact-bit-rate timings inside [sp_min, sp_max], best first."""
    found = []
    for prescaler in range(1, 1025):
        for bs1 in range(1, 17):
            for bs2 in range(1, 9):
                t = Timing(prescaler, bs1, bs2, clock_hz)
                if abs(t.bitrate - bitrate) > 1e-6 * bitrate:
                    continue
                if not sp_min <= t.sample_point <= sp_max:
                    continue
                found.append(t)
    # Prefer: closest sample point, then more tq per bit (finer resync resolution).
    found.sort(key=lambda t: (abs(t.sample_point - target_sp), -t.tq_per_bit))
    return found


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clock", type=float, default=45e6, help="CAN kernel clock (APB1) in Hz [45e6]")
    ap.add_argument("--bitrate", type=float, default=500e3, help="bit rate in bit/s [500e3]")
    ap.add_argument("--sample", type=float, default=0.875, help="target sample point [0.875]")
    ap.add_argument("--top", type=int, default=10, help="rows to print [10]")
    args = ap.parse_args()

    rows = solve(args.clock, args.bitrate, args.sample)
    if not rows:
        raise SystemExit("No exact solution: change the APB1 clock or accept a bit-rate error.")

    print(f"clock {args.clock / 1e6:g} MHz, {args.bitrate / 1e3:g} kbit/s, target SP {args.sample:.1%}\n")
    print(f"{'presc':>5} {'tq/bit':>6} {'BS1':>4} {'BS2':>4} {'SJWmax':>6} {'sample':>7} {'tq(ns)':>7}")
    for t in rows[: args.top]:
        tq_ns = t.prescaler / args.clock * 1e9
        print(f"{t.prescaler:>5} {t.tq_per_bit:>6} {t.bs1:>4} {t.bs2:>4} {t.max_sjw:>6} "
              f"{t.sample_point:>7.1%} {tq_ns:>7.1f}")
    print("\nHAL: Prescaler=presc, TimeSeg1=CAN_BS1_<BS1>TQ, TimeSeg2=CAN_BS2_<BS2>TQ")


if __name__ == "__main__":
    main()
