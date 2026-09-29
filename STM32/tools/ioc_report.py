#!/usr/bin/env python3
"""
ioc_report.py - pre-flight report for an STM32CubeMX .ioc file.

Answers "what has what channels, and what is free" without clicking through
CubeMX, and flags the configuration traps that CubeMX reports only as warnings.

Usage:
    python3 ioc_report.py ros32bot_stm32.ioc
    python3 ioc_report.py ros32bot_stm32.ioc --db ~/STM32CubeMX/db

The --db option (auto-detected from ~/STM32CubeMX/db if present) enables the
chip-wide pin-function matrix and the free-pin list, read from the MCU XML that
ships with CubeMX. Without it, the report still covers the .ioc itself.
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

NS = {"m": "http://mcd.rou.st.com/modules.php?name=mcu"}

# osPriority mapping for CMSIS-RTOS v2, used to decode FREERTOS.Tasks01
OS_PRIORITY = {
    "osPriorityNone": 0, "osPriorityIdle": 1,
    **{f"osPriorityLow{i}" if i else "osPriorityLow": 8 + i for i in range(8)},
    **{f"osPriorityNormal{i}" if i else "osPriorityNormal": 24 + i for i in range(8)},
    **{f"osPriorityHigh{i}" if i else "osPriorityHigh": 32 + i for i in range(8)},
    "osPriorityRealtime": 56,
}


def decode_priority(n):
    """Map a raw CubeMX priority number back to its osPriority name."""
    for name, val in sorted(OS_PRIORITY.items(), key=lambda kv: -kv[1]):
        if val == n:
            return name
    return "?"


def parse_ioc(path):
    cfg = {}
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            cfg[key] = val
    return cfg


def enabled_ips(cfg):
    """Mcu.IP<n>=NAME -> sorted list of peripheral instance names."""
    ips = []
    for key, val in cfg.items():
        if re.fullmatch(r"Mcu\.IP\d+", key):
            ips.append(val)
    return sorted(ips)


def assigned_pins(cfg):
    """
    Build pin -> {signal, mode, label} from the .ioc.

    Signal may be recorded as a plain value (PA9.Signal=USART1_TX) or via a
    peripheral parameter (fixed) — we handle the plain form plus the S_* form
    used for timer channels.
    """
    pins = []
    for key, val in cfg.items():
        if re.fullmatch(r"Mcu\.Pin\d+", key):
            pins.append(val)
    out = {}
    for pin in pins:
        out[pin] = {
            "signal": cfg.get(f"{pin}.Signal", ""),
            "mode": cfg.get(f"{pin}.Mode", ""),
            "label": cfg.get(f"{pin}.GPIO_Label", ""),
        }
    return dict(sorted(out.items()))


def timer_channels(cfg):
    """
    Parse SH.<signal>.<n>=<SIGNAL>[,<MODE>] entries for timer channels.

    Returns {timer: {channel: mode_or_None}}. A None mode means the channel has
    a signal assigned but no mode - the yellow-triangle state that makes PWM
    unavailable (see SETUP guide 9.7).
    """
    timers = defaultdict(dict)
    for key, val in cfg.items():
        # channel signals may carry an _ETR suffix (TIM2_CH1_ETR shares PA0)
        m = re.fullmatch(r"SH\.(S_TIM\d+_CH\d+N?(?:_ETR)?)\.\d+", key)
        if not m:
            continue
        signal = m.group(1)
        tmatch = re.match(r"S_(TIM\d+)_(CH\d+N?)(?:_ETR)?$", signal)
        if not tmatch:
            continue
        timer, chan = tmatch.group(1), tmatch.group(2)
        parts = val.split(",", 1)
        mode = parts[1].strip() if len(parts) > 1 and parts[1].strip() else None
        timers[timer][chan] = mode
    return {t: dict(sorted(c.items())) for t, c in sorted(timers.items())}


def init_functions(cfg):
    """Return the set of MX_*_Init functions CubeMX plans to generate."""
    raw = cfg.get("ProjectManager.functionlistsort", "")
    found = set()
    for entry in raw.split(","):
        m = re.match(r"\d+-(MX_\w+)-", entry.strip())
        if m:
            found.add(m.group(1))
    return found


def mcu_xml_path(cfg, db_root):
    """Locate the device XML using Mcu.Name, which matches the filename."""
    name = cfg.get("Mcu.Name")
    if not name or not db_root:
        return None
    candidate = os.path.join(db_root, "mcu", f"{name}.xml")
    return candidate if os.path.isfile(candidate) else None


def chip_functions(xml_path):
    """pin -> [signal names] for every I/O pin on the device."""
    if not xml_path:
        return {}
    root = ET.parse(xml_path).getroot()
    funcs = {}
    for pin in root.findall("m:Pin", NS):
        if pin.get("Type") != "I/O":
            continue
        sigs = [s.get("Name") for s in pin.findall("m:Signal", NS) if s.get("Name")]
        funcs[pin.get("Name")] = sigs
    return funcs


def hr(title):
    print(f"\n{title}\n{'-' * len(title)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ioc")
    ap.add_argument("--db", default=os.path.expanduser("~/STM32CubeMX/db"),
                    help="CubeMX db directory (for the pin-function matrix)")
    args = ap.parse_args()

    if not os.path.isfile(args.ioc):
        sys.exit(f"error: no such file: {args.ioc}")

    cfg = parse_ioc(args.ioc)
    print(f"ioc        : {args.ioc}")
    print(f"device     : {cfg.get('Mcu.CPN', '?')}  ({cfg.get('Mcu.Package', '?')})")

    # ---- enabled peripherals -------------------------------------------------
    hr("ENABLED PERIPHERALS (Mcu.IP list)")
    ips = enabled_ips(cfg)
    print("  " + ", ".join(ips) if ips else "  (none)")
    print(f"  count: {cfg.get('Mcu.IPNb', '?')}")

    # ---- pin allocation ------------------------------------------------------
    pins = assigned_pins(cfg)
    hr(f"PIN ALLOCATION ({len(pins)} pins)")
    print(f"  {'PIN':<18} {'SIGNAL':<20} {'MODE':<22} LABEL")
    for pin, info in pins.items():
        print(f"  {pin:<18} {info['signal'] or '-':<20} "
              f"{info['mode'] or '-':<22} {info['label'] or '-'}")

    # ---- timer channels ------------------------------------------------------
    chans = timer_channels(cfg)
    hr("TIMER CHANNELS")
    unconfigured = []
    if not chans:
        print("  (no timer channels assigned)")
    for timer, chans_ in chans.items():
        for chan, mode in chans_.items():
            flag = "" if mode else "   <-- NO MODE SET (will not work)"
            print(f"  {timer}_{chan:<5} {mode or '(none)':<28}{flag}")
            if not mode:
                unconfigured.append(f"{timer}_{chan}")

    # encoder mode needs BOTH TI1 and TI2 (TI2 is required for direction even in
    # TI1-only mode), so a lone encoder channel is always an incomplete config
    incomplete_enc = []
    for timer, chans_ in chans.items():
        enc = sorted(c for c, m in chans_.items() if m and "Encoder" in m)
        if enc and len(enc) < 2:
            incomplete_enc.append(f"{timer} has only {', '.join(enc)} in Encoder mode "
                                  f"(encoder needs both CH1 and CH2)")

    # ---- init functions ------------------------------------------------------
    hr("GENERATED INIT FUNCTIONS (from functionlistsort)")
    inits = init_functions(cfg)
    print("  " + ", ".join(sorted(inits)))
    # an enabled timer with no MX_*_Init is the TIM1-not-instantiated symptom
    for ip in ips:
        if ip.startswith("TIM") and f"MX_{ip}_Init" not in inits:
            print(f"  !! {ip} is listed as an enabled IP but MX_{ip}_Init is absent")

    # ---- clocks --------------------------------------------------------------
    hr("CLOCK CHECK")
    checks = [
        ("SYSCLK", cfg.get("RCC.SYSCLKFreq_VALUE"), "72000000", "should be 72 MHz"),
        ("HCLK", cfg.get("RCC.HCLKFreq_Value"), "72000000", "should be 72 MHz"),
        ("APB1 div", cfg.get("RCC.APB1CLKDivider"), "RCC_HCLK_DIV2", "must be DIV2 (36 MHz max)"),
        ("APB2 div", cfg.get("RCC.APB2CLKDivider", "RCC_HCLK_DIV1"), "RCC_HCLK_DIV1", "should be DIV1 (72 MHz)"),
        ("PCLK1", cfg.get("RCC.APB1Freq_Value"), "36000000", "36 MHz"),
        ("PCLK2", cfg.get("RCC.APB2Freq_Value"), "72000000", "72 MHz"),
        ("APB1 tim clk", cfg.get("RCC.APB1TimFreq_Value"), "72000000", "should be 72 MHz"),
        ("APB2 tim clk", cfg.get("RCC.APB2TimFreq_Value"), "72000000", "should be 72 MHz"),
        ("ADC clk", cfg.get("RCC.ADCFreqValue"), None, "must be <= 14 MHz"),
    ]
    adc_enabled = any(i.startswith("ADC") for i in ips)
    for name, got, want, note in checks:
        if name == "ADC clk" and not adc_enabled:
            print(f"  {name:<14} {str(got):<12} {note} (ADC not enabled - not checked)")
            continue
        if got is None:
            print(f"  {name:<14} {'?':<12} {note}")
            continue
        if name == "ADC clk":
            try:
                bad = int(got) > 14_000_000
            except ValueError:
                bad = False
        else:
            bad = (got != want)
        mark = "  <-- CHECK" if bad else ""
        print(f"  {name:<14} {got:<12} {note}{mark}")

    # ---- freeRTOS ------------------------------------------------------------
    tasks = cfg.get("FREERTOS.Tasks01")
    if tasks:
        hr("FREERTOS TASKS (higher number = higher priority)")
        rows = []
        for entry in tasks.split(";"):
            f = entry.split(",")
            if len(f) >= 3:
                try:
                    prio = int(f[1])
                except ValueError:
                    prio = -1
                rows.append((prio, f[0], f[2], f[3] if len(f) > 3 else ""))
        for prio, name, stack, entry in sorted(rows, key=lambda r: -r[0]):
            words = int(stack) if stack.isdigit() else 0
            print(f"  prio {prio:<3} ({decode_priority(prio):<18}) "
                  f"stack {words:>4} words ({words * 4:>5} B)  {name}  -> {entry}")
        print("  note: the control-loop task should have the highest priority")

    # ---- chip-wide view ------------------------------------------------------
    xml_path = mcu_xml_path(cfg, args.db)
    if not xml_path:
        print("\n(chip-wide pin matrix skipped: device XML not found; "
              "pass --db or omit for .ioc-only report)")
        return

    funcs = chip_functions(xml_path)
    used = set(pins.keys())

    hr("FUNCTIONS AVAILABLE ON EACH USED PIN")
    print("  (the alternatives CubeMX will offer if you free a peripheral)")
    for pin in sorted(pins):
        alts = funcs.get(pin)
        if not alts:
            continue
        cur = pins[pin]["signal"]
        alt_txt = ", ".join(a for a in alts if a != cur and a != "GPIO")
        print(f"  {pin:<18} in use: {cur or '-':<18} alternatives: {alt_txt or '(none)'}")

    hr("FREE I/O PINS")
    free = [p for p in sorted(funcs) if p not in used]
    print(f"  count: {len(free)}")
    for pin in free:
        print(f"  {pin:<18} could be: {', '.join(funcs[pin])}")

    # ---- summary of problems -------------------------------------------------
    problems = []
    if unconfigured:
        problems.append(f"timer channels with no mode: {', '.join(unconfigured)}")
    problems.extend(incomplete_enc)
    if adc_enabled:
        try:
            if int(cfg.get("RCC.ADCFreqValue", 0)) > 14_000_000:
                problems.append(f"ADC clock {cfg.get('RCC.ADCFreqValue')} exceeds 14 MHz")
        except ValueError:
            pass
    if cfg.get("RCC.APB2CLKDivider", "RCC_HCLK_DIV1") == "RCC_HCLK_DIV2":
        extra = " (ADC clock would be derived from a halved PCLK2)" if adc_enabled else ""
        problems.append(f"APB2 divider is DIV2 (expected DIV1) - halves PCLK2{extra}")
    hr("PROBLEMS FOUND")
    if problems:
        for p in problems:
            print(f"  !! {p}")
    else:
        print("  none detected")


if __name__ == "__main__":
    main()
