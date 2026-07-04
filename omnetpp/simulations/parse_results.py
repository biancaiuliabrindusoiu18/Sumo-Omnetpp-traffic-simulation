#!/usr/bin/env python3
"""

Utilizare:
    python parse_results.py
    python parse_results.py --dir results/
    python parse_results.py --config Varf_2_Dinamic
"""

import os
import re
import sys
import argparse
import glob
import math
from collections import defaultdict

try:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("[WARN] matplotlib/numpy lipsesc — graficele sunt dezactivate.")


#  CONFIGURATII CUNOSCUTE (din omnetpp.ini)
KNOWN_CONFIGS = [
    "Noapte_1_GreenWave",
    "Noapte_2_Dinamic",
    "Normal_1_NoSync",
    "Normal_2_GreenWave",
    "Normal_3_Dinamic",
    "Varf_1_GreenWave",
    "Varf_2_Dinamic",
    "Varf_4_NoSync",
    #"Varf_3_Dinamic_V2I",
    "Aglomerat_1_GreenWave",
    "Aglomerat_2_Dinamic",
    "Aglomerat_4_NoSync",
    #"Aglomerat_3_Dinamic_V2I",
]


#  PARSARE .SCA
def parse_sca(filepath):
    scalars = defaultdict(dict)
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("scalar"):
                continue
            parts = line.split(None, 3)
            if len(parts) < 4:
                continue
            _, module, name, value = parts
            try:
                scalars[module][name] = float(value)
            except ValueError:
                pass
    return scalars


#  PARSARE .VEC
def parse_vec(filepath):
    vectors = {}
    meta = {}

    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("vector "):
                parts = line.split(None, 3)
                if len(parts) >= 4:
                    vid = int(parts[1])
                    meta[vid] = (parts[2], parts[3])
                    vectors[vid] = {"module": parts[2], "name": parts[3],
                                    "times": [], "values": []}
            else:
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        vid = int(parts[0])
                        if vid in vectors:
                            vectors[vid]["times"].append(float(parts[-2]))
                            vectors[vid]["values"].append(float(parts[-1]))
                    except (ValueError, IndexError):
                        pass
    return vectors


#  STATISTICI COADA PER-RSU DIN .VEC
def rsu_queue_stats_from_vec(vec_files, signal_name):
    acc = {}
    for vf in vec_files:
        target = {}
        try:
            with open(vf, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    if not line:
                        continue
                    c0 = line[0]
                    if c0 == "v" and line.startswith("vector "):
                        parts = line.split(None, 4)
                        if len(parts) >= 4:
                            vid    = parts[1]
                            module = parts[2]
                            name   = parts[3].strip()
                            if name == signal_name and "rsu" in module.lower():
                                target[vid] = module
                    elif c0.isdigit() and target:
                        sp = line.split()
                        if len(sp) >= 3 and sp[0] in target:
                            try:
                                val = float(sp[-1])
                            except ValueError:
                                continue
                            mod = target[sp[0]]
                            a = acc.get(mod)
                            if a is None:
                                acc[mod] = [val, 1, val]
                            else:
                                a[0] += val; a[1] += 1
                                if val > a[2]: a[2] = val
        except KeyboardInterrupt:
            print("\n[INFO] citire coada BSM intrerupta — tabel afisat fara BSM.")
            return {}
    out = {}
    for mod, (s, n, mx) in acc.items():
        if n > 0:
            out[mod] = (s / n, mx)
    return out

def attach_bsm_queue(m, vec_files):
    if vec_files:
        print("[INFO] citesc cozile (queueBlvd / queueSecondary / queueSecBsm) din .vec...")

    bsm = rsu_queue_stats_from_vec(vec_files, "queueBlvd") if vec_files else {}
    m["rsu_bsm_mean"] = {k: v[0] for k, v in bsm.items()}
    m["rsu_bsm_max"]  = {k: v[1] for k, v in bsm.items()}

    sec_real = rsu_queue_stats_from_vec(vec_files, "queueSecondary") if vec_files else {}
    m["rsu_sec_real_mean"] = {k: v[0] for k, v in sec_real.items()}
    m["rsu_sec_real_max"]  = {k: v[1] for k, v in sec_real.items()}

    sec_bsm = rsu_queue_stats_from_vec(vec_files, "queueSecBsm") if vec_files else {}
    m["rsu_sec_bsm_mean"] = {k: v[0] for k, v in sec_bsm.items()}
    m["rsu_sec_bsm_max"]  = {k: v[1] for k, v in sec_bsm.items()}
    return m


#  GASIRE FISIERE
def find_files(results_dir, config_name):
    pattern_sca = os.path.join(results_dir, f"{config_name}*.sca")
    pattern_vec = os.path.join(results_dir, f"{config_name}*.vec")
    sca_files = sorted(glob.glob(pattern_sca))
    vec_files = sorted(glob.glob(pattern_vec))
    return sca_files, vec_files


#  EXTRAGERE METRICI DIN .SCA
def extract_metrics(scalars):
    m = {}

    travel_times   = []
    waiting_times  = []
    avg_speeds     = []
    stops_list     = []
    co2_list       = []
    route_completed_sum   = 0
    route_completed_count = 0

    gw_travel  = []; gw_waiting = []; gw_speed = []; gw_stops = []
    gw_rc_sum = 0;   gw_rc_count = 0

    gwe_travel = []; gwe_waiting = []; gwe_speed = []; gwe_stops = []
    gwe_rc_sum = 0;  gwe_rc_count = 0

    rsu_adjustments   = {}
    rsu_throughput    = {}
    rsu_stopped       = {}
    rsu_avg_speed     = {}
    rsu_queue_mean    = {}
    rsu_queue_max     = {}
    rsu_stage_mean    = {}
    rsu_stage_max     = {}
    rsu_main_green_mean = {}
    rsu_main_green_max  = {}
    rsu_sec_green_mean  = {}
    rsu_sec_green_max   = {}

    for module, stats in scalars.items():
        if "veinsmobility" in module:
            if "totalCO2Emission" in stats:
                co2_list.append(stats["totalCO2Emission"])

        elif ".appl" in module and "rsu" not in module.lower():
            for k, lst in [("travelTime:mean",   travel_times),
                           ("waitingTime:mean",   waiting_times),
                           ("avgSpeed:mean",      avg_speeds),
                           ("numberOfStops:mean", stops_list)]:
                if k in stats:
                    lst.append(stats[k])

            if "routeCompleted:sum"   in stats: route_completed_sum   += stats["routeCompleted:sum"]
            if "routeCompleted:count" in stats: route_completed_count += stats["routeCompleted:count"]

            def _get(s, base):
                for suf in [":mean", ":last", ""]:
                    if base + suf in s:
                        return s[base + suf]
                return None

            for base, lst in [("gwTravelTime",  gw_travel),
                               ("gwWaitingTime", gw_waiting),
                               ("gwAvgSpeed",    gw_speed),
                               ("gwStops",       gw_stops)]:
                v = _get(stats, base)
                if v is not None: lst.append(v)
            for suf in [":sum", ""]:
                if "gwRouteCompleted" + suf in stats:
                    gw_rc_sum += stats["gwRouteCompleted" + suf]; break
            for suf in [":count", ""]:
                if "gwRouteCompleted" + suf in stats:
                    gw_rc_count += stats["gwRouteCompleted" + suf]; break

            for base, lst in [("gwWETravelTime",  gwe_travel),
                               ("gwWEWaitingTime", gwe_waiting),
                               ("gwWEAvgSpeed",    gwe_speed),
                               ("gwWEStops",       gwe_stops)]:
                v = _get(stats, base)
                if v is not None: lst.append(v)
            for suf in [":sum", ""]:
                if "gwWERouteCompleted" + suf in stats:
                    gwe_rc_sum += stats["gwWERouteCompleted" + suf]; break
            for suf in [":count", ""]:
                if "gwWERouteCompleted" + suf in stats:
                    gwe_rc_count += stats["gwWERouteCompleted" + suf]; break

        elif "rsu" in module.lower() and ".appl" in module:
            rsu_id = module
            if "adjustmentsApplied:last"      in stats: rsu_adjustments[rsu_id]  = stats["adjustmentsApplied:last"]
            if "vehiclesThroughput:last"       in stats: rsu_throughput[rsu_id]   = stats["vehiclesThroughput:last"]
            if "numberOfStoppedVehicles:last"  in stats: rsu_stopped[rsu_id]      = stats["numberOfStoppedVehicles:last"]
            
            # Viteza medie la nivel de RSU
            if "avgSpeed:mean" in stats: rsu_avg_speed[rsu_id] = stats["avgSpeed:mean"]
            elif "measuredSpeed:mean" in stats: rsu_avg_speed[rsu_id] = stats["measuredSpeed:mean"]
            elif "averageSpeed:mean" in stats: rsu_avg_speed[rsu_id] = stats["averageSpeed:mean"]
            elif "avgSpeed:last" in stats: rsu_avg_speed[rsu_id] = stats["avgSpeed:last"]

            if "queueBlvdReal:mean"            in stats: rsu_queue_mean[rsu_id]   = stats["queueBlvdReal:mean"]
            if "queueBlvdReal:max"             in stats: rsu_queue_max[rsu_id]    = stats["queueBlvdReal:max"]
            if "currentStage:mean"             in stats: rsu_stage_mean[rsu_id]   = stats["currentStage:mean"]
            if "currentStage:max"              in stats: rsu_stage_max[rsu_id]    = stats["currentStage:max"]
            if "mainGreenDuration:mean"        in stats: rsu_main_green_mean[rsu_id] = stats["mainGreenDuration:mean"]
            if "mainGreenDuration:max"         in stats: rsu_main_green_max[rsu_id]  = stats["mainGreenDuration:max"]
            if "secondaryGreenDuration:mean"   in stats: rsu_sec_green_mean[rsu_id]  = stats["secondaryGreenDuration:mean"]
            if "secondaryGreenDuration:max"    in stats: rsu_sec_green_max[rsu_id]   = stats["secondaryGreenDuration:max"]

    def safe_mean(lst): 
        valide = [x for x in lst if x is not None and not math.isnan(x)]
        return sum(valide) / len(valide) if valide else None

    m["all_travelTime_mean"]   = safe_mean(travel_times)
    m["all_waitingTime_mean"]  = safe_mean(waiting_times)
    m["all_avgSpeed_mean"]     = safe_mean(avg_speeds)
    m["all_stops_mean"]        = safe_mean(stops_list)
    m["all_routeCompleted_pct"] = (route_completed_sum / route_completed_count * 100 if route_completed_count > 0 else None)
    m["all_vehicles_total"]    = int(route_completed_count) if route_completed_count else None

    m["co2_total_kg"]          = sum(co2_list) / 1000.0 if co2_list else None
    m["co2_mean_per_veh_g"]    = safe_mean(co2_list) if co2_list else None

    m["gw_travelTime_mean"]    = safe_mean(gw_travel)
    m["gw_waitingTime_mean"]   = safe_mean(gw_waiting)
    m["gw_avgSpeed_mean"]      = safe_mean(gw_speed)
    m["gw_stops_mean"]         = safe_mean(gw_stops)
    m["gw_routeCompleted_pct"] = (gw_rc_sum / gw_rc_count * 100 if gw_rc_count > 0 else None)
    m["gw_vehicles_total"]     = int(gw_rc_count) if gw_rc_count else None

    m["gwe_travelTime_mean"]   = safe_mean(gwe_travel)
    m["gwe_waitingTime_mean"]  = safe_mean(gwe_waiting)
    m["gwe_avgSpeed_mean"]     = safe_mean(gwe_speed)
    m["gwe_stops_mean"]        = safe_mean(gwe_stops)
    m["gwe_routeCompleted_pct"] = (gwe_rc_sum / gwe_rc_count * 100 if gwe_rc_count > 0 else None)
    m["gwe_vehicles_total"]    = int(gwe_rc_count) if gwe_rc_count else None

    m["rsu_adjustments"]  = rsu_adjustments
    m["rsu_throughput"]   = rsu_throughput
    m["rsu_stopped"]      = rsu_stopped
    m["rsu_avg_speed"]    = rsu_avg_speed
    m["rsu_queue_mean"]   = rsu_queue_mean
    m["rsu_queue_max"]    = rsu_queue_max
    m["rsu_stage_mean"]   = rsu_stage_mean
    m["rsu_stage_max"]    = rsu_stage_max
    m["rsu_main_green_mean"] = rsu_main_green_mean
    m["rsu_main_green_max"]  = rsu_main_green_max
    m["rsu_sec_green_mean"]  = rsu_sec_green_mean
    m["rsu_sec_green_max"]   = rsu_sec_green_max

    return m


#  AFISARE REZULTATE
def fmt(val, unit="", scale=1.0, decimals=2):
    if val is None:
        return "N/A"
    return f"{val * scale:.{decimals}f}{unit}"

def print_metrics(config_name, m):
    sep = "─" * 60
    print(f"\n{'═'*60}")
    print(f"  SCENARIU: {config_name}")
    print(f"{'═'*60}")

    print(f"\n{'TOATE VEHICULELE':^60}")
    print(sep)
    print(f"  Vehicule totale:        {fmt(m['all_vehicles_total'], '', 1, 0)}")
    print(f"  Ruta completata:        {fmt(m['all_routeCompleted_pct'], '%')}")
    print(f"  Travel time mediu:      {fmt(m['all_travelTime_mean'], 's')}")
    print(f"  Waiting time mediu:     {fmt(m['all_waitingTime_mean'], 's')}")
    print(f"  Viteza medie:           {fmt(m['all_avgSpeed_mean'], ' km/h', 3.6)}")
    print(f"  Opriri medii/vehicul:   {fmt(m['all_stops_mean'])}")
    if m['co2_total_kg'] is not None:
        print(f"  CO2 total:              {fmt(m['co2_total_kg'], ' kg', 1, 1)}")
        print(f"  CO2 mediu/vehicul:      {fmt(m['co2_mean_per_veh_g'], ' g', 1, 1)}")

    print(f"\n{'BULEVARD (WE + EW)':^60}")
    print(sep)
    print(f"  Vehicule bulevard:      {fmt(m['gw_vehicles_total'], '', 1, 0)}")
    print(f"  Ruta completata:        {fmt(m['gw_routeCompleted_pct'], '%')}")
    print(f"  Travel time mediu:      {fmt(m['gw_travelTime_mean'], 's')}")
    print(f"  Waiting time mediu:     {fmt(m['gw_waitingTime_mean'], 's')}")
    print(f"  Viteza medie:           {fmt(m['gw_avgSpeed_mean'], ' km/h', 3.6)}")
    print(f"  Opriri medii/vehicul:   {fmt(m['gw_stops_mean'])}")

    print(f"\n{'BULEVARD WE (Vest → Est)':^60}")
    print(sep)
    print(f"  Vehicule WE:            {fmt(m['gwe_vehicles_total'], '', 1, 0)}")
    print(f"  Ruta completata:        {fmt(m['gwe_routeCompleted_pct'], '%')}")
    print(f"  Travel time mediu:      {fmt(m['gwe_travelTime_mean'], 's')}")
    print(f"  Waiting time mediu:     {fmt(m['gwe_waitingTime_mean'], 's')}")
    print(f"  Viteza medie:           {fmt(m['gwe_avgSpeed_mean'], ' km/h', 3.6)}")
    print(f"  Opriri medii/vehicul:   {fmt(m['gwe_stops_mean'])}")

    if m["rsu_adjustments"] or m["rsu_queue_mean"]:
        print(f"\n{'RSU — INTERSECTII':^60}")
        print(sep)
        def rsu_idx(mod):
            match = re.search(r'rsu\[(\d+)\]', mod)
            return int(match.group(1)) if match else 99

        all_rsu = sorted(set(
            list(m["rsu_adjustments"].keys()) +
            list(m["rsu_queue_mean"].keys()) +
            list(m.get("rsu_sec_real_mean", {}).keys()) +
            list(m.get("rsu_sec_bsm_mean", {}).keys()) +
            list(m.get("rsu_avg_speed", {}).keys())
        ), key=rsu_idx)

        total_adj = 0
        for rsu in all_rsu:
            idx = rsu_idx(rsu)
            adj  = m["rsu_adjustments"].get(rsu)
            qm   = m["rsu_queue_mean"].get(rsu)
            qmax = m["rsu_queue_max"].get(rsu)
            bm   = m.get("rsu_bsm_mean", {}).get(rsu)
            bmax = m.get("rsu_bsm_max", {}).get(rsu)
            srm   = m.get("rsu_sec_real_mean", {}).get(rsu)
            srmax = m.get("rsu_sec_real_max", {}).get(rsu)
            sbm   = m.get("rsu_sec_bsm_mean", {}).get(rsu)
            sbmax = m.get("rsu_sec_bsm_max", {}).get(rsu)
            sm   = m["rsu_stage_mean"].get(rsu)
            smax = m["rsu_stage_max"].get(rsu)
            mgm  = m.get("rsu_main_green_mean", {}).get(rsu)
            sgm  = m.get("rsu_sec_green_mean", {}).get(rsu)
            if mgm is not None and math.isnan(mgm): mgm = None
            if sgm is not None and math.isnan(sgm): sgm = None
            stp  = m.get("rsu_stopped", {}).get(rsu)
            thr  = m["rsu_throughput"].get(rsu)
            asp  = m.get("rsu_avg_speed", {}).get(rsu)

            stable_mark = " [ANCHOR]" if idx == 8 else ""
            print(f"  RSU[{idx}]{stable_mark}")
            if adj  is not None: print(f"    adjustments:       {int(adj)}")

            if qm is not None or bm is not None:
                print(f"    -- BULEVARD (main) --")
            if qm   is not None: print(f"    queue real mean:   {qm:.1f} veh   (detector/TraCI)")
            if qmax is not None: print(f"    queue real max:    {qmax:.0f} veh")
            if bm   is not None: print(f"    queue BSM  mean:   {bm:.1f} veh   (V2I)")
            if bmax is not None: print(f"    queue BSM  max:    {bmax:.0f} veh")
            if qm is not None and bm is not None and qm > 0:
                print(f"    BSM capteaza:      {bm / qm * 100:.0f}% din coada reala")

            if srm is not None or sbm is not None:
                print(f"    -- TRANSVERSALA (sec) --")
            if srm   is not None: print(f"    queue real mean:   {srm:.1f} veh   (detector/TraCI)")
            if srmax is not None: print(f"    queue real max:    {srmax:.0f} veh")
            if sbm   is not None: print(f"    queue BSM  mean:   {sbm:.1f} veh   (V2I)")
            if sbmax is not None: print(f"    queue BSM  max:    {sbmax:.0f} veh")
            if srm is not None and sbm is not None and srm > 0:
                print(f"    BSM capteaza:      {sbm / srm * 100:.0f}% din coada reala")

            print(f"    -- DURATA VERDE (control) --")
            if mgm is not None or sgm is not None:
                if mgm is not None: print(f"    verde main mean:   {mgm:.1f} s")
                if sgm is not None: print(f"    verde sec  mean:   {sgm:.1f} s")
                if mgm is not None and sgm is not None:
                    print(f"    split main-sec:    {mgm - sgm:+.1f} s   (>0 = mai mult verde pe bulevard)")
            else:
                print(f"    verde:             neschimbat de la baseline (fara ajustari)")

            if sm   is not None: print(f"    stage mean:        {sm:.2f}")
            if smax is not None: print(f"    stage max:         {smax:.0f}")
            if stp  is not None: print(f"    opriri (distinct): {int(stp)} veh")
            if thr  is not None: print(f"    throughput:        {int(thr)} veh")
            if asp  is not None: print(f"    viteza medie:      {asp * 3.6:.1f} km/h")
            if adj: total_adj += int(adj)

        print(f"\n  Total adjustments aplicate: {total_adj}")

    print(f"\n{'═'*60}\n")


#  GRAFICE DIN .VEC
def plot_vectors(config_name, vec_files, output_dir=None):
    if not HAS_PLOT:
        return
    if not vec_files:
        print("[INFO] Nu există fișiere .vec pentru grafice.")
        return

    all_vectors = {}
    for vf in vec_files:
        vecs = parse_vec(vf)
        for vid, vdata in vecs.items():
            key = f"{vdata['module']}::{vdata['name']}"
            if key not in all_vectors:
                all_vectors[key] = vdata
            else:
                all_vectors[key]["times"]  += vdata["times"]
                all_vectors[key]["values"] += vdata["values"]

    signals_of_interest = [
        "mainGreenDuration",
        "secondaryGreenDuration",
        "currentStage",
        "queueBlvdReal",
        "queueBlvd",
        "queueSecondary",
        "queueSecBsm",
    ]

    by_signal = defaultdict(dict)
    for key, vdata in all_vectors.items():
        for sig in signals_of_interest:
            if sig in vdata["name"] and "rsu" in vdata["module"].lower():
                match = re.search(r'rsu\[(\d+)\]', vdata["module"])
                if match:
                    idx = int(match.group(1))
                    by_signal[sig][idx] = (vdata["times"], vdata["values"])

    if not by_signal:
        print("[INFO] Nu s-au găsit vectori relevanți în .vec.")
        return

    n_plots = len(by_signal)
    fig, axes = plt.subplots(n_plots, 1, figsize=(14, 4 * n_plots), sharex=False)
    if n_plots == 1:
        axes = [axes]

    fig.suptitle(f"Evolutie in timp — {config_name}", fontsize=14, fontweight="bold")
    colors = plt.cm.tab20.colors

    for ax, (sig_name, rsu_data) in zip(axes, by_signal.items()):
        for i, (rsu_idx, (times, values)) in enumerate(sorted(rsu_data.items())):
            label = f"RSU[{rsu_idx}]" + (" [ANCHOR]" if rsu_idx == 8 else "")
            ax.plot(times, values, label=label, color=colors[i % len(colors)],
                    linewidth=1.0, alpha=0.8)
        ax.set_title(sig_name, fontsize=11)
        ax.set_xlabel("Timp simulare (s)")
        ax.set_ylabel("Valoare")
        ax.legend(fontsize=7, ncol=4, loc="upper right")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        out_path = os.path.join(output_dir, f"{config_name}_vectors.png")
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"[INFO] Grafic salvat: {out_path}")
    else:
        plt.show()

    plt.close()


#  COMPARATIE INTRE DOUA SCENARII
def compare_metrics(name_a, m_a, name_b, m_b):
    print(f"\n{'═'*70}")
    print(f"  COMPARATIE: {name_a}  vs  {name_b}")
    print(f"{'═'*70}")

    keys = [
        ("all_waitingTime_mean",   "Waiting time (toate)",    "s",    1.0),
        ("all_travelTime_mean",    "Travel time (toate)",     "s",    1.0),
        ("all_avgSpeed_mean",      "Viteza medie (toate)",    "km/h", 3.6),
        ("all_stops_mean",         "Opriri (toate)",          "",     1.0),
        ("gw_waitingTime_mean",    "Waiting time (WE+EW)",   "s",    1.0),
        ("gw_avgSpeed_mean",       "Viteza medie (WE+EW)",   "km/h", 3.6),
        ("gwe_waitingTime_mean",   "Waiting time (WE)",      "s",    1.0),
        ("gwe_avgSpeed_mean",      "Viteza medie (WE)",      "km/h", 3.6),
        ("co2_total_kg",           "CO2 total",              "kg",   1.0),
    ]

    print(f"\n  {'Metrica':<30} {'':>12} {'':>12} {'Delta':>10}")
    print(f"  {'─'*30} {'─'*12} {'─'*12} {'─'*10}")
    print(f"  {'':30} {name_a:>12} {name_b:>12}")
    print(f"  {'─'*30} {'─'*12} {'─'*12} {'─'*10}")

    for key, label, unit, scale in keys:
        va = m_a.get(key)
        vb = m_b.get(key)
        sa = fmt(va, unit, scale)
        sb = fmt(vb, unit, scale)
        if va is not None and vb is not None and va != 0:
            delta_pct = (vb * scale - va * scale) / (va * scale) * 100
            delta_str = f"{delta_pct:+.1f}%"
        else:
            delta_str = "N/A"
        print(f"  {label:<30} {sa:>12} {sb:>12} {delta_str:>10}")

    adj_a = sum(int(v) for v in m_a.get("rsu_adjustments", {}).values())
    adj_b = sum(int(v) for v in m_b.get("rsu_adjustments", {}).values())
    print(f"\n  {'Total adjustments RSU':<30} {str(adj_a):>12} {str(adj_b):>12}")
    print(f"{'═'*70}\n")


#  MENIU INTERACTIV
def interactive_menu(results_dir):
    while True:
        print("\n" + "═"*60)
        print("  PARSER REZULTATE — Bd. Rebreanu")
        print("═"*60)
        print("  Scenarii disponibile:")
        for i, cfg in enumerate(KNOWN_CONFIGS, 1):
            print(f"    {i:>2}. {cfg}")
        print(f"  {'─'*56}")
        print(f"    c. Compara doua scenarii")
        print(f"    a. Mediaza repetitiile unui scenariu (multi-seed)")
        print(f"    q. Iesire")
        print()

        choice = input("  Selectie: ").strip().lower()

        if choice == "q":
            break
        elif choice == "c":
            run_compare(results_dir)
        elif choice == "a":
            sel = input("  Scenariu de mediat (nr sau nume): ").strip()
            try:
                idx = int(sel) - 1
                cfg = KNOWN_CONFIGS[idx] if 0 <= idx < len(KNOWN_CONFIGS) else sel
            except ValueError:
                cfg = sel
            run_averaged(results_dir, cfg)
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(KNOWN_CONFIGS):
                    run_single(results_dir, KNOWN_CONFIGS[idx])
                else:
                    print("[ERR] Numar invalid.")
            except ValueError:
                if choice in KNOWN_CONFIGS:
                    run_single(results_dir, choice)
                else:
                    print("[ERR] Optiune necunoscuta.")

def _mean_sd(vals):
    clean = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not clean:
        return (None, None)
    mean = sum(clean) / len(clean)
    if len(clean) < 2:
        return (mean, 0.0)
    var = sum((x - mean) ** 2 for x in clean) / (len(clean) - 1)
    return (mean, var ** 0.5)

def average_metrics(mlist):
    means, sds = {}, {}
    keys = set()
    for m in mlist:
        keys.update(m.keys())
    for k in keys:
        vals = [m.get(k) for m in mlist]
        dicts = [v for v in vals if isinstance(v, dict)]
        if dicts:
            submods = set()
            for d in dicts:
                submods.update(d.keys())
            md, sd = {}, {}
            for sm in submods:
                mn, s = _mean_sd([d.get(sm) for d in dicts])
                if mn is not None:
                    md[sm] = mn
                    sd[sm] = s
            means[k] = md
            sds[k] = sd
        else:
            mn, s = _mean_sd(vals)
            means[k] = mn
            sds[k] = s
    return means, sds

def _pm(mean, sd, unit="", dec=1):
    if mean is None:
        return "n/a"
    if sd is None or sd == 0:
        return f"{mean:.{dec}f}{unit}"
    return f"{mean:.{dec}f} ± {sd:.{dec}f}{unit}"

def print_metrics_avg(name, means, sds, n):
    def g(d, k, sub=None):
        v = d.get(k)
        if sub is not None:
            return (v or {}).get(sub)
        return v
    print("\n" + "═"*60)
    print(f"  SCENARIU: {name}   (MEDIAT pe {n} repetitii)")
    print("═"*60)

    print("\n" + " "*22 + "TOATE VEHICULELE")
    print("─"*60)
    print(f"  Vehicule totale:        {_pm(g(means,'all_vehicles_total'), g(sds,'all_vehicles_total'), '', 0)}")
    print(f"  Ruta completata:        {_pm(g(means,'all_routeCompleted_pct'), g(sds,'all_routeCompleted_pct'), '%')}")
    print(f"  Travel time mediu:      {_pm(g(means,'all_travelTime_mean'), g(sds,'all_travelTime_mean'), 's')}")
    print(f"  Waiting time mediu:     {_pm(g(means,'all_waitingTime_mean'), g(sds,'all_waitingTime_mean'), 's')}")
    print(f"  Viteza medie:           {_pm(g(means,'all_avgSpeed_mean'), g(sds,'all_avgSpeed_mean'), ' km/h')}")
    print(f"  Opriri medii/vehicul:   {_pm(g(means,'all_stops_mean'), g(sds,'all_stops_mean'), '', 2)}")
    print(f"  CO2 total:              {_pm(g(means,'co2_total_kg'), g(sds,'co2_total_kg'), ' kg')}")

    print("\n" + " "*21 + "BULEVARD (WE + EW)")
    print("─"*60)
    print(f"  Vehicule bulevard:      {_pm(g(means,'gw_vehicles_total'), g(sds,'gw_vehicles_total'), '', 0)}")
    print(f"  Ruta completata:        {_pm(g(means,'gw_routeCompleted_pct'), g(sds,'gw_routeCompleted_pct'), '%')}")
    print(f"  Travel time mediu:      {_pm(g(means,'gw_travelTime_mean'), g(sds,'gw_travelTime_mean'), 's')}")
    print(f"  Waiting time mediu:     {_pm(g(means,'gw_waitingTime_mean'), g(sds,'gw_waitingTime_mean'), 's')}")
    print(f"  Viteza medie:           {_pm(g(means,'gw_avgSpeed_mean'), g(sds,'gw_avgSpeed_mean'), ' km/h')}")
    print(f"  Opriri medii/vehicul:   {_pm(g(means,'gw_stops_mean'), g(sds,'gw_stops_mean'), '', 2)}")

    print("\n" + " "*20 + "BULEVARD WE (Vest → Est)")
    print("─"*60)
    print(f"  Vehicule WE:            {_pm(g(means,'gwe_vehicles_total'), g(sds,'gwe_vehicles_total'), '', 0)}")
    print(f"  Ruta completata:        {_pm(g(means,'gwe_routeCompleted_pct'), g(sds,'gwe_routeCompleted_pct'), '%')}")
    print(f"  Travel time mediu:      {_pm(g(means,'gwe_travelTime_mean'), g(sds,'gwe_travelTime_mean'), 's')}")
    print(f"  Waiting time mediu:     {_pm(g(means,'gwe_waitingTime_mean'), g(sds,'gwe_waitingTime_mean'), 's')}")
    print(f"  Viteza medie:           {_pm(g(means,'gwe_avgSpeed_mean'), g(sds,'gwe_avgSpeed_mean'), ' km/h')}")
    print(f"  Opriri medii/vehicul:   {_pm(g(means,'gwe_stops_mean'), g(sds,'gwe_stops_mean'), '', 2)}")

    def rsu_idx(mod):
        mt = re.search(r'rsu\[(\d+)\]', mod)
        return int(mt.group(1)) if mt else 99
    
    rsus = sorted(set(list(g(means,'rsu_adjustments',) or {}) +
                      list(g(means,'rsu_queue_mean') or {}) +
                      list(g(means,'rsu_avg_speed') or {})), key=rsu_idx)
    if rsus:
        print("\n" + " "*21 + "RSU — INTERSECTII")
        print("─"*60)
        for rsu in rsus:
            idx = rsu_idx(rsu)
            anchor = " [ANCHOR]" if idx == 8 else ""
            print(f"  RSU[{idx}]{anchor}")
            print(f"    adjustments:       {_pm(g(means,'rsu_adjustments',rsu), g(sds,'rsu_adjustments',rsu), '', 1)}")
            print(f"    coada blvd real:   {_pm(g(means,'rsu_queue_mean',rsu), g(sds,'rsu_queue_mean',rsu), ' veh')}   (detector)")
            print(f"    coada blvd BSM:    {_pm(g(means,'rsu_bsm_mean',rsu), g(sds,'rsu_bsm_mean',rsu), ' veh')}   (V2I)")
            print(f"    coada sec real:    {_pm(g(means,'rsu_sec_real_mean',rsu), g(sds,'rsu_sec_real_mean',rsu), ' veh')}   (detector)")
            print(f"    verde main:        {_pm(g(means,'rsu_main_green_mean',rsu), g(sds,'rsu_main_green_mean',rsu), ' s')}")
            print(f"    verde sec:         {_pm(g(means,'rsu_sec_green_mean',rsu), g(sds,'rsu_sec_green_mean',rsu), ' s')}")
            
            asp_m = g(means, 'rsu_avg_speed', rsu)
            asp_s = g(sds, 'rsu_avg_speed', rsu)
            if asp_m is not None:
                asp_m_kmh = asp_m * 3.6
                asp_s_kmh = (asp_s * 3.6) if asp_s else 0
                print(f"    viteza medie:      {_pm(asp_m_kmh, asp_s_kmh, ' km/h', 1)}")

    print("\n" + "═"*60)

def run_averaged(results_dir, config_name):
    sca_files, vec_files = find_files(results_dir, config_name)
    if not sca_files:
        print(f"[ERR] Nu s-au gasit fisiere .sca pentru '{config_name}' in '{results_dir}'")
        return
    mlist = []
    for sf in sca_files:
        vf = sf[:-4] + ".vec"
        vlist = [vf] if os.path.exists(vf) else []
        mi = extract_metrics(parse_sca(sf))
        attach_bsm_queue(mi, vlist)
        mlist.append(mi)
    n = len(mlist)
    print(f"\n[INFO] {n} repetitii gasite: {[os.path.basename(f) for f in sca_files]}")
    if n < 2:
        print("[WARN] O singura repetitie - media = valoarea ei, SD = 0.")
    means, sds = average_metrics(mlist)

    import io
    buf = io.StringIO(); old = sys.stdout; sys.stdout = buf
    print_metrics_avg(config_name, means, sds, n)
    sys.stdout = old
    txt = buf.getvalue(); print(txt)
    out = os.path.join(results_dir, f"{config_name}_MEDIAT_{n}rep_rezultate.txt")
    with open(out, "w", encoding="utf-8") as f:
        f.write(txt)
    print(f"[INFO] Rezultate mediate salvate in: {out}")

def run_single(results_dir, config_name):
    sca_files, vec_files = find_files(results_dir, config_name)
    if not sca_files:
        print(f"[ERR] Nu s-au găsit fișiere .sca pentru '{config_name}' în '{results_dir}'")
        return

    print(f"\n[INFO] .sca găsite: {[os.path.basename(f) for f in sca_files]}")

    combined_scalars = defaultdict(dict)
    for sf in sca_files:
        sc = parse_sca(sf)
        for mod, stats in sc.items():
            combined_scalars[mod].update(stats)

    m = extract_metrics(combined_scalars)
    attach_bsm_queue(m, vec_files)

    import io
    buf = io.StringIO()
    old_stdout = sys.stdout
    sys.stdout = buf
    print_metrics(config_name, m)
    sys.stdout = old_stdout
    output_text = buf.getvalue()
    print(output_text)
    out_txt = os.path.join(results_dir, f"{config_name}_rezultate.txt")
    with open(out_txt, "w", encoding="utf-8") as f:
        f.write(output_text)
    print(f"[INFO] Rezultate salvate in: {out_txt}")

    if vec_files and HAS_PLOT:
        do_plot = input("  Generează grafice din .vec? [y/N]: ").strip().lower()
        if do_plot == "y":
            plot_vectors(config_name, vec_files, output_dir=results_dir)
    elif vec_files and not HAS_PLOT:
        print("[INFO] matplotlib lipsește — graficele nu pot fi generate.")

def run_compare(results_dir):
    print("\n  Introdu numerele celor două scenarii de comparat:")
    for i, cfg in enumerate(KNOWN_CONFIGS, 1):
        print(f"    {i:>2}. {cfg}")

    try:
        a = int(input("  Scenariu A (nr): ").strip()) - 1
        b = int(input("  Scenariu B (nr): ").strip()) - 1
        if not (0 <= a < len(KNOWN_CONFIGS) and 0 <= b < len(KNOWN_CONFIGS)):
            print("[ERR] Numere invalide.")
            return
    except ValueError:
        print("[ERR] Input invalid.")
        return

    name_a, name_b = KNOWN_CONFIGS[a], KNOWN_CONFIGS[b]

    def load(cfg):
        sca_files, vec_files = find_files(results_dir, cfg)
        if not sca_files:
            print(f"[ERR] Nu s-au găsit fișiere pentru '{cfg}'")
            return None
        combined = defaultdict(dict)
        for sf in sca_files:
            for mod, stats in parse_sca(sf).items():
                combined[mod].update(stats)
        m = extract_metrics(combined)
        attach_bsm_queue(m, vec_files)
        return m

    m_a = load(name_a)
    m_b = load(name_b)
    if m_a and m_b:
        import io
        buf = io.StringIO()
        old_stdout = sys.stdout
        sys.stdout = buf
        print_metrics(name_a, m_a)
        print_metrics(name_b, m_b)
        compare_metrics(name_a, m_a, name_b, m_b)
        sys.stdout = old_stdout
        output_text = buf.getvalue()
        print(output_text)
        out_txt = os.path.join(results_dir, f"Comparatie_{name_a}_vs_{name_b}.txt")
        with open(out_txt, "w", encoding="utf-8") as f:
            f.write(output_text)
        print(f"[INFO] Comparatie salvata in: {out_txt}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parser rezultate Rebreanu")
    parser.add_argument("--dir",    default="results", help="Director cu fișierele .sca/.vec")
    parser.add_argument("--config", default=None,      help="Rulează direct un config (bypass meniu)")
    parser.add_argument("--avg",    action="store_true", help="Mediaza toate repetitiile config-ului (multi-seed)")
    args = parser.parse_args()

    results_dir = args.dir
    if not os.path.isdir(results_dir):
        if os.path.isdir("."):
            results_dir = "."
        print(f"[WARN] Directorul '{args.dir}' nu există, folosesc '{results_dir}'")

    if args.config and args.avg:
        run_averaged(results_dir, args.config)
    elif args.config:
        run_single(results_dir, args.config)
    else:
        interactive_menu(results_dir)