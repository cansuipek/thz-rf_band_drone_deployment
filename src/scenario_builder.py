"""
Scenario construction for the UAV RF/THz deployment optimization.
Parameters that were read from YAML config files.
"""

from __future__ import annotations
import random
import math
from pathlib import Path
from typing import Any

from channel_models import (
    SNR_expected_rf,
    data_rate_rf,
    data_rate_thz,
    p_los_itu1410,
    set_thz_coeff_file_paths,
)

def _section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}

def build_scenario(config: dict[str, Any]) -> tuple[dict[str, Any], list[tuple[float, float]], list[tuple[float, float, float]]]:
    scenario = _section(config, "scenario")
    users_cfg = _section(config, "users")
    area_cfg = _section(config, "area")
    drones_cfg = _section(config, "drones")
    env_cfg = _section(config, "environment")
    rf_cfg = _section(config, "rf")
    thz_cfg = _section(config, "thz")
    constraints_cfg = _section(config, "constraints")

    random_seed = int(scenario.get("random_seed", 0))
    random.seed(random_seed)

    # Area and candidate grid
    x_min = int(area_cfg.get("x_min", 0))
    x_max = int(area_cfg.get("x_max", 1000))
    y_min = int(area_cfg.get("y_min", 0))
    y_max = int(area_cfg.get("y_max", 1000))
    dx = int(area_cfg.get("dx", 100))
    dy = int(area_cfg.get("dy", 100))

    # Users
    num_users = int(users_cfg.get("num_users", 50))
    U = list(range(num_users))
    distribution = str(users_cfg.get("distribution", "uniform")).lower()

    if distribution == "clustered":
        num_clusters = int(users_cfg.get("num_clusters", 5))
        cluster_std = float(users_cfg.get("cluster_std", 90))

        cluster_centers = [
            (
                random.uniform(x_min + 100, x_max - 100),
                random.uniform(y_min + 100, y_max - 100),
            )
            for _ in range(num_clusters)
        ]

        user_xy = []
        for _ in U:
            cx, cy = random.choice(cluster_centers)
            x = random.gauss(cx, cluster_std)
            y = random.gauss(cy, cluster_std)

            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))

            user_xy.append((x, y))
    else:
        user_xy = [
            (random.uniform(x_min, x_max), random.uniform(y_min, y_max))
            for _ in U
        ]

    altitudes = list(drones_cfg.get("altitudes_m", [90, 100]))

    xy_points = [
        (x, y)
        for x in range(x_min, x_max + 1, dx)
        for y in range(y_min, y_max + 1, dy)
    ]

    cand = [(x, y, h) for (x, y) in xy_points for h in altitudes]
    D = list(range(len(cand)))

    print("Number of users:", len(U))
    print("Number of XY points:", len(xy_points))
    print("XY points:", xy_points)
    print("Altitude levels:", altitudes)
    print("Number of candidates:", len(D))
    print(cand)

    # Parameters
    Pt_dbm = float(rf_cfg.get("tx_power_dbm", 20.0))
    gt_dbi = float(rf_cfg.get("tx_gain_dbi", 5.0))
    gr_dbi = float(rf_cfg.get("rx_gain_dbi", 0.0))

    # RF
    f_rf_hz = float(rf_cfg.get("f_rf_hz", 2.0e9))
    Bw_rf_hz = float(rf_cfg.get("bandwidth_hz", 20e6))
    NF_rf_db = float(rf_cfg.get("noise_figure_db", 7.0))

    eta_los_db = float(env_cfg.get("eta_los_db", 1.6))
    eta_nlos_db = float(env_cfg.get("eta_nlos_db", 23.0))

    # THz
    f_thz_hz = float(thz_cfg.get("f_thz", 0.8))
    Bw_thz_hz = float(thz_cfg.get("bandwidth_hz", 0.5e9))
    NF_thz_db = float(thz_cfg.get("noise_figure_db", 10.0))
    thz_model_type = str(thz_cfg.get("model_type", "agnostic"))

    g2uav_coeff_file = str(thz_cfg.get("g2uav_coeff_file", "data/coefficients/G2UAV_model_coefficients.mat"))
    uav2uav_coeff_file = str(thz_cfg.get("uav2uav_coeff_file", "data/coefficients/UAV2UAV_model_coefficients.mat"))
    set_thz_coeff_file_paths(g2uav_coeff_file, uav2uav_coeff_file)

    # Environment parameters for LoS probability
    alpha = float(env_cfg.get("alpha", 0.5))
    beta = float(env_cfg.get("beta", 550))
    gamma = float(env_cfg.get("gamma", 35))

    # A2G
    SNR_RF = {}
    SNR_THz = {}
    R_RF = {}
    R_THz = {}
    pLos_du = {}

    for d in D:
        xd, yd, hd = cand[d]
        for u in U:
            xu, yu = user_xy[u]
            dist = math.sqrt((xd - xu)**2 + (yd - yu)**2 + hd**2)
            dist_horiz_m = math.sqrt((xd - xu)**2 + (yd - yu)**2)
            r_km = dist_horiz_m / 1000.0

            pLos_du[(d, u)] = p_los_itu1410(
                r_km=r_km,
                h_tx=hd,
                h_rx=0,
                alpha=alpha,
                beta=beta,
                gamma=gamma,
            )

            SNR_results = SNR_expected_rf(
                r_km=r_km,
                d_tr=dist,
                h_tx=hd,
                h_rx=0,
                alpha=alpha,
                beta=beta,
                gamma=gamma,
                f_rf_hz=f_rf_hz,
                f_thz_hz=f_thz_hz,
                Bw_rf_hz=Bw_rf_hz,
                Bw_thz_hz=Bw_thz_hz,
                Pt_dbm=Pt_dbm,
                gt_dbi=gt_dbi,
                gr_dbi=gr_dbi,
                NF_rf_db=NF_rf_db,
                NF_thz_db=NF_thz_db,
                eta_los_db=eta_los_db,
                eta_nlos_db=eta_nlos_db,
                is_a2g=True,
                thz_model_type=thz_model_type,
            )

            rate_bps_rf = data_rate_rf(
                pLos_du=pLos_du[(d, u)],
                snr_los_rf=SNR_results["snr_los_rf"],
                snr_nlos_rf=SNR_results["snr_nlos_rf"],
                Bw_hz=Bw_rf_hz,
            )
            rate_bps_thz = data_rate_thz(
                pLos_du=pLos_du[(d, u)],
                snr_thz_lin=SNR_results["snr_thz_lin"],
                Bw_hz=Bw_thz_hz,
            )

            SNR_RF[(d, u)] = SNR_results["snr_rf_exp_db"]
            SNR_THz[(d, u)] = SNR_results["snr_thz_db"]
            R_RF[(d, u)] = rate_bps_rf / 1e6
            R_THz[(d, u)] = rate_bps_thz / 1e6

    # A2A
    SNR_RF_A2A = {}
    SNR_THz_A2A = {}
    R_RF_A2A = {}
    R_THz_A2A = {}
    pLos_ddp = {}
    dist_A2A = {}

    for d in D:
        xd, yd, hd = cand[d]
        for dp in D:
            if dp == d:
                continue

            x2, y2, h2 = cand[dp]
            dist = math.sqrt((xd - x2)**2 + (yd - y2)**2 + (hd - h2)**2)
            dist_A2A[(d, dp)] = dist

            dist_horiz_m = math.sqrt((xd - x2)**2 + (yd - y2)**2)
            r_km = dist_horiz_m / 1000.0

            pLos_ddp[(d, dp)] = p_los_itu1410(
                r_km=r_km,
                h_tx=hd,
                h_rx=h2,
                alpha=alpha,
                beta=beta,
                gamma=gamma,
            )

            SNR_results_a2a = SNR_expected_rf(
                r_km=r_km,
                d_tr=dist,
                h_tx=hd,
                h_rx=h2,
                alpha=alpha,
                beta=beta,
                gamma=gamma,
                f_rf_hz=f_rf_hz,
                f_thz_hz=f_thz_hz,
                Bw_rf_hz=Bw_rf_hz,
                Bw_thz_hz=Bw_thz_hz,
                Pt_dbm=Pt_dbm,
                gt_dbi=gt_dbi,
                gr_dbi=gr_dbi,
                NF_rf_db=NF_rf_db,
                NF_thz_db=NF_thz_db,
                eta_los_db=eta_los_db,
                eta_nlos_db=eta_nlos_db,
                is_a2g=False,
                thz_model_type=thz_model_type,
            )

            rate_bps_rf_a2a = data_rate_rf(
                pLos_du=pLos_ddp[(d, dp)],
                snr_los_rf=SNR_results_a2a["snr_los_rf"],
                snr_nlos_rf=SNR_results_a2a["snr_nlos_rf"],
                Bw_hz=Bw_rf_hz,
            )
            rate_bps_thz_a2a = data_rate_thz(
                pLos_du=pLos_ddp[(d, dp)],
                snr_thz_lin=SNR_results_a2a["snr_thz_lin"],
                Bw_hz=Bw_thz_hz,
            )

            SNR_RF_A2A[(d, dp)] = SNR_results_a2a["snr_rf_exp_db"]
            SNR_THz_A2A[(d, dp)] = SNR_results_a2a["snr_thz_db"]
            R_RF_A2A[(d, dp)] = rate_bps_rf_a2a / 1e6
            R_THz_A2A[(d, dp)] = rate_bps_thz_a2a / 1e6

    data = dict(
        D=D,
        U=U,
        Cost_master=float(drones_cfg.get("cost_master", 40.0)),
        Cost_slave=float(drones_cfg.get("cost_slave", 6.0)),

        Rmin=float(constraints_cfg.get("Rmin", 5)),
        SNR_A2G_RF_min=float(constraints_cfg.get("SNR_A2G_RF_min", 5.0)),
        SNR_A2G_THz_min=float(constraints_cfg.get("SNR_A2G_THz_min", 5.0)),
        SNR_A2A_RF_min=float(constraints_cfg.get("SNR_A2A_RF_min", 5.0)),
        SNR_A2A_THz_min=float(constraints_cfg.get("SNR_A2A_THz_min", 5.0)),

        pLoS_THz_min=float(constraints_cfg.get("pLoS_THz_min", 0.8)),

        SNR_RF=SNR_RF,
        SNR_THz=SNR_THz,
        R_RF=R_RF,
        R_THz=R_THz,
        pLos_du=pLos_du,

        SNR_RF_A2A=SNR_RF_A2A,
        SNR_THz_A2A=SNR_THz_A2A,
        R_RF_A2A=R_RF_A2A,
        R_THz_A2A=R_THz_A2A,
        pLos_ddp=pLos_ddp,
        dist_A2A=dist_A2A,
    )

    return data, user_xy, cand
