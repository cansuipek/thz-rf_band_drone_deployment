"""
Channel, path-loss, SNR, and data-rate helper functions.
"""

import math
import warnings
from functools import lru_cache
import numpy as np
from scipy.io import loadmat

G2UAV_COEFF_FILE = "G2UAV_model_coefficients.mat"
UAV2UAV_COEFF_FILE = "UAV2UAV_model_coefficients.mat"


def set_thz_coeff_file_paths(g2uav_coeff_file: str, uav2uav_coeff_file: str) -> None:
    global G2UAV_COEFF_FILE, UAV2UAV_COEFF_FILE
    G2UAV_COEFF_FILE = str(g2uav_coeff_file)
    UAV2UAV_COEFF_FILE = str(uav2uav_coeff_file)
    load_thz_coeffs.cache_clear()


def p_los_itu1410(
    r_km: float,          # horizontal distance r from transmitter to obstacle (km)
    h_tx: float,          # transmitter height (m)
    h_rx: float,          # receiver height (m)
    alpha: float,         # α land area fraction covered by buildings
    beta: float,          # β building density (buildings / km^2)
    gamma: float,         # γ Rayleigh height parameter (m) - the mode
):
    if r_km <= 0:
        return 1 #if the horizontal distance is zero, assume LoS
    # expected number of buildings for r km
    br = int(math.floor(r_km * math.sqrt(alpha * beta)))
    if br <= 0:
        return 1 #if there are no buildings, assume LoS
    sum_weighted = 0.0
    prob_up_to_m = 1.0  # to update with each m
    log_prob = 0.0      # error if not initialized
    for m in range(br):
        di = (m + 0.5) * (r_km / br)
        # height of ray at distance di
        h = h_tx - ((h_tx - h_rx) / r_km) * di

        #CDF of Rayleigh at h
        # probability that a building is smaller than hight hi
        cdf = 1.0 - math.exp(-(h * h) / (2*gamma * gamma))
        cdf = min(max(cdf, 1e-12), 1.0 - 1e-12) #to avoid log(0)

        log_prob += math.log(cdf)
        prob_up_to_m = math.exp(log_prob)
        # 2*m+1 is the weight depending on the distance from the transmitter
        sum_weighted += (2 * m + 1) * prob_up_to_m

    p = sum_weighted / (br * br)
    #just in case
    if p <= 0.0:
        p = 0.0
    elif p > 1.0:
        p = 1.0
    return p

def noise_power_dbm(Bw_hz: float, NF_db: float) -> float:
    #Bw in Hz, noise figure in dB
    pn_db = -174.0 + 10.0 * math.log10(Bw_hz) + NF_db
    return pn_db

def path_loss_rf_db(d_tr: float, f_hz: float, eta_db: float) -> float:
    c = 3e8
    pl_db = 20 * math.log10((4.0 * math.pi * f_hz* d_tr) / c)
    pl_db += eta_db
    return pl_db

@lru_cache(maxsize=None)
def load_thz_coeffs(coeff_file: str):
    S = loadmat(coeff_file, squeeze_me=True, struct_as_record=False)
    if "coeffs" not in S:
        raise ValueError(f'"{coeff_file}" does not contain a struct named "coeffs".')
    return S["coeffs"]

def _polyval(coeffs, x: float) -> float:
    return float(np.polyval(np.asarray(coeffs, dtype=float).ravel(), x))

def trans_to_pl_local(trans: float, d_km: float, f_thz: float) -> float:
    f_hz = f_thz * 1e12
    d_m = d_km * 1000.0
    c = 299792458.0

    if trans < 0:
        raise ValueError("Transmittance became negative, which is not physical.")

    trans = max(float(trans), np.finfo(float).tiny)

    abs_coeff = (-math.log(trans)) / d_m
    abs_loss_db = abs_coeff * d_m * 10.0 * math.log10(math.e)

    pl_spread_db = (
        20.0 * math.log10(d_m)
        + 20.0 * math.log10(f_hz / 1e6)
        + 20.0 * math.log10(4.0 * math.pi * 1e6 / c)
    )

    return pl_spread_db + abs_loss_db

def g2uav_estimated_pathloss_py(
    model_type: str,
    theta_deg: float,
    d_km: float,
    f_thz: float,
    coeff_file: str = "G2UAV_model_coefficients.mat",
) -> float:
    if model_type not in ("agnostic", "adaptive"):
        raise ValueError("model_type must be 'agnostic' or 'adaptive'")
    if not (0.0 <= theta_deg <= 90.0):
        raise ValueError("theta_deg must be in [0, 90]")
    if d_km <= 0:
        raise ValueError("d_km must be positive")
    if f_thz <= 0:
        raise ValueError("f_thz must be positive")

    if d_km < 0.001 or d_km > 2.5:
        warnings.warn(f"G2UAV: d_km={d_km:.6f} is outside intended range [0.001, 2.5] km.")
    if f_thz < 0.79 or f_thz > 0.83:
        warnings.warn(f"G2UAV: f_THz={f_thz:.6f} is outside intended range [0.79, 0.83] THz.")

    coeffs = load_thz_coeffs(coeff_file)

    if model_type == "adaptive":
        theta_grid = np.asarray(coeffs.theta_adaptive.theta_vec, dtype=float).ravel()
        idx_theta = int(np.argmin(np.abs(theta_grid - theta_deg)))

        lam_all = getattr(coeffs.theta_adaptive, "lambda")
        lam = np.asarray(lam_all[idx_theta], dtype=float).ravel()

        b_est = float(np.polyval(lam, f_thz))
        tau_est = math.exp(b_est * d_km)

    else:  # agnostic
        lambda_h = np.asarray(coeffs.theta_agnostic.lambda_h, dtype=float).ravel()
        lambda_v = np.asarray(coeffs.theta_agnostic.lambda_v, dtype=float).ravel()

        kh = float(np.polyval(lambda_h, f_thz))
        kv = float(np.polyval(lambda_v, f_thz))

        # d_h = d * sin(theta), d_v = d * cos(theta)
        d_h = d_km * math.sin(math.radians(theta_deg))
        d_v = d_km * math.cos(math.radians(theta_deg))

        tau_est = math.exp(kh * d_h + kv * d_v)

    tau_est = max(tau_est, np.finfo(float).tiny)
    return trans_to_pl_local(tau_est, d_km, f_thz)

def uav2uav_estimated_pathloss_py(
    model_type: str,
    l_km: float,
    theta_deg: float,
    d_km: float,
    f_thz: float,
    coeff_file: str = "UAV2UAV_model_coefficients.mat",
) -> float:
    if model_type not in ("agnostic", "adaptive"):
        raise ValueError("model_type must be 'agnostic' or 'adaptive'")
    if l_km <= 0:
        raise ValueError("l_km must be positive")
    if not (0.0 <= theta_deg <= 90.0):
        raise ValueError("theta_deg must be in [0, 90]")
    if d_km <= 0:
        raise ValueError("d_km must be positive")
    if f_thz <= 0:
        raise ValueError("f_thz must be positive")

    if l_km < 0.03 or l_km > 2.0:
        warnings.warn(f"UAV2UAV: l_km={l_km:.6f} is outside intended range [0.03, 2.0] km.")
    if d_km < 0.01 or d_km > 2.43:
        warnings.warn(f"UAV2UAV: d_km={d_km:.6f} is outside intended range [0.01, 2.43] km.")
    if f_thz < 0.79 or f_thz > 0.83:
        warnings.warn(f"UAV2UAV: f_THz={f_thz:.6f} is outside intended range [0.79, 0.83] THz.")

    coeffs = load_thz_coeffs(coeff_file)

    # theta=0 -> vertical, theta=90 -> horizontal
    d_h = d_km * math.sin(math.radians(theta_deg))
    d_v = d_km * math.cos(math.radians(theta_deg))

    if model_type == "adaptive":
        theta_grid = np.asarray(coeffs.theta_adaptive.theta_vec, dtype=float).ravel()
        idx_theta = int(np.argmin(np.abs(theta_grid - theta_deg)))

        lam_all = getattr(coeffs.theta_adaptive, "lambda")
        lam = np.asarray(lam_all[idx_theta], dtype=float).ravel()

        b2_all = np.asarray(coeffs.theta_adaptive.b2, dtype=float).ravel()
        b2_est = float(b2_all[idx_theta])

        a2_est = float(np.polyval(lam, f_thz))
        slope_est = a2_est * math.exp(b2_est * l_km)
        tau_est = math.exp(slope_est * d_km)

    else:  # agnostic
        lambda_h = np.asarray(coeffs.theta_agnostic.lambda_h, dtype=float).ravel()
        lambda_v = np.asarray(coeffs.theta_agnostic.lambda_v, dtype=float).ravel()
        b2_h = float(coeffs.theta_agnostic.b2_h)
        b2_v = float(coeffs.theta_agnostic.b2_v)

        Lambda_h = float(np.polyval(lambda_h, f_thz))
        Lambda_v = float(np.polyval(lambda_v, f_thz))

        tau_est = math.exp(
            Lambda_h * math.exp(b2_h * l_km) * d_h
            + Lambda_v * math.exp(b2_v * l_km) * d_v
        )

    tau_est = float(np.clip(tau_est, np.finfo(float).tiny, 1.0))
    return trans_to_pl_local(tau_est, d_km, f_thz)

def pathloss_thz_a2g_geom_db(
    d_tr: float,
    h_tx: float,
    h_rx: float,
    f_thz: float,
    coeff_file: str = "G2UAV_model_coefficients.mat",
    model_type: str = "agnostic",
) -> float:
    if d_tr <= 0:
        return 0.0

    vert_m = abs(h_tx - h_rx)
    horiz_m = math.sqrt(max(d_tr**2 - vert_m**2, 0.0))

    # zenith angle: 0° vertical, 90° horizontal
    theta_deg = math.degrees(math.atan2(horiz_m, vert_m)) if vert_m > 0 else 90.0
    d_km = d_tr / 1000.0

    return g2uav_estimated_pathloss_py(
        model_type=model_type,
        theta_deg=theta_deg,
        d_km=d_km,
        f_thz=f_thz,
        coeff_file=coeff_file,
    )

def pathloss_thz_a2a_geom_db(
    d_tr: float,
    h_tx: float,
    h_rx: float,
    f_thz: float,
    coeff_file: str = "UAV2UAV_model_coefficients.mat",
    model_type: str = "agnostic",
) -> float:
    if d_tr <= 0:
        return 0.0

    vert_m = abs(h_tx - h_rx)
    horiz_m = math.sqrt(max(d_tr**2 - vert_m**2, 0.0))

    # zenith angle: 0° vertical, 90° horizontal
    theta_deg = math.degrees(math.atan2(horiz_m, vert_m)) if vert_m > 0 else 90.0
    l_km = min(h_tx, h_rx) / 1000.0
    d_km = d_tr / 1000.0

    return uav2uav_estimated_pathloss_py(
        model_type=model_type,
        l_km=l_km,
        theta_deg=theta_deg,
        d_km=d_km,
        f_thz=f_thz,
        coeff_file=coeff_file,
    )

def SNR_linear(
    p_tx_dbm: float,
    g_tx_db: float,
    g_rx_db: float,
    pl_db: float,
    pn_db: float,
) -> float:
    snr_db = p_tx_dbm + g_tx_db + g_rx_db - pl_db - pn_db
    return 10 ** (snr_db / 10)

def SNR_expected_rf(
    r_km: float,
    d_tr: float,
    h_tx: float,
    h_rx: float,
    alpha: float,
    beta: float,
    gamma: float,
    f_rf_hz: float,
    f_thz_hz: float,
    Bw_rf_hz: float,
    Bw_thz_hz: float,
    Pt_dbm: float,
    gt_dbi: float,
    gr_dbi: float,
    NF_rf_db: float,
    NF_thz_db: float,
    eta_los_db: float,
    eta_nlos_db: float,
    is_a2g: bool,
    thz_model_type: str = "agnostic",
) -> dict:
    #RF
    p_los = p_los_itu1410(r_km, h_tx, h_rx, alpha, beta, gamma)
    Pn_db_rf = noise_power_dbm(Bw_rf_hz, NF_rf_db)

    pl_los_db_rf = path_loss_rf_db(d_tr, f_rf_hz, eta_los_db)
    pl_nlos_db_rf = path_loss_rf_db(d_tr, f_rf_hz, eta_nlos_db)

    snr_los_rf = SNR_linear(Pt_dbm, gt_dbi, gr_dbi, pl_los_db_rf, Pn_db_rf)
    snr_nlos_rf = SNR_linear(Pt_dbm, gt_dbi, gr_dbi, pl_nlos_db_rf, Pn_db_rf)

    snr_rf_exp = p_los * snr_los_rf + (1.0 - p_los) * snr_nlos_rf
    snr_rf_exp_db = 10.0 * math.log10(snr_rf_exp) if snr_rf_exp > 0 else -100.0
    #THz
    Pn_db_thz = noise_power_dbm(Bw_thz_hz, NF_thz_db)

    Pt_thz_dbm = 20.0
    gt_thz_dbi = 30.0
    gr_thz_dbi = 25.0

    if is_a2g:
        pl_db_thz = pathloss_thz_a2g_geom_db(
            d_tr=d_tr,
            h_tx=h_tx,
            h_rx=h_rx,
            f_thz=f_thz_hz,
            coeff_file=G2UAV_COEFF_FILE,
            model_type=thz_model_type,
        )
    else:
        pl_db_thz = pathloss_thz_a2a_geom_db(
            d_tr=d_tr,
            h_tx=h_tx,
            h_rx=h_rx,
            f_thz=f_thz_hz,
            coeff_file=UAV2UAV_COEFF_FILE,
            model_type=thz_model_type,
        )

    snr_thz = SNR_linear(Pt_thz_dbm, gt_thz_dbi, gr_thz_dbi, pl_db_thz, Pn_db_thz)
    snr_thz_db = 10.0 * math.log10(snr_thz) if snr_thz > 0 else -100.0

    return {
        "p_los": p_los,
        "snr_los_rf": snr_los_rf,
        "snr_nlos_rf": snr_nlos_rf,
        "snr_rf_exp_lin": snr_rf_exp,
        "snr_thz_lin": snr_thz,
        "snr_rf_exp_db": snr_rf_exp_db,
        "snr_thz_db": snr_thz_db,
    }

def data_rate_rf(pLos_du: float, snr_los_rf: float, snr_nlos_rf:float, Bw_hz: float) -> float:
    rate_bps_rf = pLos_du * Bw_hz * math.log2(1 + snr_los_rf) + (1 - pLos_du) * Bw_hz * math.log2(1 + snr_nlos_rf)
    return rate_bps_rf

def data_rate_thz(pLos_du: float, snr_thz_lin: float, Bw_hz: float) -> float:
    rate_bps_thz = pLos_du * Bw_hz * math.log2(1 + snr_thz_lin)
    return rate_bps_thz
