"""
Gurobi optimization model for joint drone deployment and RF/THz band selection.
This file contains the decision variables, objective, and constraints.
"""

import math
import gurobipy as gp
from gurobipy import GRB

def solve_drone_deployment_gurobi(data, time_limit_s=math.inf, mip_gap=0.1, verbose=True):
    #read the dictionary "data". Build the gurobi model. Solve it. Return the solution.
    D = data["D"]
    U = data["U"]

    Cost_master = float(data["Cost_master"])
    Cost_slave  = float(data["Cost_slave"])

    Rmin = float(data["Rmin"])
    SNR_A2G_RF_min  = float(data["SNR_A2G_RF_min"])
    SNR_A2G_THz_min = float(data["SNR_A2G_THz_min"])
    SNR_A2A_RF_min  = float(data["SNR_A2A_RF_min"])
    SNR_A2A_THz_min = float(data["SNR_A2A_THz_min"])

    pLoS_THz_min = float(data["pLoS_THz_min"])

    SNR_RF   = data["SNR_RF"]
    SNR_THz  = data["SNR_THz"]
    R_RF     = data["R_RF"]
    R_THz    = data["R_THz"]
    pLos_du  = data["pLos_du"]

    SNR_RF_A2A  = data["SNR_RF_A2A"]
    SNR_THz_A2A = data["SNR_THz_A2A"]
    R_RF_A2A    = data["R_RF_A2A"]
    R_THz_A2A   = data["R_THz_A2A"]
    pLos_ddp    = data["pLos_ddp"]
    dist_A2A    = data["dist_A2A"]

    # Big-M must be large enough to relax the constraint when g=0:
    min_snr_a2a = min(
        min(SNR_RF_A2A[(d, dp)], SNR_THz_A2A[(d, dp)])
        for d in D for dp in D if dp != d
    )

    M_snr_a2a = (max(SNR_A2A_RF_min, SNR_A2A_THz_min) - min_snr_a2a) + 1.0
    M_cap = (len(U) * 100) + 1

    m = gp.Model("drone_deployment_discrete")
    m.Params.TimeLimit = time_limit_s
    m.Params.MIPGap = mip_gap #Stop when the solution is within mip_gap of optimal.
    # Decision variables
    # addVars(): Create one variable for each element of D.
    z = m.addVars(D, vtype=GRB.BINARY, name="z")        # drone deployed or not
    w = m.addVars(D, vtype=GRB.BINARY, name="w")       # deployed drone master or not
    x = m.addVars(D, U, vtype=GRB.BINARY, name="x")    # user u served by drone d or not

    t_air   = m.addVar(vtype=GRB.BINARY, name="t_air")    # A2G tech selector  0-RF, 1-THz
    t_space = m.addVar(vtype=GRB.BINARY, name="t_space") # S2A tech selector 0-RF, 1-THz

    a = m.addVars(D, U, vtype=GRB.BINARY, name="a")  # a = x AND t_air=0 , for SNR and data rate constraints
    b = m.addVars(D, U, vtype=GRB.BINARY, name="b")  # b = x AND t_air=1

    #actual rate from drone d to user u (in Mbps)
    r = m.addVars(D, U, vtype=GRB.CONTINUOUS, lb=0.0, name="r")

    # share of RF/THz "bandwidth/time" that drone d gives to user u
    theta_RF  = m.addVars(D, U, vtype=GRB.CONTINUOUS, lb=0.0, ub=1.0, name="theta_RF")
    theta_THz = m.addVars(D, U, vtype=GRB.CONTINUOUS, lb=0.0, ub=1.0, name="theta_THz")

    gRF  = m.addVars(D, D, vtype=GRB.BINARY, name="gRF")   # A2A RF link 1 or 0
    gTHz = m.addVars(D, D, vtype=GRB.BINARY, name="gTHz")   # A2A THz link 1 or 0

    # Actual A2A forwarded flow from drone d to drone dp, in Mbps
    fA2A = m.addVars(D, D, vtype=GRB.CONTINUOUS, lb=0.0, name="fA2A")

    # A2A bandwidth/time share used on each A2A link
    phi_A2A_RF = m.addVars(D, D, vtype=GRB.CONTINUOUS, lb=0.0, ub=1.0, name="phi_A2A_RF")
    phi_A2A_THz = m.addVars(D, D, vtype=GRB.CONTINUOUS, lb=0.0, ub=1.0, name="phi_A2A_THz")

    # Slave indicator s_d = z_d - w_d (binary because w<=z)
    s = m.addVars(D, vtype=GRB.BINARY, name="s_slave")

    m.setObjective(
        gp.quicksum(w[d] * Cost_master + (z[d] - w[d]) * Cost_slave for d in D),
        GRB.MINIMIZE
    )
    # Constraints
    # Deployment constraints:
    # At least one master for deployment: sum_d z_d w_d >= 1  equivalently sum_d w_d >= 1 since w_d <= z_d
    m.addConstr(
        gp.quicksum(w[d] for d in D) >= 1,
        name="at_least_one_master"
    )
    m.addConstrs((w[d] <= z[d] for d in D), name="master_only_if_deployed")
    m.addConstrs((s[d] == z[d] - w[d] for d in D), name="slave_indicator")
    # Each user served by at least one drone and only if deployed.
    m.addConstrs((gp.quicksum(x[d, u] for d in D) >= 1 for u in U), name="at_least_one_serving_drone")
    m.addConstrs((x[d, u] <= z[d] for d in D for u in U), name="no_connection_with_non_deployed_drone")
    eps = 1e-3
    for d in D:
        for u in U:
            # a = x AND (1 - t_air)   deployed and RF
            m.addConstr(a[d,u] <= x[d,u])
            m.addConstr(a[d,u] <= 1 - t_air)
            m.addConstr(a[d,u] >= x[d,u] - t_air)
            # b = x * t_air    deployed and THz
            m.addConstr(b[d,u] <= x[d,u])
            m.addConstr(b[d,u] <= t_air)
            m.addConstr(b[d,u] >= x[d,u] + t_air - 1)

            m.addConstr(a[d,u] + b[d,u] == x[d,u], name=f"splitTech_{d}_{u}")

            snr_selected = SNR_RF[d,u]*a[d,u] + SNR_THz[d,u]*b[d,u]
            m.addConstr(snr_selected >= SNR_A2G_RF_min*a[d,u] + SNR_A2G_THz_min*b[d,u])
            # With bandwidth splitting:
            # r_du <= (full-band RF rate)*theta_RF + (full-band THz rate)*theta_THz
            m.addConstr(
                r[d, u] <= R_RF[d, u] * theta_RF[d, u]
                         + R_THz[d, u] * theta_THz[d, u],
                name=f"link_rate_cap_split_{d}_{u}"
            )
            # share only if link is active with that tech
            m.addConstr(theta_RF[d, u]  <= a[d, u], name=f"thetaRF_le_a_{d}_{u}")
            m.addConstr(theta_THz[d, u] <= b[d, u], name=f"thetaTHz_le_b_{d}_{u}")
            # no assignment without traffic
            m.addConstr(
                r[d, u] >= eps * x[d, u],
                name=f"min_rate_if_assigned_{d}_{u}"
            )
    # Common gateway bandwidth sharing:
    # A gateway's total bandwidth is shared between:
    #   1) direct A2G links from gateway to users
    #   2) incoming A2A links from relay/slave drones
    for d in D:
        m.addConstr(
            gp.quicksum(theta_RF[d, u] for u in U)
            + gp.quicksum(phi_A2A_RF[d, dp] for dp in D if dp != d)   # outgoing (slave)
            + gp.quicksum(phi_A2A_RF[dp, d] for dp in D if dp != d)   # incoming (master)
            <= 1.0,
            name=f"joint_RF_budget_{d}"
        )
        m.addConstr(
            gp.quicksum(theta_THz[d, u] for u in U)
            + gp.quicksum(phi_A2A_THz[d, dp] for dp in D if dp != d)  # outgoing (slave)
            + gp.quicksum(phi_A2A_THz[dp, d] for dp in D if dp != d)  # incoming (master)
            <= 1.0,
            name=f"joint_THz_budget_{d}"
        )
    for u in U:
        m.addConstr(
            gp.quicksum(r[d, u] for d in D) >= Rmin,
            name=f"RateMin_{u}"
        )
    # A2A Master–Slave constraints
    # No self-links
    for d in D:
        m.addConstr(gRF[d, d] == 0,  name=f"no_self_RF_{d}")
        m.addConstr(gTHz[d, d] == 0, name=f"no_self_THz_{d}")
    # t_space = 0 the n only RF A2A links allowed  -all gTHz must be 0
    # t_space = 1 then only THz A2A links allowed -all gRF  must be 0
    for d in D:
        for dp in D:
            if dp == d:
                continue
            # gRF[d,dp]  can only be 1 when t_space = 0
            m.addConstr(gRF[d, dp]  <= 1 - t_space, name=f"gRF_tech_global_{d}_{dp}")
            # gTHz[d,dp] can only be 1 when t_space = 1
            m.addConstr(gTHz[d, dp] <= t_space,  name=f"gTHz_tech_global_{d}_{dp}")

    # Only deployed nodes can have A2A links, g links exist only from a SLAVE d to a MASTER d' and both deployed.
    for d in D:
        for dp in D:
            if dp == d:
                continue
            for gvar, tech in [(gRF, "RF"), (gTHz, "THz")]:
                m.addConstr(gvar[d, dp] <= z[d],        name=f"g{tech}_le_z_from_{d}_{dp}")
                m.addConstr(gvar[d, dp] <= (1 - w[d]),  name=f"g{tech}_le_slave_from_{d}_{dp}")
                m.addConstr(gvar[d, dp] <= w[dp],       name=f"g{tech}_le_master_to_{d}_{dp}")
                m.addConstr(gvar[d, dp] <= z[dp],       name=f"g{tech}_le_z_to_{d}_{dp}")
    # Each slave connects to at least one master:
    for d in D:
        m.addConstr(
            gp.quicksum((gRF[d, dp] + gTHz[d, dp]) for dp in D if dp != d) >= s[d],
            name=f"slave_connects_{d}"
        )
    # A2A bandwidth sharing and actual forwarded flow
    # No self-flow and no self-bandwidth share
    for d in D:
        m.addConstr(fA2A[d, d] == 0, name=f"no_self_flow_A2A_{d}")
        m.addConstr(phi_A2A_RF[d, d] == 0, name=f"no_self_phi_RF_A2A_{d}")
        m.addConstr(phi_A2A_THz[d, d] == 0, name=f"no_self_phi_THz_A2A_{d}")
    for d in D:
        for dp in D:
            if dp == d:
                continue
            # A2A flow cannot exceed the shared-capacity of the selected A2A link
            m.addConstr(
                fA2A[d, dp] <= R_RF_A2A[d, dp] * phi_A2A_RF[d, dp]
                            + R_THz_A2A[d, dp] * phi_A2A_THz[d, dp],
                name=f"A2A_flow_capacity_with_sharing_{d}_{dp}"
            )
            # RF share can only be positive if the RF A2A link is selected
            m.addConstr(
                phi_A2A_RF[d, dp] <= gRF[d, dp],
                name=f"phi_A2A_RF_le_gRF_{d}_{dp}"
            )
            # THz share can only be positive if the THz A2A link is selected
            m.addConstr(
                phi_A2A_THz[d, dp] <= gTHz[d, dp],
                name=f"phi_A2A_THz_le_gTHz_{d}_{dp}"
            )
    # Flow conservation for slaves:
    # Traffic served by a slave must be forwarded to master/s
    for d in D:
        slave_access_load = gp.quicksum(r[d, u] for u in U)
        slave_forwarded_load = gp.quicksum(fA2A[d, dp] for dp in D if dp != d)
        # If d is a slave, w[d] = 0, so these become equality.
        # If d is a master, w[d] = 1, so they are relaxed.
        m.addConstr(
            slave_forwarded_load >= slave_access_load - M_cap * w[d],
            name=f"slave_flow_conservation_lower_{d}"
        )
        m.addConstr(
            slave_forwarded_load <= slave_access_load + M_cap * w[d],
            name=f"slave_flow_conservation_upper_{d}"
        )
    # A2G THz LoS gating
    for d in D:
        for u in U:
            m.addConstr(
                pLos_du[(d, u)] >= pLoS_THz_min * b[d, u],
                name=f"A2G_THz_LoS_gate_{d}_{u}"
            )
    # A2A SNR requirement:
    for d in D:
        for dp in D:
            if dp == d:
                continue
            # RF
            m.addConstr(
                SNR_RF_A2A[d, dp] + M_snr_a2a * (1 - gRF[d, dp]) >= SNR_A2A_RF_min,
                name=f"A2A_SNR_RF_{d}_{dp}"
            )
            # THz
            m.addConstr(
                SNR_THz_A2A[d, dp] + M_snr_a2a * (1 - gTHz[d, dp]) >= SNR_A2A_THz_min,
                name=f"A2A_SNR_THz_{d}_{dp}"
            )
    # A2A THz LoS gating:
    for d in D:
        for dp in D:
            if dp == d:
                continue
            m.addConstr(
                pLos_ddp[d, dp] >= pLoS_THz_min * gTHz[d, dp],
                name=f"A2A_THz_LoS_gate_{d}_{dp}"
            )
    # Solve
    m.optimize()
    status = m.Status
    if status == GRB.INFEASIBLE:
        m.computeIIS()
        m.write("model.ilp")
        return {
            "status": status,
            "status_str": "INFEASIBLE",
            "iis_file": "model.ilp"
        }
    if status in [GRB.UNBOUNDED, GRB.INF_OR_UNBD]:
        return {
            "status": status,
            "status_str": "UNBOUNDED/INF_OR_UNBD"
        }
    # If time limit, only read solution if an incumbent exists
    if status == GRB.TIME_LIMIT and m.SolCount == 0:
        return {
            "status": status,
            "status_str": "TIME_LIMIT_NO_SOLUTION"
        }
    sol = {
        "status": status,
        "status_str": {
            GRB.OPTIMAL: "OPTIMAL",
            GRB.TIME_LIMIT: "TIME_LIMIT"
        }.get(status, str(status)),
        "obj": m.ObjVal
    }
    sol["t_air"] = int(round(t_air.X))
    sol["t_space"] = int(round(t_space.X))
    sol["deployed"] = [d for d in D if z[d].X > 0.5]
    sol["masters"]  = [d for d in D if w[d].X > 0.5]
    sol["slaves"]   = [d for d in D if s[d].X > 0.5]
    user_rates = {}
    for u in U:
        user_rates[u] = sum(r[d, u].X for d in D)
    sol["user_rates"] = user_rates

    drone_rates = {}
    for d in D:
        drone_rates[d] = sum(r[d, u].X for u in U)
    sol["drone_rates"] = drone_rates
    # Assignments: for each user, list of serving drones
    assign = {}
    for u in U:
        serving_drones = []
        for d in D:
            if x[d, u].X > 0.5:
                serving_drones.append(d)
        if serving_drones:
            assign[u] = serving_drones
    sol["assignment"] = assign

    # A2A links
    links_rf = []
    links_thz = []
    for d in D:
        for dp in D:
            if d == dp:
                continue
            if gRF[d, dp].X > 0.5:
                links_rf.append((d, dp))
            if gTHz[d, dp].X > 0.5:
                links_thz.append((d, dp))
    sol["a2a_rf_links"] = links_rf
    sol["a2a_thz_links"] = links_thz
    sol["r_du"] = {(d,u): r[d,u].X for d in D for u in U}
    sol["a2a_flow"] = {
    (d, dp): fA2A[d, dp].X
    for d in D for dp in D
    if d != dp and fA2A[d, dp].X > 1e-6
}
    sol["a2a_phi_rf"] = {
        (d, dp): phi_A2A_RF[d, dp].X
        for d in D for dp in D
        if d != dp and phi_A2A_RF[d, dp].X > 1e-6
    }
    sol["a2a_phi_thz"] = {
        (d, dp): phi_A2A_THz[d, dp].X
        for d in D for dp in D
        if d != dp and phi_A2A_THz[d, dp].X > 1e-6
    }
    return sol
