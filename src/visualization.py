"""
Solution reporting and deployment visualization.
"""
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
def analyze_and_visualize_solution(sol, data, user_xy, cand, filename="drone_deployment.pdf"):
    status = sol.get("status_str", "")
    if status not in ("OPTIMAL", "TIME_LIMIT"):
        print(f"No feasible solution to analyze (status = {status}).")
        return
    print("\nDETAILED LINK PERFORMANCE")
    t_air_val = sol.get("t_air", 0)
    tech_str = "THz" if t_air_val == 1 else "RF"

    a2g_rf_color  = "mediumseagreen"
    a2a_rf_color  = "darkgreen"
    a2g_thz_color = "royalblue"
    a2a_thz_color = "midnightblue"
    a2g_color = a2g_thz_color if t_air_val == 1 else a2g_rf_color

    assign_raw = sol.get("assignment", {})
    assign = {}
    for u, val in assign_raw.items():
        drones = list(val) if isinstance(val, (list, tuple, set)) else [val]
        if drones:
            assign[u] = drones

    # Print A2G pLoS, SNR, Rate 
    if not assign:
        print("No users are assigned to any drone.")
    else:
        print("\nA2G LINK PERFORMANCE")
        for u in sorted(assign.keys()):
            drones = assign[u]
            total_rate = 0.0
            print(f"User {u:2} is served by {len(drones)} drone(s):")
            for d in drones:
                link_snr  = data["SNR_THz"][(d, u)] if t_air_val == 1 else data["SNR_RF"][(d, u)]
                link_plos = data["pLos_du"][(d, u)]
                link_rate = sol["r_du"][(d, u)]
                total_rate += link_rate

                print(
                    f"  -> Drone {d:3} | Tech: {tech_str} | "
                    f"pLoS: {link_plos:6.4f} | "
                    f"SNR: {link_snr:7.2f} dB | "
                    f"Rate: {link_rate:9.3f} Mbps"
                )

            print(f"     ==> Total rate for user {u:2}: {total_rate:9.3f} Mbps\n")
    
    #Print A2A pLoS, SNR, Rate
    print("\nSELECTED A2A LINK PERFORMANCE")
    rf_links  = sol.get("a2a_rf_links", [])
    thz_links = sol.get("a2a_thz_links", [])

    if not rf_links and not thz_links:
        print("No A2A links are selected.")
    else:
        for (d1, d2) in rf_links:
            print(
                f"  RF  A2A link {d1:3} -> {d2:3} | "
                f"pLoS: {data['pLos_ddp'][(d1,d2)]:6.4f} | "
                f"SNR: {data['SNR_RF_A2A'][(d1,d2)]:7.2f} dB | "
                f"Rate: {data['R_RF_A2A'][(d1,d2)]:9.3f} Mbps"
            )

        for (d1, d2) in thz_links:
            print(
                f"  THz A2A link {d1:3} -> {d2:3} | "
                f"pLoS: {data['pLos_ddp'][(d1,d2)]:6.4f} | "
                f"SNR: {data['SNR_THz_A2A'][(d1,d2)]:7.2f} dB | "
                f"Rate: {data['R_THz_A2A'][(d1,d2)]:9.3f} Mbps"
            )
    # Print average SNR and pLoS for selected links
    print("\nAVERAGE LINK STATISTICS")

    # A2G averages
    a2g_plos_vals = []
    a2g_snr_vals = []

    for u in sorted(assign.keys()):
        drones = assign[u]
        for d in drones:
            a2g_plos_vals.append(data["pLos_du"][(d, u)])
            if t_air_val == 1:
                a2g_snr_vals.append(data["SNR_THz"][(d, u)])
            else:
                a2g_snr_vals.append(data["SNR_RF"][(d, u)])

    if a2g_plos_vals:
        avg_a2g_plos = sum(a2g_plos_vals) / len(a2g_plos_vals)
        avg_a2g_snr  = sum(a2g_snr_vals) / len(a2g_snr_vals)
        print(
            f"A2G ({tech_str}) -> "
            f"Average pLoS: {avg_a2g_plos:.4f} | "
            f"Average SNR: {avg_a2g_snr:.2f} dB"
        )
    else:
        print("A2G -> No selected links.")

    # A2A RF averages
    rf_plos_vals = [data["pLos_ddp"][(d1, d2)] for (d1, d2) in rf_links]
    rf_snr_vals  = [data["SNR_RF_A2A"][(d1, d2)] for (d1, d2) in rf_links]

    if rf_plos_vals:
        avg_rf_plos = sum(rf_plos_vals) / len(rf_plos_vals)
        avg_rf_snr  = sum(rf_snr_vals) / len(rf_snr_vals)
        print(
            f"A2A RF -> "
            f"Average pLoS: {avg_rf_plos:.4f} | "
            f"Average SNR: {avg_rf_snr:.2f} dB"
        )
    else:
        print("A2A RF -> No selected links.")

    # A2A THz averages
    thz_plos_vals = [data["pLos_ddp"][(d1, d2)] for (d1, d2) in thz_links]
    thz_snr_vals  = [data["SNR_THz_A2A"][(d1, d2)] for (d1, d2) in thz_links]

    if thz_plos_vals:
        avg_thz_plos = sum(thz_plos_vals) / len(thz_plos_vals)
        avg_thz_snr  = sum(thz_snr_vals) / len(thz_snr_vals)
        print(
            f"A2A THz -> "
            f"Average pLoS: {avg_thz_plos:.4f} | "
            f"Average SNR: {avg_thz_snr:.2f} dB"
        )
    else:
        print("A2A THz -> No selected links.")
    # Plotting the deployment
    with plt.rc_context({
        "font.family":      "serif",
        "font.serif":       ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size":        26,
        "axes.labelsize":   30,
        "axes.labelweight": "bold",
        "xtick.labelsize":  22,
        "ytick.labelsize":  22,
        "legend.fontsize":  30,
    }):
        fig = plt.figure(figsize=(11, 10))
        ax = fig.add_subplot(111, projection="3d")

        # Users
        u_xs = [pos[0] for pos in user_xy]
        u_ys = [pos[1] for pos in user_xy]
        ax.scatter(u_xs, u_ys, 0, c="gray", marker="o", s=30, alpha=0.5)

        # Drones
        masters = sol.get("masters", [])
        slaves  = sol.get("slaves", [])

        for d_idx in masters:
            x, y, h = cand[d_idx]
            ax.scatter(x, y, h, c="red", marker="^", s=120)
            ax.text(x, y, h, f" G{d_idx}", color="red", fontsize=18, fontweight="bold")

        for d_idx in slaves:
            x, y, h = cand[d_idx]
            ax.scatter(x, y, h, c="dimgray", marker="v", s=100)
            ax.text(x, y, h, f" R{d_idx}", color="dimgray", fontsize=18, fontweight="bold")

        # A2G links
        for u_idx, d_list in assign.items():
            ux, uy = user_xy[u_idx]
            if not isinstance(d_list, (list, tuple, set)):
                d_list = [d_list]
            for d_idx in d_list:
                dx, dy, dh = cand[d_idx]
                ax.plot(
                    [ux, dx], [uy, dy], [0, dh],
                    c=a2g_color, linestyle=":", linewidth=1.2, alpha=0.85
                )

        # A2A RF links
        for (d1, d2) in rf_links:
            x1, y1, h1 = cand[d1]
            x2, y2, h2 = cand[d2]
            ax.plot(
                [x1, x2], [y1, y2], [h1, h2],
                c=a2a_rf_color, linestyle="-", linewidth=2.5
            )

        # A2A THz links
        for (d1, d2) in thz_links:
            x1, y1, h1 = cand[d1]
            x2, y2, h2 = cand[d2]
            ax.plot(
                [x1, x2], [y1, y2], [h1, h2],
                c=a2a_thz_color, linestyle="-", linewidth=2.5
            )

        ax.set_xlabel("X Position (m)", labelpad=14)
        ax.set_ylabel("Y Position (m)", labelpad=14)
        ax.set_zlabel("")

        ax.tick_params(axis="both", which="major", labelsize=22, pad=4)
        ax.tick_params(axis="z", which="major", labelsize=22, pad=8)

        for label in ax.get_xticklabels():
            label.set_fontweight("bold")
        for label in ax.get_yticklabels():
            label.set_fontweight("bold")
        for label in ax.get_zticklabels():
            label.set_fontweight("bold")

        legend_handles = [
            Line2D([0], [0], marker="o", color="gray",    linestyle="None", markersize=10, alpha=0.7, label="Users"),
            Line2D([0], [0], marker="^", color="red",     linestyle="None", markersize=13,            label="Gateway"),
            Line2D([0], [0], marker="v", color="dimgray", linestyle="None", markersize=13,            label="Relay"),
            Line2D([0], [0], color=a2g_rf_color,  linestyle=":", linewidth=4.5, label="A2G RF"),
            Line2D([0], [0], color=a2g_thz_color, linestyle=":", linewidth=4.5, label="A2G THz"),
            Line2D([0], [0], color=a2a_rf_color,  linestyle="-", linewidth=4.5, label="A2A RF"),
            Line2D([0], [0], color=a2a_thz_color, linestyle="-", linewidth=4.5, label="A2A THz"),
        ]

        leg = ax.legend(
            handles=legend_handles,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.04),
            ncol=3,
            frameon=True,
            framealpha=0.95,
            borderpad=0.3,
            labelspacing=0.2,
            handlelength=1.5,
            columnspacing=0.5,
            handletextpad=0.4,
        )

        for text in leg.get_texts():
            text.set_fontweight("bold")

        fig.subplots_adjust(left=0.02, right=0.84, bottom=0.24, top=0.98)

        fig.text(
            0.80, 0.62, "Altitude (m)",
            rotation=90,
            fontsize=26,
            fontweight="bold",
            va="center",
            ha="center",
            family="serif"
        )

        fig.savefig(filename, format="pdf", dpi=600)
        plt.show()

