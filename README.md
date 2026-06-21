# Joint Optimization of Drone Deployment and THz/RF Band Selection

This repository contains the ENS 492 graduation project optimization code for drone deployment with RF and THz communication links.


## Repository structure

```text
.
├── configs/
│   ├── dense_urban_50users_90_100m_rmin5.yaml
│   └── template_edit_me.yaml
├── data/
│   └── coefficients/
│       ├── G2UAV_model_coefficients.mat    
│       └── UAV2UAV_model_coefficients.mat  
├── src/
│   ├── channel_models.py
│   ├── scenario_builder.py
│   ├── optimization_model.py
│   ├── visualization.py
│   ├── utils.py
│   └── run_scenario.py
├── results/
├── notebook/
```

## Files
- `src/channel_models.py`: LoS probability, RF path loss, THz path loss, SNR, and data-rate functions.
- `src/scenario_builder.py`: Creates users, candidate drone locations, A2G/A2A link tables, and the `data` dictionary.
- `src/optimization_model.py`: Gurobi model, decision variables, objective, and constraints.
- `src/visualization.py`: Link-performance printout and 3D deployment figure.
- `src/run_scenario.py`: Main command-line runner.

## Running the default scenario
To run the project, Gurobi license is required. 
From the repository:
```bash
python src/run_scenario.py --config configs/dense_urban_50users_90_100m_rmin5.yaml
```

The default config values:
- 50 users
- dense urban environment
- altitudes `[200, 300]` m
- 250 m candidate grid spacing
- `Rmin = 5` Mbps
- RF and THz SNR thresholds set to 5 dB
- master cost 40
- slave cost 6

Outputs are written to:
```text
results/dense_urban_50users_90_100m_rmin5/
```

## Running without plotting
```bash
python src/run_scenario.py --config configs/dense_urban_50users_90_100m_rmin5.yaml --no-plot
```

## Creating a new scenario
Copy the template in configs/template_edit_me.yaml configs/my_scenario.yaml, edit the values and run python src/run_scenario.py --config configs/my_scenario.yaml command.

## Jupyter notebook
The original jupyter notebook is in:

```text
notebook/Joint_optimization_of_drone_deployment_and_thz_rf_band_selection.ipynb
```
