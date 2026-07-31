# Joint Optimization of Drone Deployment and THz/RF Band Selection

This repository contains the ENS 492 graduation project optimization code for drone deployment with RF and THz communication links and a simple demo.


## Repository structure

```text
.
├── configs/
│   └──  dense_urban_50users_200_300m_rmin5.yaml
│   
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
- `src/scenario_builder.py`: Creates users, candidate drone locations, candidate A2G/A2A links, and the `data` dictionary.
- `src/optimization_model.py`: Gurobi model, decision variables, objective, and constraints.
- `src/visualization.py`: Link-performance prints and 3D deployment figure.
- `src/run_scenario.py`: Main command runner.

## Running the default scenario
Note: To run the project, Gurobi license is required. 

Default scenario can be run with the following command:
```bash
python src/run_scenario.py --config configs/dense_urban_50users_200_300m_rmin5.yaml
```

The default config values:
- 50 users
- dense urban environment
- altitudes `[200, 300]` m
- 250 m candidate grid spacing
- `Rmin = 5` Mbps
- 5 dB RF and THz SNR thresholds
- master cost 40
- slave cost 6

Outputs are written to the following path:
```text
results/dense_urban_50users_200_300m_rmin5/
```

## Running without plotting
```bash
python src/run_scenario.py --config configs/dense_urban_50users_200_300m_rmin5.yaml --no-plot
```

## Creating a new scenario
For creating a new scenario, edit the values in yaml file and run python src/run_scenario.py --config configs/my_scenario.yaml command.

## Jupyter notebook
The original jupyter notebook is placed in:

```text
notebook/Joint_optimization_of_drone_deployment_and_thz_rf_band_selection.ipynb
```
