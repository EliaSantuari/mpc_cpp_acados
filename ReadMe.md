# Quadruped Robot Closed-Loop Simulation with OPTI-PESSI MPC

This repository contains a **closed-loop simulation** for quadruped robot feet position control, implemented using an **OPTI-PESSI MPC** controller developed in **C++**.

The build pipeline automates the generation of the dynamic model and the OCP (*Optimal Control Problem*) via a dedicated Python script before compiling the C++ core.


The simulation workflow relies on an integrated C++/Python approach:

**Global Configuration (`config.yaml`)**: Physical robot parameters, MPC weight costs, prediction horizons, and all the other parameters are defined centrally in a single YAML file.

**Automatic OCP Generation (Build-time)**: When triggering the build process from inside `build/`, CMake automatically invokes `model_gen.py`. This script parses configuration parameters to generate and initialize the dynamic model and the optimal control problem (OCP).

**Closed-Loop Execution (C++)**: The compiled executable (`main.cpp`) runs the real-time closed-loop control loop, solving the MPC problem.

**Data Logging**: State vectors, controls, and reference trajectories are exported during runtime to `build/mpc_simulation_data.csv`.

**Data Visualization**: Executing `plotter.py` reads the output CSV file and displays time-series plots of state trajectories, control inputs others and a cool animation. 



### Compilation and Execution

Create the `build` directory and run CMake. The build process will automatically run `setup_ocp.py` before compiling the C++ executable:

```bash

mkdir -p build && cd build

# 2. Configure project and compile
cmake ..
make

# 3. Run the closed-loop simulation
./mpc_sim
```

### Visualizing Results

Once the simulation finishes, output logs are stored in `build/mpc_simulation_data.csv`. To render the plots, run:

```bash
# Execute from the project root directory
python3 plotter.py
```
