 # AV-fuzzer
AV-Fuzzer is a scenario fuzzing framework for autonomous vehicles, built on top of the CARLA simulator. It leverages a genetic algorithm to automatically generate diverse and safety-critical driving scenarios by mutating environmental conditions and non-player character (NPC) behaviors.

## Key Features
### Genetic Algorithm–Based Scenario Generation
Automatically evolves test scenarios using configurable parameters:
- Population size
- Number of generations
- Crossover and mutation rates
- Tournament selection size

## Installation
1. Clone the repository
   ```sh
   https://github.com/cclinus/AV-Fuzzer.git
   cd av-fuzzer/carla_sim/
   ```
2. Install Python dependencies
   ```sh
   pip install -r requirements.txt
   ```
3. Download and run CARLA (version 0.9.x)
   - Download and extract CARLA from the official site.
   - Start the CARLA server:
   ```sh
    ./CarlaUE4.sh (in Linux)
    .\CarlaUE4.exe (in Windows)
   ```
## Configuration
Configuration files are stored in the parameter directory. Key sections:
- spawn.yaml: Defines vehicle start and end points
- weather.yaml: Specifies weather conditions
- ga.yaml: Sets genetic algorithm parameters (pop_size, max_gens, crossover_rate, mutation_rate, tournament_k, diversity_weight

## Usage

  ```sh
  python GA.py
  ```
## Paper
- AV-FUZZER: Finding Safety Violations in Autonomous Driving Systems (ISSRE'20)
  
## License
This project is licensed under the BSD 3-Clause License. See the [LICENSE](./LICENSE) file for details.      
