 # AV-fuzzer
AV-fuzzer is an autonomous vehicle scenario fuzz testing framework built on the CARLA simulator. It uses a genetic algorithm to automatically generate diverse test scenarios for evaluating vehicle safety under varying environmental conditions and NPC behaviors.

## Features
- Genetic algorithm–based scenario generation (configurable population size, number of generations, crossover rate, mutation rate, tournament size, diversity weight)-
- Integration with CARLA BehaviorAgent for realistic NPC control
- Configurable weather parameters, NPC behaviors, and vehicle start/end locations

## Installation
1. Clone the repository
   ```sh
   https://github.com/cclinus/AV-Fuzzer.git
   cd av-fuzzer
   ```
2. Install Python dependencies
   ```sh
   pip install -r requirements.txt
   ```
3. Download and run CARLA (version 0.9.x)
   - Download and extract CARLA from the official site.
   - Start the CARLA server:
   ```sh
    ./CarlaUE4.sh (linux)
    .\CarlaUE4.exe (windows)
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

## License
      
