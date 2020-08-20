# AV-Fuzzer
## Installing LGSVL Simulator
  * We are using LGSVL 2019.05
  * Download page: https://github.com/lgsvl/simulator
## Installing Apollo Simulator forked 3.5 LGSVL: 
### Pulling LGSVL Docker image
LGSVL maintains a docker image to be used alongside this repository. The docker image is available [here](https://hub.docker.com/r/lgsvl/apollo-3.5/).

To pull the image use the following command:

    docker pull lgsvl/apollo-3.5

### Cloning the Repository
This repository includes a couple of submodules for HD Maps and lgsvl msgs. To make sure that the submodules are also cloned use the following command:

    git clone --recurse-submodules https://github.com/lgsvl/apollo-3.5.git


### Building Apollo and bridge
Now everything should be in place to build apollo. Apollo must be built from the container. To launch the container navigate to the directory where the repository was cloned and enter:

    ./docker/scripts/dev_start.sh

This should launch the container and mount a few volumes. It could take a few minutes to pull the latest volumes on the first run.

To get into the container:

    ./docker/scripts/dev_into.sh

Build Apollo:

    ./apollo.sh build_gpu

### Launching Apollo alongside the simulator

Here we only describe only a simple case of driving from point A to point B using Apollo and the simulator. 

To launch apollo, first launch and enter a container as described in the previous steps.

* To start Apollo:

        bootstrap.sh

    Note: you may receive errors about dreamview not being build if you do not run the script from the `/apollo` directory.

* Launch bridge (inside docker container):

        bridge.sh
* Break the bridge:

        ctrl+c
* Stop Apollo:

        bootstrap.sh stop
        
### Reference:
* https://www.lgsvlsimulator.com/docs/apollo-instructions/
* https://github.com/lgsvl/apollo-3.5


## Installing LGSVL Python API
### Cloning the Repository

    git clone https://github.com/lgsvl/PythonAPI.git    
### Requirements

* Python 3.5 or higher

### Installing

    pip3 install --user .
    
    # install in development mode
    pip3 install --user -e .

### Reference:
* https://github.com/lgsvl/PythonAPI
* https://www.lgsvlsimulator.com/docs/python-api/

## Installing AV-Fuzzer & Running AV-Fuzzer
### Cloning the Repository
    git clone https://github.com/cclinus/AV-Fuzzer.git
    
### Running AV-Fuzzer
* Launching Apollo alongside the simulator (see the section above)
* go to AV-Fuzzer directory and choose the driving environment you want to test 
  * `cd <where you put the AV-Fuzzer>/AV-Fuzzer/freeway` 
  * or
  * `cd <where you put the AV-Fuzzer>/AV-Fuzzer/urban` 
* `python3 drive_experiment.py`
* If you want to continue the previous experiment from a saved checkpoint, please uncomment the set checkpoint line and in drive_experiment.py 
  
