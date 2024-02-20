[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

# HGIC: A Hand Gesture Based Interactive Control System for Efficient and Scalable Multi-UAV Operationss

This repository contains the code for the paper [HGIC](https://sites.bu.edu/mrs2023/program/list-of-accepted-papers-and-presentations/)

## Code Structure

* `hand_recognition` folder contains the code for hand gesture recognition.
* `swarm_controller` folder contains the code for swarm control.
* `settings.json` file contains the settings for the AirSim simulator.

<!-- GETTING STARTED -->

## Requirements
* Plase run this script
  ```sh
  ./run.sh
  ```
or ...

* Install by running the following command.
  ```sh
  git clone https://github.com/kingchou007/HGIC_Platform
  ```
* Environment
  ```sh
  pip install -r requirements.txt
  ```

## AirSim Download and Installation

Please follow the instructions in the [AirSim](https://microsoft.github.io/AirSim/) website to download and install the AirSim simulator.

Once you have installed the AirSim simulator, please replace the `settings.json` file in the `Documents/AirSim` folder with the `settings.json` file in the `AirSim` folder in this repository.

## Running the Code

Please open the AirSim simulator. Then, Open two terminals and run the following commands in each terminal.

Terminal 1:

```sh
cd hand_recognition # Path: hand_recognition
python3 main.py
```

Terminal 2:

```sh
cd swarm_controller # Path: swarm_controller
python connect.py
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

* [AirSim](https://github.com/microsoft/AirSim)
* [Google Media-Pipe](https://github.com/google/mediapipe)
