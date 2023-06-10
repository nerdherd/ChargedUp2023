# ChargedUp2023

***REVERT COMMIT 1ccc3624a70894689ab905dcbe8031c5c6df80eb BEFORE MERGING*** 

FRC Team 687's code for its Charged Up 2023 competition robot, *Is Me*.

## How to Use

This program requires WPILib VSCode 2023 and FRC Game Tools 2023 to be installed.
Clone this repository from <https://github.com/nerdherd/ChargedUp2023> 
or download and unzip the [latest release](https://github.com/nerdherd/ChargedUp2023/releases/latest),
open it in VSCode, and press shift + F5 to deploy it to the robot. 
Then use Driver Station to enable the robot.

This program uses two PS4 Controllers for operator input. Due to hardware issues,
a wrapper class `BadPS4` is used to translate button inputs to typical PS4 inputs.

## Contributing

Please branch off from the `dev` branch for any changes, and then submit a PR when finished. 
Changes will be merged into the main branch and tagged as a release every two weeks.

## 3rd Party Libraries

This repository uses:
- [NavX (Official 2023 Release)](https://dev.studica.com/releases/2023/NavX.json)
- [CTRE Phoenix v5](https://store.ctr-electronics.com/software/)
- [PhotonVision](https://photonvision.org/)
- [PathPlanner](https://github.com/mjansen4857/pathplanner)
