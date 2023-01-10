# ChargedUp2023

Team 687's code for the 2023 FRC game, Charged Up.

## How to Use

This program requires WPILib VSCode 2023 and FRC Game Tools 2023 to be installed.
Clone this repository from [https://github.com/nerdherd/ChargedUp2023], open it in VSCode, and press shift + F5 to deploy it to the robot. 
Then use Driver Station to enable the robot.

This program uses two PS4 Controllers for operator input. Because the buttons on the controllers used by team 687 are mapped incorrectly,
a wrapper class `BadPS4` is used to translate button inputs to typical PS4 inputs.

## Contributing

Please branch off from the `dev` branch for any changes, and then submit a PR when finished. 
Changes will be merged into the main branch and tagged as a release every two weeks.

## 3rd Party Libraries

This repository uses:
- [NavX (Unofficial 2023 release)](https://github.com/rzblue/navx-frc)
- [Phoenix CTRE v5](https://store.ctr-electronics.com/software/)
- [PhotonVision](https://photonvision.org/)
