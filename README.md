# SwerveBase-CTRE
[![CI](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml/badge.svg)](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml)

A base project for future robots that has CTRE generated swerve drive code and Photon Vision AprilTag processing.

# Features
- Swerve drive code using CTRE's swerve code generator.
- Device logging with SignalLogger and data logging with Epilogue and DogLog.
- Device Fault Logging as telemetry for at-home testing. The faults are also logged with DogLog for post-match review.
- Pre-match self-check with custom self-check commands.
- A custom `VisionPoseEstimator` class that reads from a Photon Vision camera and updates the swerve pose estimator with filtered and disambiguated AprilTag vision measurements.
- Automated wheel radius characterization routine (based on 6328's wheel characterization).
- A custom zshell script that performs camera calibration with mrcal given an input video of a chessboard dance.
- Choreo support.

# Todo
- Add more unit tests.
- Add proper licenses.

# How This Will Be Used
- At the start of the 2025 season this project should be re-imported in the 2025 wpilib release for any additional updates.
- After that, since this project is a template on github, the actual 2025 robot code repo can be generated from this template:
![github templates](https://docs.github.com/assets/cb-76823/mw-1440/images/help/repository/use-this-template-button.webp)

# Calibration Process
- Run `calibration/calibrator.zsh` in zshell in WSL in the `calibration` folder (make sure all packages are installed).
- Analyze calibration as according to the mrcal tour.
- If the calibration was good, load the generated JSON into photon vision.
