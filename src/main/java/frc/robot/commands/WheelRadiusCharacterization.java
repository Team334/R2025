// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class WheelRadiusCharacterization extends Command {
  private final Swerve _swerve;

  private double _lastGyroYaw;
  private double _accumGyroYaw;

  private double[] _initialWheelDistances;

  private double _wheelRadius;

  public WheelRadiusCharacterization(Swerve swerve) {
    setName("Wheel Radius Characterization");

    _swerve = swerve;
    addRequirements(_swerve);
  }

  /** Get the estimated wheel radius. */
  public Distance getWheelRadius() {
    return Meters.of(_wheelRadius);
  }

  // returns the distance traveled by each individual drive wheel in radians
  private double[] getWheelDistancesRadians() {
    SwerveModulePosition[] positions = _swerve.getState().ModulePositions;

    double[] distances = new double[4];

    for (int i = 0; i < _swerve.getModules().length; i++) {
      // radians = (meters / radius)
      distances[i] = positions[i].distanceMeters / TunerConstants.FrontLeft.WheelRadius;
    }

    return distances;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lastGyroYaw = _swerve.getHeading().getRadians();
    _accumGyroYaw = 0;

    _initialWheelDistances = getWheelDistancesRadians();

    _wheelRadius = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(0, 0, -1); // TODO: make this tuneable

    // add the heading traveled since the last execute call to the accum yaw
    // angle modulus is needed for when the gyro crosses from [0, pi] to [-pi, 0]
    _accumGyroYaw +=
        Math.abs(MathUtil.angleModulus(_swerve.getHeading().getRadians() - _lastGyroYaw));
    _lastGyroYaw = _swerve.getHeading().getRadians();

    DogLog.log(
        "Wheel Radius Characterization/Meters Turned",
        SwerveConstants.driveRadius.in(Meters) * _accumGyroYaw);

    double averageWheelDistance = 0;
    double[] wheelDistances = getWheelDistancesRadians();

    for (int i = 0; i < _swerve.getModules().length; i++) {
      // wheel distances should be positive always since the turning distance will always be
      // positive
      averageWheelDistance += Math.abs(wheelDistances[i] - _initialWheelDistances[i]);
    }

    averageWheelDistance /= _swerve.getModules().length;

    _wheelRadius = (SwerveConstants.driveRadius.in(Meters) * _accumGyroYaw) / averageWheelDistance;

    DogLog.log(
        "Wheel Radius Characterization/Estimated Wheel Radius", Units.metersToInches(_wheelRadius));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.drive(0, 0, 0);

    if (_accumGyroYaw <= 2 * Math.PI) {
      FaultLogger.report("Need more info for characterization!", FaultType.ERROR);
    } else {
      FaultLogger.report(
          "Wheel Radius (inches): " + Units.metersToInches(_wheelRadius), FaultType.INFO);
    }
  }
}
