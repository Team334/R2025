// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import frc.robot.subsystems.Swerve;

public class Autos {
  private final Swerve _swerve;

  private final AutoFactory _factory;

  private boolean _seesPiece = false;

  public Autos(Swerve swerve) {
    _swerve = swerve;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            (traj, isActive) -> {
              traj = traj.flipped();

              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });
  }

  public AutoRoutine simpleTrajectory() {
    var routine = _factory.newRoutine("Simple Trajectory");
    var start = routine.trajectory("Start-Reef");
    var humanToReef1 = routine.trajectory("Human-Reef1");
    var reefToHuman1 = routine.trajectory("Reef-Human1");
    var humanToReef2 = routine.trajectory("Human-Reef2");
    var reefToHuman2 = routine.trajectory("Reef-Human2");

    routine.active().onTrue(sequence(start.resetOdometry(), start.cmd()));

    start.done().onTrue(reefToHuman1.cmd());

    reefToHuman1
        .active()
        .and(() -> _seesPiece)
        .onTrue(
            sequence(
                _swerve.alignToPiece(),
                _swerve.driveTo(humanToReef1.getFinalPose().get()),
                reefToHuman2.cmd()));
    reefToHuman1.done().onTrue(humanToReef1.cmd());
    humanToReef1.done().onTrue(reefToHuman2.cmd());

    reefToHuman2
        .active()
        .and(() -> _seesPiece)
        .onTrue(
            sequence(
                _swerve.alignToPiece(),
                _swerve.driveTo(humanToReef2.getFinalPose().get()),
                reefToHuman2.cmd()));
    reefToHuman2.done().onTrue(humanToReef2.cmd());

    return routine;
  }
}
