// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanEntry;
import frc.lib.Tuning;
import frc.robot.subsystems.Swerve;

public class Autos {
  private final Swerve _swerve;

  private final AutoFactory _factory;

  private final BooleanEntry _seesPiece = Tuning.entry("PieceVisable", false);
  private final BooleanEntry _end = Tuning.entry("End", false);

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
    var start = routine.trajectory("Start-Reef-Human");
    var humanToReef = routine.trajectory("Human-Reef");
    var reefToHuman = routine.trajectory("Reef-Human");

    routine.active().onTrue(sequence(start.resetOdometry(), start.cmd()));

    start.done().onTrue(humanToReef.cmd());

    humanToReef.done().onTrue(reefToHuman.cmd());
    reefToHuman.active().and(_seesPiece).onTrue(runOnce(() -> reefToHuman.cmd().cancel()));

    reefToHuman.cmd().handleInterrupt(() -> _swerve.alignToPiece().andThen(humanToReef.cmd()));
    reefToHuman.done().onTrue(humanToReef.cmd());

    return routine;
  }
}
