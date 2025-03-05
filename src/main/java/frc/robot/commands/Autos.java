// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.WristevatorConstants.Preset.HUMAN;
import static frc.robot.Constants.WristevatorConstants.Preset.L4;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;

public class Autos {
  private final Swerve _swerve;
  private final Wristevator _wristevator;

  private final AutoFactory _factory;

  public Autos(Swerve swerve, Wristevator wristevator) {
    _swerve = swerve;
    _wristevator = wristevator;

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

  public AutoRoutine ThreePieceAuton() {
    var routine = _factory.newRoutine("Command Test");
    var A = routine.trajectory("3PieceAuton-A");
    var B = routine.trajectory("3PieceAuton-B");
    var C = routine.trajectory("3PieceAuton-C");
    var D = routine.trajectory("3PieceAuton-D");
    var E = routine.trajectory("3PieceAuton-E");

    routine.active().onTrue(sequence(A.resetOdometry(), A.cmd()));

    routine.anyActive(A, C, E).onTrue(_wristevator.setGoal(L4));
    routine.anyActive(B, D).onTrue(_wristevator.setGoal(HUMAN));

    A.atTime("placeL4")
        .onTrue(
            sequence(
                B.resetOdometry(),
                waitSeconds(1.5),
                B.cmd().andThen(C.resetOdometry(), waitSeconds(1.5), C.cmd())));
    C.atTime("placeL4")
        .onTrue(
            sequence(
                D.resetOdometry(),
                waitSeconds(1.5),
                D.cmd().andThen(E.resetOdometry(), waitSeconds(1.5), E.cmd())));

    return routine;
  }
}
