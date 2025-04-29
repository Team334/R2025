// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants.FieldLocation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;
import frc.robot.utils.AlignPoses.AlignSide;
import java.util.function.Consumer;

public class Autos {
  private final Swerve _swerve;
  private final Consumer<Piece> _currentPieceSetter;
  private final Wristevator _wristevator;
  private final Manipulator _manipulator;
  private final Intake _intake;
  private final Serializer _serializer;

  private final AutoFactory _factory;

  // private boolean _seesPiece = false;

  private SendableChooser<Side> _sideSelector = new SendableChooser<Side>();

  private enum Side {
    LEFT("Left "),
    CENTER("Center "),
    RIGHT("Right ");

    private final String _dir;

    private Side(String dir) {
      _dir = dir;
    }

    public String getDirectory() {
      return _dir;
    }
  }

  public Autos(
      Swerve swerve,
      Consumer<Piece> currentPieceSetter,
      Wristevator wristevator,
      Manipulator manipulator,
      Intake intake,
      Serializer serializer) {
    _swerve = swerve;
    _currentPieceSetter = currentPieceSetter;
    _wristevator = wristevator;
    _manipulator = manipulator;
    _intake = intake;
    _serializer = serializer;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            (traj, isActive) -> {
              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });

    _factory
        .bind("Ground Intake", Superstructure.groundIntake(_intake, _serializer))
        .bind("L4", _wristevator.setGoal(L4))
        .bind("L3", _wristevator.setGoal(L3))
        .bind("L2", _wristevator.setGoal(L2))
        .bind("L1", _wristevator.setGoal(L1))
        .bind("Human", _wristevator.setGoal(HUMAN))
        .bind("Manipulator Feed", _manipulator.feed().withTimeout(1.5));

    _sideSelector.setDefaultOption("Center", Side.CENTER);

    _sideSelector.addOption("Left", Side.LEFT);
    _sideSelector.addOption("Center", Side.CENTER);
    _sideSelector.addOption("Right", Side.RIGHT);

    SmartDashboard.putData("Auton Side Selector", _sideSelector);
  }

  public AutoRoutine taxi() {
    var routine = _factory.newRoutine("Taxi");
    var traj = routine.trajectory(_sideSelector.getSelected().getDirectory() + "Taxi");

    routine
        .active()
        .onTrue(
            sequence(
                runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                traj.resetOdometry(),
                traj.cmd()));

    return routine;
  }

  public AutoRoutine onePiece() {
    var routine = _factory.newRoutine("One Piece");
    var trajA = routine.trajectory(_sideSelector.getSelected().getDirectory() + "1PA");
    var trajB = routine.trajectory(_sideSelector.getSelected().getDirectory() + "1PB");

    routine
        .active()
        .onTrue(
            sequence(
                runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                trajA.resetOdometry(),
                trajA.cmd()));

    trajA
        .done()
        .onTrue(
            sequence(
                _wristevator.setGoal(L4),
                _swerve.fieldAlign(FieldLocation.REEF, AlignSide.LEFT),
                _manipulator.feed().withTimeout(1.5),
                trajB.cmd()));

    trajB.done().onTrue(sequence(_wristevator.setGoal(HOME)));

    return routine;
  }

  public AutoRoutine twoPiece() {
    var routine = _factory.newRoutine("Two Piece");
    var trajA = routine.trajectory(_sideSelector.getSelected().getDirectory() + "2PA");
    var trajB = routine.trajectory(_sideSelector.getSelected().getDirectory() + "2PB");
    var trajC = routine.trajectory(_sideSelector.getSelected().getDirectory() + "2PC");

    routine
        .active()
        .onTrue(
            sequence(
                runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                trajA.resetOdometry(),
                trajA.cmd()));

    trajA
        .done()
        .onTrue(
            sequence(
                _wristevator.setGoal(L4),
                _swerve.fieldAlign(FieldLocation.REEF, AlignSide.LEFT),
                _manipulator.feed().withTimeout(1.5),
                trajB.cmd()));

    trajB.done().onTrue(sequence(_wristevator.setGoal(HOME), trajC.cmd()));

    return routine;
  }

  public AutoRoutine simplePath() {
    var routine = _factory.newRoutine("SimplePath");
    var traj = routine.trajectory("SimplePath");

    routine
        .active()
        .onTrue(
            sequence(
                runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                traj.resetOdometry(),
                traj.cmd()));

    return routine;
  }

  public AutoRoutine resetOdometry() {
    var routine = _factory.newRoutine("Reset Odometry");
    var traj = routine.trajectory("ResetOdometry");

    routine.active().onTrue(sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }
}
