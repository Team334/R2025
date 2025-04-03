// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.Constants.WristevatorConstants.Preset.HUMAN;
import static frc.robot.Constants.WristevatorConstants.Preset.L1;
import static frc.robot.Constants.WristevatorConstants.Preset.L2;
import static frc.robot.Constants.WristevatorConstants.Preset.L3;
import static frc.robot.Constants.WristevatorConstants.Preset.L4;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;
import java.util.function.Consumer;

public class Autos {
  private final Swerve _swerve;
  private final Consumer<Piece> _currentPieceSetter;
  private final Wristevator _wristevator;
  private final Manipulator _manipulator;
  private final Intake _intake;
  private final Serializer _serializer;

  private final AutoFactory _factory;

  private boolean _seesPiece = false;

  private SendableChooser<Side> _selector = new SendableChooser<Side>();

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
              traj = traj.flipped();

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

    _selector.addOption("Left", Side.LEFT);
    _selector.addOption("Center", Side.CENTER);
    _selector.addOption("Right", Side.RIGHT);

    SmartDashboard.putData("Auton Side Selector", _selector);
  }

  public AutoRoutine ground3P() {
    var routine = _factory.newRoutine("Ground 3 Piece");
    var start = routine.trajectory("Start-Reef");
    var humanToReef1 = routine.trajectory("Human-Reef1");
    var reefToHuman1 = routine.trajectory("Reef-Human1");
    var humanToReef2 = routine.trajectory("Human-Reef2");
    var reefToHuman2 = routine.trajectory("Reef-Human2");

    routine
        .active()
        .onTrue(
            sequence(
                Commands.runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                start.resetOdometry(),
                start.cmd()));

    start.done().onTrue(reefToHuman1.cmd());

    reefToHuman1
        .active()
        .and(() -> _seesPiece)
        .onTrue(
            sequence(
                _swerve.pieceAlign(),
                _swerve.driveTo(humanToReef1.getFinalPose().get()),
                reefToHuman2.cmd()));
    reefToHuman1.done().onTrue(humanToReef1.cmd());
    humanToReef1.done().onTrue(reefToHuman2.cmd());

    reefToHuman2
        .active()
        .and(() -> _seesPiece)
        .onTrue(
            sequence(
                _swerve.pieceAlign(),
                _swerve.driveTo(humanToReef2.getFinalPose().get()),
                reefToHuman2.cmd()));
    reefToHuman2.done().onTrue(humanToReef2.cmd());

    return routine;
  }

  public AutoRoutine simplePath() {
    var routine = _factory.newRoutine("Simple Path");
    var traj = routine.trajectory("Simple Path");

    routine
        .active()
        .onTrue(
            sequence(
                Commands.runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
                traj.resetOdometry(),
                traj.cmd()));

    return routine;
  }

  public AutoRoutine onePiece() {
    var routine = _factory.newRoutine("One Piece");
    var traj = routine.trajectory("Start-Reef(A)");

    routine
        .active()
        .onTrue(
            sequence(
                Commands.runOnce(() -> _currentPieceSetter.accept(Piece.CORAL)),
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
