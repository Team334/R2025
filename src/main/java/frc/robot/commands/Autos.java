package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants.Alignment;
import frc.robot.Constants.Piece;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;
import java.util.function.Consumer;

public class Autos {
  private final AutoFactory _factory;

  private final Consumer<Piece> _manipulatorPieceSetter;

  private final Swerve _swerve;
  private final Wristevator _wristevator;
  private final Manipulator _manipulator;

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
      Consumer<Piece> manipulatorPieceSetter,
      Swerve swerve,
      Wristevator wristevator,
      Manipulator manipulator) {
    _manipulatorPieceSetter = manipulatorPieceSetter;

    _swerve = swerve;
    _wristevator = wristevator;
    _manipulator = manipulator;

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

    _sideSelector.setDefaultOption("Center", Side.CENTER);

    _sideSelector.addOption("Left", Side.LEFT);
    _sideSelector.addOption("Center", Side.CENTER);
    _sideSelector.addOption("Right", Side.RIGHT);

    SmartDashboard.putData("Auton Side Selector", _sideSelector);
  }

  public Command taxi() {
    return sequence(
        runOnce(() -> _manipulatorPieceSetter.accept(Piece.CORAL)),
        _factory.resetOdometry(_sideSelector.getSelected().getDirectory() + "taxi"),
        _factory.trajectoryCmd(_sideSelector.getSelected().getDirectory() + "taxi"));
  }

  public AutoRoutine onePiece() {
    AutoRoutine routine = _factory.newRoutine("One Piece");

    AutoTrajectory onePieceA =
        routine.trajectory(_sideSelector.getSelected().getDirectory() + "1pA");
    AutoTrajectory onePieceB =
        routine.trajectory(_sideSelector.getSelected().getDirectory() + "1pB");

    routine
        .active()
        .onTrue(
            sequence(
                runOnce(() -> _manipulatorPieceSetter.accept(Piece.CORAL)),
                onePieceA.resetOdometry(),
                onePieceA.cmd()));

    onePieceA
        .done()
        .onTrue(
            sequence(
                _wristevator.setGoal(L4),
                _swerve.alignToTag(Alignment.LEFT),
                waitSeconds(0.5),
                _manipulator.feed()));

    onePieceA.doneDelayed(5).onTrue(onePieceB.cmd());
    onePieceB.done().onTrue(_wristevator.setGoal(HOME));

    return routine;
  }
}
