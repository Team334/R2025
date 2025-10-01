package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Autos {
  private final AutoFactory _factory;

  private final Swerve _swerve;

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
              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });
  }

  public AutoRoutine example() {
    AutoRoutine routine = _factory.newRoutine("example");

    AutoTrajectory exampleTraj = routine.trajectory("example");

    routine.active().onTrue(Commands.sequence(exampleTraj.resetOdometry(), exampleTraj.cmd()));

    return routine;
  }
}
