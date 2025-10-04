package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class Autos {
  private final AutoFactory _factory;

  private final Swerve _swerve;
  private final Intake _intake;

  public Autos(Swerve swerve, Intake intake) {
    _swerve = swerve;
    _intake = intake;

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

  public AutoRoutine shortPath() {
    AutoRoutine routine = _factory.newRoutine("shortPath");

    AutoTrajectory shortPath = routine.trajectory("shortPath");

    routine.active().onTrue(sequence(shortPath.resetOdometry(), shortPath.cmd()));

    return routine;
  }

  public AutoRoutine forwardIntakeRight() {
    AutoRoutine routine = _factory.newRoutine("forwardIntakeRight");

    // Load the routine's trajectories
    AutoTrajectory forwardMeter = routine.trajectory("forwardMeter");
    AutoTrajectory rightMeter = routine.trajectory("rightMeter");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(sequence(forwardMeter.resetOdometry(), forwardMeter.cmd()));

    forwardMeter.done().onTrue(_intake.intake().withTimeout(3).andThen(rightMeter.cmd()));

    return routine;
  }
}
