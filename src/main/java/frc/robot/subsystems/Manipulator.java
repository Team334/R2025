package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Manipulator extends AdvancedSubsystem {
  public Manipulator() {
    setDefaultCommand(setSpeed(0));
  }

  @Logged(name = "Speed ")
  public double getSpeed() {
    return 0;
  }

  /** Set the speed of the manipulator motor in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {});
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}
