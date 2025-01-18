package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Manipulator extends AdvancedSubsystem {
  public Manipulator() {
    setDefaultCommand(setSpeed(0));
  }

  public Command setSpeed(double speed) {
    return run(() -> {}); // return rad/s
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0; // returns wheel speed in rad/s
  }

  @Override
  public void close() throws Exception {}
}
