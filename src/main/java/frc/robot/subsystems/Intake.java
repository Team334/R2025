package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Intake extends AdvancedSubsystem {

  public Intake() {
    setDefaultCommand(set(0.0, 0.0));
  }

  public Command set(double actuatorAngle, double feedSpeed) {
    return run(() -> {});
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0.0;
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return 0.0;
  }

  @Override
  public void close() throws Exception {}
}

