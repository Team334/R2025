package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Intake extends AdvancedSubsystem {
  public Intake() {
    setDefaultCommand(set(0.0, 0.0));
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0.0;
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return 0.0;
  }

  /**
   * Set the actuator angle and feed speed.
   *
   * @param actuatorAngle Actuator angle in rad.
   * @param feedSpeed Feed motor speed in rad/s.
   */
  public Command set(double actuatorAngle, double feedSpeed) {
    return run(() -> {}).withName("Set");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}
