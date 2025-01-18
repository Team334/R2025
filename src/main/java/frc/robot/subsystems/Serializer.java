package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Serializer extends AdvancedSubsystem {
  public Serializer() {
    setDefaultCommand(setSpeed(0));
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }

  /** Set the speed of the feed motor in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {});
  }

  /** Hold the coral inside the serializer. */
  public Command holdCoral() {
    return run(() -> {});
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}
