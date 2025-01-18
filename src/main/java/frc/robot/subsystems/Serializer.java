package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class Serializer extends AdvancedSubsystem {
  @Override
  public void close() throws Exception {}

  public Serializer() {
    setDefaultCommand(setSpeed(0));
  }

  public Command setSpeed(double speed) {
    return run(() -> {});
  }

  public void holdCoral() {}

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }
}
