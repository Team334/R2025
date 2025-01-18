package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import java.util.function.DoubleSupplier;

public class Wristevator extends AdvancedSubsystem {

  public static enum ElevatorSetpoint {
    HOME(Radians.of(0), Meters.of(0)),
    HUMAN(Radians.of(0), Meters.of(0)),
    L1(Radians.of(0), Meters.of(0)),
    L2(Radians.of(0), Meters.of(0)),
    L3(Radians.of(0), Meters.of(0)),
    L4(Radians.of(0), Meters.of(0));

    private final Angle _angle;
    private final Distance _height;

    private ElevatorSetpoint(Angle angle, Distance height) {
      _angle = angle;
      _height = height;
    }

    public Angle getAngle() {
      return _angle;
    }

    public Distance getHeight() {
      return _height;
    }
  }

  public Wristevator() {}

  public Command setSetpoint(ElevatorSetpoint elevatorSetpoint) {
    return run(() -> {});
  }

  public Command setSpeeds(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
    return run(() -> {});
  }

  @Logged(name = "Height")
  public double getHeight() {
    return 0;
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
