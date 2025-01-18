package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import java.util.function.DoubleSupplier;

public class Wristevator extends AdvancedSubsystem {
  /** A setpoint for the wristevator. */
  public static enum WristevatorSetpoint {
    HOME(Radians.of(0), Meters.of(0)),
    HUMAN(Radians.of(0), Meters.of(0)),
    L1(Radians.of(0), Meters.of(0)),
    L2(Radians.of(0), Meters.of(0)),
    L3(Radians.of(0), Meters.of(0)),
    L4(Radians.of(0), Meters.of(0));

    private final Angle _angle;
    private final Distance _height;

    private WristevatorSetpoint(Angle angle, Distance height) {
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

  @Logged(name = "Height")
  public double getHeight() {
    return 0;
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return 0;
  }

  /** Set the wristevator to a setpoint. */
  public Command setSetpoint(WristevatorSetpoint elevatorSetpoint) {
    return run(() -> {});
  }

  /** Control the elevator and wrist speeds individually. */
  public Command setSpeeds(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
    return run(() -> {});
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}
