package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants.SerializerConstants;

public class Serializer extends AdvancedSubsystem {
  private DigitalInput _frontBeam;
  private DigitalInput _backBeam;

  public Serializer() {
    setDefaultCommand(setSpeed(0));

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    new Trigger(this::getFrontBeam).whileTrue(holdCoral());
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }

  /** Set the speed of the feed motor in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {}).withName("Set Speed");
  }

  /** Hold the coral inside the serializer. */
  public Command holdCoral() {
    return run(() -> {}).withName("Hold Coral");
  }

  public boolean getFrontBeam() {
    return !_frontBeam.get();
  }

  public boolean getBackBeam() {
    return !_backBeam.get();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}
