package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Robot;

public class Serializer extends AdvancedSubsystem {
  private final DigitalInput _frontBeam;
  private final DigitalInput _backBeam;

  private final DIOSim _frontBeamSim;
  private final DIOSim _backBeamSim;

  public Serializer() {
    setDefaultCommand(setSpeed(0));

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    if (Robot.isSimulation()) {
      _frontBeamSim = new DIOSim(_frontBeam);
      _backBeamSim = new DIOSim(_backBeam);

      SmartDashboard.putBoolean("Front Beam Value", getBackBeam());
      SmartDashboard.putBoolean("Back Beam Value", getBackBeam());
    } else {
      _frontBeamSim = null;
      _backBeamSim = null;
    }
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }

  /** Set the speed of the feed motor in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {}).withName("Set Speed");
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
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // TODO: switch to tuning class once that's made
    _frontBeamSim.setValue(SmartDashboard.getBoolean("Front Beam Value", getFrontBeam()));
    _backBeamSim.setValue(SmartDashboard.getBoolean("Back Beam Value", getBackBeam()));
  }

  @Override
  public void close() {
    _frontBeam.close();
    _backBeam.close();
  }
}
