package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private final TalonFX _frontBeamMotor;
  private final TalonFX _backBeamMotor;
  private final VoltageOut m_request = new VoltageOut(0);

  public Serializer() {
    setDefaultCommand(setSpeed(0));

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    _frontBeamMotor = new TalonFX(SerializerConstants.frontBeamMotorId);
    _backBeamMotor = new TalonFX(SerializerConstants.backBeamMotorId);

    // Apply default configurations
    _frontBeamMotor.getConfigurator().apply(new TalonFXConfiguration());
    _backBeamMotor.getConfigurator().apply(new TalonFXConfiguration());

    // Oppose master because you want inverse direction
    _backBeamMotor.setControl(new Follower(_frontBeamMotor.getDeviceID(), true));

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
    return _backBeamMotor.getVelocity().getValue().in(RadiansPerSecond);
  }

  /** Set the speed of the front feed wheels in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {
          _frontBeamMotor.setControl(m_request.withOutput(SerializerConstants.serializerVolts));
        })
        .withName("Set Speed");
  }

  @Logged(name = "Front Beam")
  public boolean getFrontBeam() {
    return !_frontBeam.get();
  }

  @Logged(name = "Back Beam")
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
    _frontBeamSim.setValue(!SmartDashboard.getBoolean("Front Beam Value", getFrontBeam()));
    _backBeamSim.setValue(!SmartDashboard.getBoolean("Back Beam Value", getBackBeam()));
  }

  @Override
  public void close() {
    _frontBeam.close();
    _backBeam.close();
  }
}
