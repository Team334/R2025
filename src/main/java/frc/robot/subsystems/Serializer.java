package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Tuning;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Robot;

public class Serializer extends AdvancedSubsystem {
  private final DigitalInput _frontBeam;
  private final DigitalInput _backBeam;

  private DIOSim _frontBeamSim;
  private DIOSim _backBeamSim;

  private BooleanEntry _frontBeamSimValue;
  private BooleanEntry _backBeamSimValue;

  private final TalonFX _feedMotor = new TalonFX(SerializerConstants.feedMotorId);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  public Serializer() {
    setDefaultCommand(setSpeed(0));

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    if (Robot.isSimulation()) {
      _frontBeamSim = new DIOSim(_frontBeam);
      _backBeamSim = new DIOSim(_backBeam);

      _frontBeamSimValue = Tuning.entry("/Tuning/Serializer Front Beam", false);
      _backBeamSimValue = Tuning.entry("/Tuning/Serializer Back Beam", false);
    }

    var feedMotorConfigs = new TalonFXConfiguration();

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);

    FaultLogger.register(_feedMotor);
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  /** Set the speed of the front feed wheels in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {
          _feedMotor.setControl(_feedVelocitySetter.withVelocity(speed));
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

    _frontBeamSim.setValue(!_frontBeamSimValue.get());
    _backBeamSim.setValue(!_backBeamSimValue.get());
  }

  @Override
  public void close() {
    _frontBeam.close();
    _backBeam.close();

    _feedMotor.close();

    _frontBeamSimValue.close();
    _backBeamSimValue.close();
  }
}
