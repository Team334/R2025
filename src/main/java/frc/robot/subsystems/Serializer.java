package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Robot;
import frc.robot.utils.SysId;

public class Serializer extends AdvancedSubsystem {
  private final DigitalInput _coralBeam;

  private DIOSim _coralBeamSim;
  private BooleanSubscriber _coralBeamState;

  private final TalonFX _feedMotor =
      new TalonFX(SerializerConstants.feedMotorId, Constants.canivore);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final SysIdRoutine _serializerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setFeedVoltage(volts.in(Volts)), null, this));

  public Serializer() {
    setDefaultCommand(idle());

    _coralBeam = new DigitalInput(SerializerConstants.coralBeamPort);

    SysId.displayRoutine("Serializer Feed", _serializerRoutine);

    if (Robot.isSimulation()) {
      _coralBeamSim = new DIOSim(_coralBeam);
      _coralBeamState = DogLog.tunable("Serializer/Coral Beam State", false);
    }

    var feedMotorConfigs = new TalonFXConfiguration();

    feedMotorConfigs.Slot0.kS = SerializerConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = SerializerConstants.feedkV.in(Volts.per(RotationsPerSecond));
    feedMotorConfigs.Slot0.kP = SerializerConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotorConfigs.Feedback.SensorToMechanismRatio = SerializerConstants.feedGearRatio;

    _feedVelocitySetter.UpdateFreqHz =
        250; // to quickly stop the serializer once the coral beam is broken

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);

    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _feedMotor.getPosition(),
                _feedMotor.getVelocity(),
                _feedMotor.getMotorVoltage()),
        _feedMotor);

    FaultLogger.register(_feedMotor);
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  // Set the speed of the front feed wheels in rad/s.
  private Command setSpeed(double speed) {
    return run(
        () ->
            _feedMotor.setControl(
                _feedVelocitySetter.withVelocity(Units.radiansToRotations(speed))));
  }

  @Logged(name = "Coral Beam")
  public boolean getCoralBeam() {
    return !_coralBeam.get();
  }

  public Command idle() {
    return setSpeed(0).withName("Idle");
  }

  /** Intakes a coral until the coral beam is broken. */
  public Command intake() {
    return setSpeed(SerializerConstants.feedSpeed.in(RadiansPerSecond))
        .until(this::getCoralBeam)
        .withName("Intake");
  }

  /** Outtakes a coral to the intake. */
  public Command outtake() {
    return setSpeed(SerializerConstants.feedSpeed.unaryMinus().in(RadiansPerSecond))
        .withName("Outtake");
  }

  /** Passoffs a coral to the manipulator. */
  public Command passoff() {
    return setSpeed(SerializerConstants.passoffSpeed.in(RadiansPerSecond)).withName("Passoff");
  }

  /** Inverse passoff from the manipulator. */
  public Command inversePassoff() {
    return setSpeed(SerializerConstants.passoffSpeed.unaryMinus().in(RadiansPerSecond))
        .withName("Inverse Passoff");
  }

  private void setFeedVoltage(double volts) {
    _feedMotor.setControl(_feedVoltageSetter.withOutput(volts));
  }

  @Override
  public void periodic() {
    DogLog.time("Time/Serializer/periodic()");

    super.periodic();

    DogLog.timeEnd("Time/Serializer/periodic()");
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _coralBeamSim.setValue(!_coralBeamState.get());
  }

  @Override
  public void close() {
    _feedMotor.close();

    _coralBeam.close();
    _coralBeamState.close();
  }
}
