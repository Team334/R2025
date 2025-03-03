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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Tuning;
import frc.robot.Constants;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.utils.SysId;
import java.util.function.Consumer;

public class Serializer extends AdvancedSubsystem {
  private final DigitalInput _frontBeam;
  private final DigitalInput _backBeam;

  private DIOSim _frontBeamSim;
  private DIOSim _backBeamSim;

  private BooleanEntry _frontBeamSimValue;
  private BooleanEntry _backBeamSimValue;

  @Logged(name = "Desired Speed")
  private double _desiredSpeed;

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

  private final Consumer<Piece> _currentPieceSetter;

  public Serializer(Consumer<Piece> currentPieceSetter) {
    setDefaultCommand(idle());

    _currentPieceSetter = currentPieceSetter;

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    SysId.displayRoutine("Serializer Feed", _serializerRoutine);

    if (Robot.isSimulation()) {
      _frontBeamSim = new DIOSim(_frontBeam);
      _backBeamSim = new DIOSim(_backBeam);

      _frontBeamSimValue = Tuning.entry("/Tuning/Serializer Front Beam", false);
      _backBeamSimValue = Tuning.entry("/Tuning/Serializer Back Beam", false);
    }

    var feedMotorConfigs = new TalonFXConfiguration();

    feedMotorConfigs.Slot0.kS = SerializerConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = SerializerConstants.feedkV.in(Volts.per(RotationsPerSecond));
    feedMotorConfigs.Slot0.kP = SerializerConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotorConfigs.Feedback.SensorToMechanismRatio = SerializerConstants.feedGearRatio;

    _feedVelocitySetter.UpdateFreqHz = 250;

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
        () -> {
          _desiredSpeed = speed;
          _feedMotor.setControl(_feedVelocitySetter.withVelocity(Units.radiansToRotations(speed)));

          // check in the hoot log when desired speed changes to 0 (when the idle command is called)
          // and check when the motor voltage changes, look at the timestamp difference to determine
          // if the
          // control request takes effect too late
          // SignalLogger.writeDouble("Desired Speed", speed);

          // check in the hoot log when this becomes true, and compare it to when desired speed
          // becomes 0 to make
          // sure it's not an issue in robot code
          // SignalLogger.writeBoolean("Coral Beam", getFrontBeam());
        });
  }

  /**
   * Whether a coral is at all inside the serializer.
   *
   * @return front beam broken OR back beam broken
   */
  public boolean hasCoral() {
    return getFrontBeam() || getBackBeam();
  }

  @Logged(name = "Front Beam")
  public boolean getFrontBeam() {
    return !_frontBeam.get();
  }

  @Logged(name = "Back Beam")
  public boolean getBackBeam() {
    return false;
    // return !_backBeam.get();
  }

  public Command idle() {
    return setSpeed(0).withName("Idle");
  }

  /** Intakes a coral until the front beam is broken. */
  public Command intake() {
    return setSpeed(SerializerConstants.feedSpeed.in(RadiansPerSecond))
        .until(this::getFrontBeam)
        .withName("Intake");
  }

  /** Outtakes a coral to the intake. */
  public Command outtake() {
    return setSpeed(-SerializerConstants.feedSpeed.in(RadiansPerSecond)).withName("Outtake");
  }

  /** Passoffs a coral to the manipulator. */
  public Command passoff() {
    return setSpeed(SerializerConstants.feedSpeed.in(RadiansPerSecond)).withName("Passoff");
  }

  /** Inverse passoff from the manipulator. */
  public Command inversePassoff() {
    return setSpeed(-SerializerConstants.feedSpeed.in(RadiansPerSecond))
        .until(this::getBackBeam)
        .andThen(Commands.runOnce(() -> _currentPieceSetter.accept(Piece.NONE)))
        .withName("Inverse Passoff");
  }

  private void setFeedVoltage(double volts) {
    _feedMotor.setControl(_feedVoltageSetter.withOutput(volts));
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
