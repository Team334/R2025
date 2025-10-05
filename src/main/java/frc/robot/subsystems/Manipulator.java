package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;
import static frc.robot.Robot.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.Piece;
import frc.robot.Robot;
import frc.robot.utils.SysId;
import java.util.Map;
import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final DigitalInput _coralBeam = new DigitalInput(ManipulatorConstants.coralBeam);
  private final DigitalInput _algaeBeam = new DigitalInput(ManipulatorConstants.algaeBeam);

  private final BooleanEvent _coralEvent =
      new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), this::getCoralBeam);
  private final BooleanEvent _algaeEvent =
      new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), this::getAlgaeBeam);

  private final TalonFX _leftMotor =
      new TalonFX(ManipulatorConstants.leftMotorId, Constants.canivore);
  private final TalonFX _rightMotor =
      new TalonFX(ManipulatorConstants.rightMotorId, Constants.canivore);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _leftMotor.getVelocity();

  private final SysIdRoutine _leftRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setFlywheelVoltage(volts.in(Volts), _leftMotor), null, this));

  private final SysIdRoutine _rightRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setFlywheelVoltage(volts.in(Volts), _rightMotor), null, this));

  private DIOSim _coralBeamSim;
  private DIOSim _algaeBeamSim;

  private BooleanSubscriber _coralBeamState;
  private BooleanSubscriber _algaeBeamState;

  private final Consumer<Piece> _manipulatorPieceSetter;

  public Manipulator(Consumer<Piece> manipulatorPieceSetter) {
    setDefaultCommand(idle());

    _manipulatorPieceSetter = manipulatorPieceSetter;

    new Trigger(() -> getManipulatorPiece() == Piece.NONE).whileTrue(idle());
    new Trigger(() -> getManipulatorPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getManipulatorPiece() == Piece.ALGAE).whileTrue(holdAlgae());

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();

    leftMotorConfigs.Slot0.kS = ManipulatorConstants.leftFlywheelkS.in(Volts);
    leftMotorConfigs.Slot0.kV =
        ManipulatorConstants.leftFlywheelkV.in(Volts.per(RotationsPerSecond));

    leftMotorConfigs.Slot0.kP =
        ManipulatorConstants.leftFlywheelkP.in(Volts.per(RotationsPerSecond));

    leftMotorConfigs.Feedback.SensorToMechanismRatio = ManipulatorConstants.flywheelGearRatio;

    leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightMotorConfigs.Slot0.kS = ManipulatorConstants.rightFlywheelkS.in(Volts);
    rightMotorConfigs.Slot0.kV =
        ManipulatorConstants.rightFlywheelkV.in(Volts.per(RotationsPerSecond));

    rightMotorConfigs.Slot0.kP =
        ManipulatorConstants.rightFlywheelkP.in(Volts.per(RotationsPerSecond));

    rightMotorConfigs.Feedback.SensorToMechanismRatio = ManipulatorConstants.flywheelGearRatio;

    rightMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);

    CTREUtil.attempt(() -> _leftMotor.optimizeBusUtilization(), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.optimizeBusUtilization(), _rightMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _leftMotor.getPosition(),
                _leftMotor.getVelocity(),
                _leftMotor.getMotorVoltage()),
        _leftMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _rightMotor.getPosition(),
                _rightMotor.getVelocity(),
                _rightMotor.getMotorVoltage()),
        _rightMotor);

    _feedVelocitySetter.UpdateFreqHz = 250;

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);

    SysId.displayRoutine("Manipulator Left Flywheel", _leftRoutine);
    SysId.displayRoutine("Manipulator Right Flywheel", _rightRoutine);

    if (Robot.isSimulation()) {
      _coralBeamSim = new DIOSim(_coralBeam);
      _algaeBeamSim = new DIOSim(_algaeBeam);

      _coralBeamState = DogLog.tunable("Manipulator/Coral Beam State", false);
      _algaeBeamState = DogLog.tunable("Manipulator/Algae Beam State", false);
    }
  }

  private void setFlywheelVoltage(double volts, TalonFX motor) {
    motor.setControl(_feedVoltageSetter.withOutput(volts));
  }

  @Logged(name = "Coral Beam")
  public boolean getCoralBeam() {
    return !_coralBeam.get();
  }

  @Logged(name = "Algae Beam")
  public boolean getAlgaeBeam() {
    return !_algaeBeam.get();
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  // set the speed of the back feed wheels in rad/s
  private Command setSpeed(double speed) {
    return run(
        () -> {
          _feedVelocitySetter.Velocity = Units.radiansToRotations(speed);

          _leftMotor.setControl(_feedVelocitySetter);
          _rightMotor.setControl(_feedVelocitySetter);
        });
  }

  /** Sets the current piece when the coral beam changes state. */
  private Command watchCoralBeam(Piece piece, boolean onTrue) {
    BooleanEvent coralEvent = onTrue ? _coralEvent.rising() : _coralEvent.falling();

    return Commands.run(
        () -> {
          if (coralEvent.getAsBoolean()) _manipulatorPieceSetter.accept(piece);
        });
  }

  /** Sets the current piece when the algae beam changes state. */
  private Command watchAlgaeBeam(Piece piece, boolean onTrue) {
    BooleanEvent algaeEvent = onTrue ? _algaeEvent.rising() : _algaeEvent.falling();

    return Commands.run(
        () -> {
          if (algaeEvent.getAsBoolean()) _manipulatorPieceSetter.accept(piece);
        });
  }

  public Command idle() {
    return setSpeed(0).withName("Idle");
  }

  /** Hold coral in place. */
  public Command holdCoral() {
    return idle().alongWith(watchCoralBeam(Piece.NONE, false)).withName("Hold Coral");
  }

  /** Hold algae in place. */
  public Command holdAlgae() {
    return run(() -> {
          _feedVoltageSetter.Output = ManipulatorConstants.holdAlgaeVoltage.in(Volts);

          _leftMotor.setControl(_feedVoltageSetter);
          _rightMotor.setControl(_feedVoltageSetter);
        })
        .alongWith(watchAlgaeBeam(Piece.NONE, false))
        .withName("Hold Algae");
  }

  /** Feeds in the proper direction depending on the wristevator goal. */
  public Command feed() {
    return Commands.select(
            Map.ofEntries(
                Map.entry(L1, feedOut(ManipulatorConstants.coralOuttakeSpeed)),
                Map.entry(L2, feedOut(ManipulatorConstants.coralOuttakeSpeed)),
                Map.entry(L3, feedOut(ManipulatorConstants.coralOuttakeSpeed)),
                Map.entry(L4, feedIn(ManipulatorConstants.coralIntakeSpeed)),
                Map.entry(LOWER_ALGAE, feedIn(ManipulatorConstants.algaeIntakeSpeed)),
                Map.entry(UPPER_ALGAE, feedIn(ManipulatorConstants.algaeIntakeSpeed)),
                Map.entry(PROCESSOR, feedOut(ManipulatorConstants.algaeOuttakeSpeed)),
                Map.entry(HOME, feedOut(ManipulatorConstants.coralOuttakeSpeed)),
                Map.entry(HUMAN, feedIn(ManipulatorConstants.humanIntakeSpeed))),
            () -> getWristevatorGoal())
        .withName("Feed");
  }

  /** Spin wheels inwards and change the current piece. */
  private Command feedIn(AngularVelocity speed) {
    return setSpeed(speed.in(RadiansPerSecond))
        .alongWith(
            watchCoralBeam(Piece.CORAL, true),
            watchAlgaeBeam(Piece.ALGAE, true),
            watchCoralBeam(Piece.NONE, false));
  }

  /** Spin wheels outwards and change the current piece. */
  private Command feedOut(AngularVelocity speed) {
    return setSpeed(speed.in(RadiansPerSecond))
        .alongWith(watchCoralBeam(Piece.NONE, false), watchAlgaeBeam(Piece.NONE, false));
  }

  /** Passoff from the serializer. */
  // public Command passoff() {
  //     BooleanEvent coralEventFalling = _coralEvent.falling();

  //     return setSpeed(ManipulatorConstants.passoffSpeed.unaryMinus().in(RadiansPerSecond))
  //         .until(coralEventFalling::getAsBoolean)
  //         .andThen(
  //             setSpeed(ManipulatorConstants.passoffSpeed.in(RadiansPerSecond))
  //                 .alongWith(watchCoralBeam(Piece.CORAL, true)))
  //         .withName("Passoff");
  // }

  /** Passoff from the serializer. */
  public Command passoff() {
    return setSpeed(ManipulatorConstants.passoffSpeed.unaryMinus().in(RadiansPerSecond))
        .until(_coralEvent.falling()::getAsBoolean)
        .andThen(feedIn(ManipulatorConstants.passoffSpeed))
        .withName("Passoff");
  }

  // /** Inverse passoff into the serializer. */
  // public Command inversePassoff() {
  //     return setSpeed(ManipulatorConstants.passoffSpeed.in(RadiansPerSecond))
  //         .withName("Inverse Passoff");
  // }

  /** Inverse passoff into the serializer. */
  public Command inversePassoff() {
    return feedIn(ManipulatorConstants.passoffSpeed).withName("Inverse Passoff");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _coralBeamSim.setValue(!_coralBeamState.get());
    _algaeBeamSim.setValue(!_algaeBeamState.get());
  }

  @Override
  public void close() {
    _coralBeam.close();
    _algaeBeam.close();

    _leftMotor.close();
    _rightMotor.close();

    _coralBeamState.close();
    _algaeBeamState.close();
  }
}
