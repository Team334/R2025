package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Robot.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Tuning;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Robot;
import frc.robot.utils.SysId;

import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final DigitalInput _coralBeam = new DigitalInput(ManipulatorConstants.coralBeam);
  private final DigitalInput _algaeBeam = new DigitalInput(ManipulatorConstants.algaeBeam);

  private final BooleanEvent _coralEvent =
      new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), this::getCoralBeam);
  private final BooleanEvent _algaeEvent =
      new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), this::getAlgaeBeam);

  private final Consumer<Piece> _currentPieceSetter;

  private final TalonFX _leftMotor =
      new TalonFX(ManipulatorConstants.leftMotorId, Constants.canivore);
  private final TalonFX _rightMotor =
      new TalonFX(ManipulatorConstants.rightMotorId, Constants.canivore);

  private FlywheelSim _leftFlywheelSim;

  private Notifier _simNotifier;

  private double _lastSimTime;

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _leftMotor.getVelocity();

  private final SysIdRoutine _feedRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setFeedVoltage(volts.in(Volts)), null, this));

  private DIOSim _coralBeamSim;
  private DIOSim _algaeBeamSim;

  private BooleanEntry _coralBeamSimValue;
  private BooleanEntry _algaeBeamSimValue;

  public Manipulator(Consumer<Piece> currentPieceSetter) {
    setDefaultCommand(idle());

    _currentPieceSetter = currentPieceSetter;

    new Trigger(() -> getCurrentPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getCurrentPiece() == Piece.ALGAE).whileTrue(holdAlgae());

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();

    leftMotorConfigs.Slot0.kV = ManipulatorConstants.flywheelkV.in(Volts.per(RotationsPerSecond));

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);

    _rightMotor.setControl(new Follower(ManipulatorConstants.leftMotorId, true));

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);

    SysId.displayRoutine("Manipulator Feed", _feedRoutine);

    if (Robot.isSimulation()) {
      _coralBeamSim = new DIOSim(_coralBeam);
      _algaeBeamSim = new DIOSim(_algaeBeam);

      _coralBeamSimValue = Tuning.entry("/Tuning/Manipulator Coral Beam", false);
      _algaeBeamSimValue = Tuning.entry("/Tuning/Manipulator Algae Beam", false);

      _leftFlywheelSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getKrakenX60(1), 0.001, ManipulatorConstants.flywheelGearRatio),
              DCMotor.getKrakenX60(2));

      startSimThread();
    }
  }

  /** Represents a possible game piece in the manipulator. */
  public static enum Piece {
    CORAL,
    ALGAE,
    NONE
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              final double batteryVoltage = RobotController.getBatteryVoltage();

              var leftMotorSimState = _leftMotor.getSimState();

              leftMotorSimState.setSupplyVoltage(batteryVoltage);

              _leftFlywheelSim.setInputVoltage(
                  leftMotorSimState.getMotorVoltageMeasure().in(Volts));
              _leftFlywheelSim.update(deltaTime);

              leftMotorSimState.setRotorVelocity(
                  _leftFlywheelSim.getAngularVelocity().in(RotationsPerSecond)
                      * ManipulatorConstants.flywheelGearRatio);

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Manipulator Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
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
          _leftMotor.setControl(_feedVelocitySetter.withVelocity(Units.radiansToRotations(speed)));
        });
  }

  /** Sets the current piece when the coral beam changes state. */
  private Command watchCoralBeam(Piece piece, boolean onTrue) {
    BooleanEvent coralEvent = onTrue ? _coralEvent.rising() : _coralEvent.falling();

    return Commands.run(
        () -> {
          if (coralEvent.getAsBoolean()) _currentPieceSetter.accept(piece);
        });
  }

  /** Sets the current piece when the algae beam changes state. */
  private Command watchAlgaeBeam(Piece piece, boolean onTrue) {
    BooleanEvent algaeEvent = onTrue ? _algaeEvent.rising() : _algaeEvent.falling();

    return Commands.run(
        () -> {
          if (algaeEvent.getAsBoolean()) _currentPieceSetter.accept(piece);
        });
  }

  /** Idle the manipulator. */
  public Command idle() {
    return setSpeed(0).withName("Idle");
  }

  /** Hold a coral in place. */
  public Command holdCoral() {
    return run(() -> {
          _leftMotor.setControl(_feedVoltageSetter.withOutput(0));
        })
        .alongWith(watchCoralBeam(Piece.NONE, false))
        .withName("Hold Coral");
  }

  /** Hold an algae in place. */
  public Command holdAlgae() {
    return run(() -> {
          _leftMotor.setControl(
              _feedVoltageSetter.withOutput(ManipulatorConstants.holdAlgaeVoltage));
        })
        .alongWith(watchAlgaeBeam(Piece.NONE, false))
        .withName("Hold Algae");
  }

  /** General intake. */
  public Command intake() {
    return setSpeed(ManipulatorConstants.feedSpeed.in(RadiansPerSecond))
        .alongWith(watchCoralBeam(Piece.CORAL, true), watchAlgaeBeam(Piece.ALGAE, true))
        .withName("Intake");
  }

  /** General outtake. */
  public Command outtake() {
    return setSpeed(-ManipulatorConstants.feedSpeed.in(RadiansPerSecond))
        .alongWith(watchCoralBeam(Piece.NONE, false), watchAlgaeBeam(Piece.NONE, false))
        .withName("Outtake");
  }

  /** Passoff from the serializer. */
  public Command passoff() {
    return setSpeed(0)
        .alongWith(watchCoralBeam(Piece.CORAL, false))
        .until(() -> getCurrentPiece() == Piece.CORAL)
        .finallyDo(this::pulse)
        .withName("Passoff");
  }

  /** Inverse passoff into the serializer. */
  public Command inversePassoff() {
    return setSpeed(0).withName("Inverse Passoff");
  }

  /** Pulse the manipulator until coral triggers the beam */
  public Command pulse() {
    return setSpeed(ManipulatorConstants.feedSpeed.div(2).in(RadiansPerSecond))
        .until(() -> _coralEvent.rising().getAsBoolean());
  }

  private void setFeedVoltage(double volts) {
    _leftMotor.setControl(_feedVoltageSetter.withOutput(volts));
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _coralBeamSim.setValue(!_coralBeamSimValue.get());
    _algaeBeamSim.setValue(!_algaeBeamSimValue.get());
  }

  @Override
  public void close() {
    _coralBeam.close();
    _algaeBeam.close();

    _leftMotor.close();
    _rightMotor.close();

    _simNotifier.close();

    _coralBeamSimValue.close();
    _algaeBeamSimValue.close();
  }
}
