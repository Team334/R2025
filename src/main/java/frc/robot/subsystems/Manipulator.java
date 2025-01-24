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
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Tuning;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Robot;
import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final DigitalInput _beam = new DigitalInput(ManipulatorConstants.beamPort);
  private final DigitalInput _limitSwitch = new DigitalInput(ManipulatorConstants.switchPort);

  private final Trigger _beamBroken = new Trigger(this::getBeam);
  private final Trigger _switchPressed = new Trigger(this::getSwitch);

  private final Trigger _beamWithPiece = _beamBroken.and(() -> getCurrentPiece() != Piece.NONE);
  private final Trigger _beamNoPiece = _beamBroken.and(() -> getCurrentPiece() == Piece.NONE);

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

  private DIOSim _beamSim;
  private DIOSim _limitSwitchSim;

  private BooleanEntry _beamSimValue;
  private BooleanEntry _limitSwitchSimValue;

  public Manipulator(Consumer<Piece> currentPieceSetter) {
    setDefaultCommand(setSpeed(0));

    new Trigger(() -> getCurrentPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getCurrentPiece() == Piece.ALGAE).whileTrue(holdAlgae());

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();

    leftMotorConfigs.Slot0.kV = ManipulatorConstants.flywheelkV.in(VoltsPerRadianPerSecond);

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);

    _rightMotor.setControl(new Follower(ManipulatorConstants.leftMotorId, true));

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);

    _beamNoPiece.onFalse(
        Commands.runOnce(
            () -> currentPieceSetter.accept(Piece.CORAL))); // coral pick up not from passoff

    _beamWithPiece.onFalse(
        Commands.runOnce(() -> currentPieceSetter.accept(Piece.NONE))
            .onlyIf(() -> Math.signum(getSpeed()) != 1)); // any piece came only while outtaking

    _switchPressed.onTrue(
        Commands.runOnce(() -> currentPieceSetter.accept(Piece.ALGAE))); // algae picked up

    if (Robot.isSimulation()) {
      _beamSim = new DIOSim(_beam);
      _limitSwitchSim = new DIOSim(_limitSwitch);

      _beamSimValue = Tuning.entry("/Tuning/Manipulator Beam", false);
      _limitSwitchSimValue = Tuning.entry("/Tuning/Manipulator Limit Switch", false);

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

  public void startSimThread() {
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

  /** A trigger that is true when the beam is broken and current piece is not none. */
  public Trigger getBeamWithPiece() {
    return _beamWithPiece;
  }

  /** A trigger that is true when the beam is broken and current piece is none. */
  public Trigger getBeamNoPiece() {
    return _beamNoPiece;
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Beam")
  public boolean getBeam() {
    return !_beam.get();
  }

  @Logged(name = "Switch")
  public boolean getSwitch() {
    // TODO: might have two switches
    return _limitSwitch.get();
  }

  /** Set the speed of the back feed wheels in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {
          _leftMotor.setControl(_feedVelocitySetter.withVelocity(speed));
        })
        .withName("Set Speed");
  }

  /** Hold a coral in place. */
  public Command holdCoral() {
    return run(() -> {
          _leftMotor.setControl(_feedVoltageSetter.withOutput(0));
        })
        .withName("Hold Coral");
  }

  /** Hold an algae in place. */
  public Command holdAlgae() {
    return run(() -> {
          _leftMotor.setControl(
              _feedVoltageSetter.withOutput(ManipulatorConstants.holdAlgaeVoltage));
        })
        .withName("Hold Algae");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _beamSim.setValue(!_beamSimValue.get());
    _limitSwitchSim.setValue(_limitSwitchSimValue.get());
  }

  @Override
  public void close() {
    _beam.close();
    _limitSwitch.close();

    _leftMotor.close();
    _rightMotor.close();

    _simNotifier.close();

    _beamSimValue.close();
    _limitSwitchSimValue.close();
  }
}
