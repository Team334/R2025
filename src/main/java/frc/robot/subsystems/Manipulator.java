package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Robot.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Robot;
import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final Consumer<Piece> _currentPieceSetter;

  private final DigitalInput _beam = new DigitalInput(ManipulatorConstants.beamPort);
  private final DigitalInput _limitSwitch = new DigitalInput(ManipulatorConstants.switchPort);

  private final DIOSim _beamSim;
  private final DIOSim _limitSwitchSim;

  private final Trigger _beamBroken = new Trigger(this::getBeam);
  private final Trigger _switchPressed = new Trigger(this::getSwitch);

  private final Trigger _beamWithPiece = _beamBroken.and(() -> getCurrentPiece() != Piece.NONE);
  private final Trigger _beamNoPiece = _beamBroken.and(() -> getCurrentPiece() == Piece.NONE);

  private final TalonFX _leftMotor =
      new TalonFX(ManipulatorConstants.leftMotorId, Constants.canivore);
  private final TalonFX _rightMotor =
      new TalonFX(ManipulatorConstants.rightMotorId, Constants.canivore);

  private VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> _speedGetter = _leftMotor.getVelocity();

  public Manipulator(Consumer<Piece> currentPieceSetter) {
    _currentPieceSetter = currentPieceSetter;

    setDefaultCommand(setSpeed(0));

    if (Robot.isSimulation()) {
      _beamSim = new DIOSim(_beam);
      _limitSwitchSim = new DIOSim(_limitSwitch);

      SmartDashboard.putBoolean("Beam Value", getBeam());
      SmartDashboard.putBoolean("Limit Switch Value", getSwitch());
    } else {
      _beamSim = null;
      _limitSwitchSim = null;
    }

    new Trigger(() -> getCurrentPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getCurrentPiece() == Piece.ALGAE).whileTrue(holdAlgae());

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();

    _leftMotor.getConfigurator().apply(leftMotorConfigs);
    _rightMotor.getConfigurator().apply(rightMotorConfigs);

    _rightMotor.setControl(new Follower(ManipulatorConstants.leftMotorId, true));

    _beamNoPiece.onFalse(
        Commands.runOnce(
            () -> currentPieceSetter.accept(Piece.CORAL))); // coral pick up not from passoff

    _beamWithPiece.onFalse(
        Commands.runOnce(() -> currentPieceSetter.accept(Piece.NONE))); // any piece came out

    _switchPressed.onTrue(
        Commands.runOnce(() -> currentPieceSetter.accept(Piece.ALGAE))); // algae picked up
  }

  public static enum Piece {
    CORAL,
    ALGAE,
    NONE
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
    return _speedGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Manipulator Beam")
  public boolean getBeam() {
    return !_beam.get();
  }

  @Logged(name = "Manipulator Switch")
  public boolean getSwitch() {
    // TODO: might have two switches
    return _limitSwitch.get();
  }

  /** Set the speed of the back manipulator wheels in rad/s. */
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

    _beamSim.setValue(!SmartDashboard.getBoolean("Beam Value", getBeam()));
    _limitSwitchSim.setValue(SmartDashboard.getBoolean("Limit Switch Value", getSwitch()));
  }

  @Override
  public void close() {}
}
