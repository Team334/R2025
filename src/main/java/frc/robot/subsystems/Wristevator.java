package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.WristevatorConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Wristevator extends AdvancedSubsystem {
  /** A setpoint for the wristevator. */
  public static enum WristevatorSetpoint {
    HOME(Radians.of(0), Meters.of(0)),
    HUMAN(Radians.of(0), Meters.of(0)),
    PROCESSOR(Radians.of(0), Meters.of(0)),

    L1(Radians.of(0), Meters.of(0)),
    L2(Radians.of(0), Meters.of(0)),
    L3(Radians.of(0), Meters.of(0)),
    L4(Radians.of(0), Meters.of(0)),

    LOWER_ALGAE(Radians.of(0), Meters.of(0)),
    UPPER_ALGAE(Radians.of(0), Meters.of(0));

    private final Angle _angle;
    private final Distance _height;

    private WristevatorSetpoint(Angle angle, Distance height) {
      _angle = angle;
      _height = height;
    }

    public Angle getAngle() {
      return _angle;
    }

    public Distance getHeight() {
      return _height;
    }
  }

  private final Mechanism2d _mech = new Mechanism2d(1.35, 2);
  private final MechanismRoot2d _root = _mech.getRoot("elevator mount", 1, 0.1);

  private final MechanismLigament2d _elevator =
      _root.append(new MechanismLigament2d("elevator", 0, 90, 3, new Color8Bit(Color.kBlack)));

  private final MechanismLigament2d _wrist =
      _elevator.append(
          new MechanismLigament2d("wrist", 0.3, -90, 3, new Color8Bit(Color.kBlueViolet)));

  private final TalonFX _leftMotor =
      new TalonFX(WristevatorConstants.leftMotorId, Constants.canivore);
  private final TalonFX _rightMotor =
      new TalonFX(WristevatorConstants.rightMotorId, Constants.canivore);
  private final TalonFX _wristMotor =
      new TalonFX(WristevatorConstants.wristMotorId, Constants.canivore);

  StatusSignal<Angle> _heightGetter = _leftMotor.getPosition();
  StatusSignal<Angle> _angleGetter = _wristMotor.getPosition();

  private VelocityVoltage _elevatorVelocitySetter = new VelocityVoltage(0);
  private VelocityVoltage _wristVelocitySetter = new VelocityVoltage(0);

  private final DigitalInput _homeSwitch = new DigitalInput(WristevatorConstants.homeSwitch);

  private final DIOSim _homeSwitchSim;

  public Wristevator() {
    if (Robot.isSimulation()) {
      _homeSwitchSim = new DIOSim(_homeSwitch);
    } else {
      _homeSwitchSim = null;
    }

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();

    _leftMotor.getConfigurator().apply(leftMotorConfigs);
    _rightMotor.getConfigurator().apply(rightMotorConfigs);

    _rightMotor.setControl(new Follower(WristevatorConstants.leftMotorId, true));
  }

  @Logged(name = "Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Rotations)
        * WristevatorConstants.drumCircumference.in(Meters);
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return _angleGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Home Switch")
  public boolean homeSwitch() {
    return _homeSwitch.get();
  }

  /** Set the wristevator to a setpoint. */
  public Command setSetpoint(WristevatorSetpoint setpoint) {
    return run(() -> {})
        .beforeStarting(() -> DogLog.log("Wristevator/Setpoint", setpoint.toString()))
        .withName("Set Setpoint: " + setpoint.toString());
  }

  /** Control the elevator and wrist speeds individually. */
  public Command setSpeeds(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
    return run(() -> {
          _leftMotor.setControl(_elevatorVelocitySetter.withVelocity(elevatorSpeed.getAsDouble()));
          _wristMotor.setControl(_wristVelocitySetter.withVelocity(wristSpeed.getAsDouble()));
        })
        .withName("Set Speeds");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    _homeSwitchSim.setValue(getHeight() == 0);

    _elevator.setLength(getHeight());
    _wrist.setAngle(Math.toDegrees(getAngle()) - 90);

    SmartDashboard.putData("Wristevator Visualizer", _mech);
  }

  @Override
  public void close() {}
}
