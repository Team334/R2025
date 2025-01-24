package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
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
      _root.append(new MechanismLigament2d("elevator", 0, 90, 3, new Color8Bit(Color.kCyan)));

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

  private DIOSim _homeSwitchSim;

  private ElevatorSim _elevatorSim;
  private SingleJointedArmSim _wristSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Wristevator() {
    if (Robot.isSimulation()) {
      _homeSwitchSim = new DIOSim(_homeSwitch);

      _elevatorSim =
          new ElevatorSim(
            DCMotor.getKrakenX60(2),
            WristevatorConstants.elevatorGearRatio,
            Units.lbsToKilograms(9.398),
            WristevatorConstants.drumRadius.in(Meters),
            0,
            WristevatorConstants.maxElevatorHeight.in(Meters),
            false,
            0
          );

      _wristSim =
          new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            WristevatorConstants.wristGearRatio,
            SingleJointedArmSim.estimateMOI(WristevatorConstants.manipulatorLength.in(Meters), Units.lbsToKilograms(8.155)),
            WristevatorConstants.manipulatorLength.in(Meters),
            WristevatorConstants.minWristAngle.in(Radians),
            WristevatorConstants.maxWristAngle.in(Radians),
            false,
            0
          );

      startSimThread();
    }

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();
    var wristMotorConfigs = new TalonFXConfiguration();

    leftMotorConfigs.Slot0.kV = WristevatorConstants.elevatorkV.in(VoltsPerRadianPerSecond);
    leftMotorConfigs.Slot0.kA = WristevatorConstants.elevatorkA.in(VoltsPerRadianPerSecondSquared);

    leftMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.elevatorGearRatio;

    wristMotorConfigs.Slot0.kV = WristevatorConstants.wristkV.in(VoltsPerRadianPerSecond);
    wristMotorConfigs.Slot0.kA = WristevatorConstants.wristkA.in(VoltsPerRadianPerSecondSquared);

    wristMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.wristGearRatio;

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);
    CTREUtil.attempt(() -> _wristMotor.getConfigurator().apply(wristMotorConfigs), _wristMotor);

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);
    FaultLogger.register(_wristMotor);

    _rightMotor.setControl(new Follower(WristevatorConstants.leftMotorId, true));
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              var leftMotorSimState = _leftMotor.getSimState();
              var rightMotorSimState = _rightMotor.getSimState();
              var wristMotorSimState = _wristMotor.getSimState();

              leftMotorSimState.setSupplyVoltage(batteryVolts);
              rightMotorSimState.setSupplyVoltage(batteryVolts);
              wristMotorSimState.setSupplyVoltage(batteryVolts);

              _elevatorSim.setInputVoltage(leftMotorSimState.getMotorVoltageMeasure().in(Volts));
              _wristSim.setInputVoltage(wristMotorSimState.getMotorVoltageMeasure().in(Volts));

              _elevatorSim.update(deltaTime);
              _wristSim.update(deltaTime);

              // raw rotor positions
              leftMotorSimState.setRawRotorPosition(
                  _elevatorSim.getPositionMeters()
                      / WristevatorConstants.drumCircumference.in(Meters)
                      * WristevatorConstants.elevatorGearRatio);
              rightMotorSimState.setRawRotorPosition(
                  -_elevatorSim.getPositionMeters()
                      / WristevatorConstants.drumCircumference.in(Meters)
                      * WristevatorConstants.elevatorGearRatio);
              wristMotorSimState.setRawRotorPosition(
                  Units.radiansToRotations(_wristSim.getAngleRads() * WristevatorConstants.wristGearRatio));

              // raw rotor velocities
              leftMotorSimState.setRotorVelocity(
                  _elevatorSim.getVelocityMetersPerSecond()
                      / WristevatorConstants.drumCircumference.in(Meters)
                      * WristevatorConstants.elevatorGearRatio);
              rightMotorSimState.setRotorVelocity(
                  -_elevatorSim.getVelocityMetersPerSecond()
                      / WristevatorConstants.drumCircumference.in(Meters)
                      * WristevatorConstants.elevatorGearRatio);
              wristMotorSimState.setRotorVelocity(
                  Units.radiansToRotations(_wristSim.getVelocityRadPerSec() * WristevatorConstants.wristGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Wristevator Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
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

  /**
   * Control the elevator and wrist speeds individually.
   * 
   * @param elevatorSpeed The elevator drum speed in radians per second.
   * @param wristSpeed The wrist speed in radians per second.
   */
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
  public void close() {
    _leftMotor.close();
    _rightMotor.close();
    _wristMotor.close();

    _simNotifier.close();
  }
}
