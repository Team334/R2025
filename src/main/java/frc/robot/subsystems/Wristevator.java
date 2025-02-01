package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
import frc.robot.Constants.WristevatorConstants.Intermediate;
import frc.robot.Constants.WristevatorConstants.Setpoint;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Wristevator extends AdvancedSubsystem {
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

  private final StatusSignal<Angle> _heightGetter = _leftMotor.getPosition();
  private final StatusSignal<Angle> _angleGetter = _wristMotor.getPosition();

  private final DynamicMotionMagicVoltage _heightSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);
  private final DynamicMotionMagicVoltage _angleSetter = new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private final StatusSignal<AngularVelocity> _elevatorVelocityGetter = _leftMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _wristVelocityGetter = _wristMotor.getVelocity();

  private final VelocityVoltage _elevatorVelocitySetter = new VelocityVoltage(0);
  private final VelocityVoltage _wristVelocitySetter = new VelocityVoltage(0);

  private final TrapezoidProfile _elevatorMaxProfile =
      new TrapezoidProfile(
          new Constraints(
              WristevatorConstants.maxElevatorSpeed.in(RadiansPerSecond),
              WristevatorConstants.maxElevatorAcceleration.in(RadiansPerSecondPerSecond)));

  private final TrapezoidProfile _wristMaxProfile =
      new TrapezoidProfile(
          new Constraints(
              WristevatorConstants.maxWristSpeed.in(RadiansPerSecond),
              WristevatorConstants.maxWristAcceleration.in(RadiansPerSecondPerSecond)));

  private final DigitalInput _homeSwitch = new DigitalInput(WristevatorConstants.homeSwitch);

  private Setpoint _prevSetpoint = HOME;
  private Setpoint _nextSetpoint = HOME;

  @Logged(name = "Is Manual")
  private boolean _isManual = false;

  private DIOSim _homeSwitchSim;

  private ElevatorSim _elevatorSim;
  private SingleJointedArmSim _wristSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Wristevator() {
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

    setDefaultCommand(idle());

    if (Robot.isSimulation()) {
      _homeSwitchSim = new DIOSim(_homeSwitch);

      _elevatorSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(2),
              WristevatorConstants.elevatorGearRatio,
              Units.lbsToKilograms(9.398),
              WristevatorConstants.drumRadius.in(Meters),
              0,
              WristevatorConstants.maxElevatorHeight.in(Radians)
                  * WristevatorConstants.drumCircumference.in(Meters),
              false,
              0);

      _wristSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              WristevatorConstants.wristGearRatio,
              SingleJointedArmSim.estimateMOI(
                  WristevatorConstants.manipulatorLength.in(Meters), Units.lbsToKilograms(8.155)),
              WristevatorConstants.manipulatorLength.in(Meters),
              WristevatorConstants.minWristAngle.in(Radians),
              WristevatorConstants.maxWristAngle.in(Radians),
              false,
              0);

      startSimThread();
    }
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
                  Units.radiansToRotations(
                      _wristSim.getAngleRads() * WristevatorConstants.wristGearRatio));

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
                  Units.radiansToRotations(
                      _wristSim.getVelocityRadPerSec() * WristevatorConstants.wristGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Wristevator Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Elevator Velocity")
  public double getElevatorVelocity() {
    return _elevatorVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Wrist Velocity")
  public double getWristVelocity() {
    return _wristVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return _angleGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Home Switch")
  public boolean homeSwitch() {
    return _homeSwitch.get();
  }

  private Command idle() {
    return setSpeeds(() -> 0, () -> 0).withName("Idle");
  }

  // distance between current position and supplied setpoint
  private double distance(Setpoint b) {
    var x1 = getHeight();
    var x2 = b.getHeight().in(Radians);

    var y1 = getAngle();
    var y2 = b.getAngle().in(Radians);

    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  /** Whether the wristevator is near a setpoint. */
  private boolean atSetpoint(Setpoint setpoint) {
    return MathUtil.isNear(setpoint.getAngle().in(Radians), getAngle(), 0.01)
        && MathUtil.isNear(setpoint.getHeight().in(Radians), getHeight(), 0.01);
  }

  /** Whether to stop lower / upper motion. */
  private Pair<Boolean, Boolean> shouldStopMotion(double elevatorSpeed, double wristSpeed) {
    var nextHeight = getHeight() + elevatorSpeed * Robot.kDefaultPeriod;
    var nextAngle = getAngle() + wristSpeed * Robot.kDefaultPeriod;

    boolean stopLower = false;
    boolean stopUpper = false;

    if (nextAngle <= WristevatorConstants.lowerAngleLimit.get(nextHeight)) {
      stopLower = true;
    }

    if (nextAngle >= WristevatorConstants.upperAngleLimit.get(nextHeight)) {
      stopUpper = true;
    }

    return Pair.of(stopLower, stopUpper);
  }

  /** Find new constraints for the motion magic control requests. */
  private void findProfileConstraints(Setpoint setpoint) {
    _elevatorMaxProfile.calculate(
        0, new State(getHeight(), 0), new State(setpoint.getAngle().in(Radians), 0));

    _wristMaxProfile.calculate(
        0, new State(getAngle(), 0), new State(setpoint.getAngle().in(Radians), 0));

    double elevatorTime = _elevatorMaxProfile.totalTime();
    double wristTime = _wristMaxProfile.totalTime();

    TrapezoidProfile fasterProfile =
        wristTime > elevatorTime ? _elevatorMaxProfile : _wristMaxProfile;
    TrapezoidProfile slowerProfile =
        fasterProfile.equals(_elevatorMaxProfile) ? _wristMaxProfile : _elevatorMaxProfile;

    double fasterDistance =
        fasterProfile.equals(_elevatorMaxProfile)
            ? setpoint.getHeight().in(Radians) - getHeight()
            : setpoint.getAngle().in(Radians) - getAngle();

    // slower profile cruise velocity and acceleration
    double slowerVel = slowerProfile.equals(_elevatorMaxProfile) ? 0 : 0;
    double slowerAccel = slowerProfile.equals(_elevatorMaxProfile) ? 0 : 0;

    // find the acceleration and cruise times of the slower profile
    double slowerAccelTime = slowerVel / slowerAccel;
    double slowerCruiseTime = slowerProfile.totalTime() - 2 * slowerAccelTime;

    // if the slower profile doesn't reach a cruise velocity (triangular profile)
    // make the cruise time 0 and re-calculate acceleration time and peak velocity
    if (Math.signum(slowerCruiseTime) <= 0) {
      slowerCruiseTime = 0;
      slowerAccelTime = slowerProfile.totalTime() / 2;

      slowerVel = slowerAccelTime * slowerAccel;
    }

    double adjustedVel =
        fasterDistance
            / (slowerAccel / slowerVel * Math.pow(slowerAccelTime, 2) + slowerCruiseTime);
    double adjustedAccel = adjustedVel / slowerAccelTime;

    var elevatorConstraints =
        fasterProfile.equals(_elevatorMaxProfile)
            ? new Constraints(adjustedVel, adjustedAccel)
            : new Constraints(0, 0);
    var wristContraints =
        fasterProfile.equals(_wristMaxProfile)
            ? new Constraints(adjustedVel, adjustedAccel)
            : new Constraints(0, 0);

    // re-assign profile constraints to motors
    _heightSetter.Velocity = elevatorConstraints.maxVelocity;
    _heightSetter.Acceleration = elevatorConstraints.maxAcceleration;

    _angleSetter.Velocity = wristContraints.maxVelocity;
    _angleSetter.Acceleration = wristContraints.maxAcceleration;
  }

  /** Finds the next setpoint variable given the previous setpoint variable and the goal. */
  private void findNextSetpoint(Setpoint goal) {
    // if we just came from manual or are in between verticies, go to an intermediate
    if (_isManual || _prevSetpoint != _nextSetpoint) {
      Intermediate closest = Intermediate.INFINITY;

      // find the closest intermediate vertex
      for (Intermediate intermediate : Intermediate.values()) {
        if (distance(intermediate) < distance(closest)) {
          closest = intermediate;
        }
      }

      _nextSetpoint = closest;

      return;
    }

    if (WristevatorConstants.setpointMap.containsKey(Pair.of(_prevSetpoint, goal))) {
      _nextSetpoint = WristevatorConstants.setpointMap.get(Pair.of(_prevSetpoint, goal));

      return;
    }

    _nextSetpoint = goal;
  }

  /** Whether the wristevator is open for manual control or not. */
  public boolean isManual() {
    return _isManual;
  }

  /** Indicate switch to manual control. */
  public Command switchToManual() {
    return runOnce(() -> _isManual = true).withName("Switch To Manual");
  }

  /** Drives the wristevator to a goal setpoint, going to any intermediate setpoints if needed. */
  public Command setGoal(Setpoint goal) {
    return run(() -> {
          // once the next setpoint is reached, re-find the next one
          if (atSetpoint(_nextSetpoint)) {
            _prevSetpoint = _nextSetpoint;

            findNextSetpoint(goal);
            findProfileConstraints(_nextSetpoint);
          }

          // travel to next setpoint
          _heightSetter.withPosition(_nextSetpoint.getHeight().in(Radians));
          _angleSetter.withPosition(_nextSetpoint.getAngle());
        })
        .beforeStarting(
            idle()
                .until(
                    () ->
                        MathUtil.isNear(0, getElevatorVelocity(), 0.01)
                            && MathUtil.isNear(0, getWristVelocity(), 0.01))
                .andThen(
                    () -> {
                      findNextSetpoint(goal);
                      findProfileConstraints(_nextSetpoint);

                      _isManual = false;
                    }))
        .until(() -> _prevSetpoint == goal) // prev = next = goal
        .withName("Set Goal");
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

    var enableLimits = shouldStopMotion(getElevatorVelocity(), getWristVelocity());

    _elevatorVelocitySetter.LimitReverseMotion = enableLimits.getFirst();
    _wristVelocitySetter.LimitReverseMotion = enableLimits.getFirst();

    _elevatorVelocitySetter.LimitForwardMotion = enableLimits.getSecond();
    _wristVelocitySetter.LimitForwardMotion = enableLimits.getSecond();

    DogLog.log("Wristevator/Previous Setpoint", _prevSetpoint.toString());
    DogLog.log("Wristevator/Next Setpoint", _nextSetpoint.toString());
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
