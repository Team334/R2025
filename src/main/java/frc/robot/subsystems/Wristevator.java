package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.units.measure.Voltage;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.WristevatorConstants;
import frc.robot.Constants.WristevatorConstants.Intermediate;
import frc.robot.Constants.WristevatorConstants.Setpoint;
import frc.robot.Robot;
import frc.robot.utils.SysId;
import java.util.function.Consumer;
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

  private final DynamicMotionMagicVoltage _heightSetter =
      new DynamicMotionMagicVoltage(HOME.getHeight().in(Rotations), 0, 0, 0);
  private final DynamicMotionMagicVoltage _angleSetter =
      new DynamicMotionMagicVoltage(HOME.getAngle().in(Rotations), 0, 0, 0);

  private final VoltageOut _elevatorVoltageSetter = new VoltageOut(0);
  private final VoltageOut _wristVoltageSetter = new VoltageOut(0);

  private final StatusSignal<Angle> _heightGetter = _leftMotor.getPosition();
  private final StatusSignal<Angle> _angleGetter = _wristMotor.getPosition();

  private final StatusSignal<Double> _elevatorReference = _leftMotor.getClosedLoopReference();
  private final StatusSignal<Double> _elevatorReferenceSlope =
      _leftMotor.getClosedLoopReferenceSlope();

  private final StatusSignal<Double> _wristReference = _wristMotor.getClosedLoopReference();
  private final StatusSignal<Double> _wristReferenceSlope =
      _wristMotor.getClosedLoopReferenceSlope();

  private final SysIdRoutine _elevatorRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(2),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setElevatorVoltage(volts.in(Volts)), null, this));

  private final SysIdRoutine _wristRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.3).per(Second),
              Volts.of(0.8),
              Seconds.of(4),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setWristVoltage(volts.in(Volts)), null, this));

  @Logged(name = "Motion Magic Timestamp Threshold")
  private double _motionMagicTimestampThreshold = 0;

  private final Trigger _motionMagicTrigger =
      new Trigger(
              () ->
                  _leftMotor.getMotionMagicIsRunning().getValue()
                      == MotionMagicIsRunningValue.Enabled)
          .and(
              () ->
                  _wristMotor.getMotionMagicIsRunning().getValue()
                      == MotionMagicIsRunningValue.Enabled)
          .onTrue(
              Commands.runOnce(
                  () -> _motionMagicTimestampThreshold = Utils.getCurrentTimeSeconds()));

  private final StatusSignal<AngularVelocity> _elevatorVelocityGetter = _leftMotor.getVelocity();
  private final StatusSignal<AngularVelocity> _wristVelocityGetter = _wristMotor.getVelocity();

  private final VelocityVoltage _elevatorVelocitySetter = new VelocityVoltage(0).withSlot(1);
  private final VelocityVoltage _wristVelocitySetter = new VelocityVoltage(0).withSlot(1);

  // elevator profile
  private final Constraints _elevatorMaxConstraints =
      new Constraints(
          WristevatorConstants.maxElevatorSpeed.in(RadiansPerSecond),
          WristevatorConstants.maxElevatorAcceleration.in(RadiansPerSecondPerSecond));

  private final State _elevatorMaxGoal = new State();
  private final State _elevatorMaxSetpoint = new State();

  private final TrapezoidProfile _elevatorMaxProfile =
      new TrapezoidProfile(_elevatorMaxConstraints);

  // wrist profile
  private final Constraints _wristMaxConstraints =
      new Constraints(
          WristevatorConstants.maxWristSpeed.in(RadiansPerSecond),
          WristevatorConstants.maxWristAcceleration.in(RadiansPerSecondPerSecond));

  private final State _wristMaxGoal = new State();
  private final State _wristMaxSetpoint = new State();

  private final TrapezoidProfile _wristMaxProfile = new TrapezoidProfile(_wristMaxConstraints);

  private final DigitalInput _homeSwitch = new DigitalInput(WristevatorConstants.homeSwitch);

  @Logged(name = "Is Manual")
  private boolean _isManual = false;

  @Logged(name = "Is Motion Magic")
  private boolean _isMotionMagic = false;

  @Logged(name = "Finished Latest Profiles")
  private boolean _finishedLatestProfiles = true;

  private Setpoint _latestSetpoint = HOME;
  private Consumer<Setpoint> _wristevatorGoalSetter;

  private DIOSim _homeSwitchSim;

  private ElevatorSim _elevatorSim;
  private SingleJointedArmSim _wristSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Wristevator(Consumer<Setpoint> wristevatorGoalSetter) {
    _wristevatorGoalSetter = wristevatorGoalSetter;

    new Trigger(() -> _isManual)
        .onTrue(Commands.runOnce(() -> _wristevatorGoalSetter.accept(null)));

    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();
    var wristMotorConfigs = new TalonFXConfiguration();

    // left motor configs
    var leftMotorSlot = new SlotConfigs();

    leftMotorSlot.kS = WristevatorConstants.elevatorkS.in(Volts);
    leftMotorSlot.kG = WristevatorConstants.elevatorkG.in(Volts);
    leftMotorSlot.kV = WristevatorConstants.elevatorkV.in(Volts.per(RotationsPerSecond));
    leftMotorSlot.kA = WristevatorConstants.elevatorkA.in(Volts.per(RotationsPerSecondPerSecond));

    leftMotorSlot.kP = WristevatorConstants.elevatorkP.in(Volts.per(Rotations));

    leftMotorSlot.GravityType = GravityTypeValue.Elevator_Static;

    leftMotorConfigs.Slot0 = Slot0Configs.from(leftMotorSlot);
    leftMotorConfigs.Slot1 = Slot1Configs.from(leftMotorSlot).withKP(0);

    leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.elevatorGearRatio;

    leftMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        WristevatorConstants.maxElevatorHeight.in(Rotations);
    leftMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        WristevatorConstants.minElevatorHeight.in(Rotations);

    leftMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // wrist motor configs
    var wristMotorSlot = new SlotConfigs();

    wristMotorSlot.kS = WristevatorConstants.wristkS.in(Volts);
    wristMotorSlot.kG = WristevatorConstants.wristkG.in(Volts);
    wristMotorSlot.kV = WristevatorConstants.wristkV.in(Volts.per(RotationsPerSecond));
    wristMotorSlot.kA = WristevatorConstants.wristkA.in(Volts.per(RotationsPerSecondPerSecond));

    wristMotorSlot.kP = WristevatorConstants.wristkP.in(Volts.per(Rotations));

    wristMotorSlot.GravityType = GravityTypeValue.Arm_Cosine;

    wristMotorConfigs.Slot0 = Slot0Configs.from(wristMotorSlot);
    wristMotorConfigs.Slot1 = Slot1Configs.from(wristMotorSlot).withKP(0);

    wristMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.wristGearRatio;

    wristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        WristevatorConstants.maxWristAngle.in(Rotations);
    wristMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        WristevatorConstants.minWristAngle.in(Rotations);

    wristMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);
    CTREUtil.attempt(() -> _wristMotor.getConfigurator().apply(wristMotorConfigs), _wristMotor);

    CTREUtil.attempt(() -> _leftMotor.optimizeBusUtilization(), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.optimizeBusUtilization(), _rightMotor);
    CTREUtil.attempt(() -> _wristMotor.optimizeBusUtilization(), _wristMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _leftMotor.getPosition(),
                _leftMotor.getVelocity(),
                _leftMotor.getClosedLoopReference(),
                _leftMotor.getClosedLoopReferenceSlope(),
                _leftMotor.getMotionMagicIsRunning(),
                _leftMotor.getMotorVoltage()),
        _leftMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _wristMotor.getPosition(),
                _wristMotor.getVelocity(),
                _wristMotor.getClosedLoopReference(),
                _wristMotor.getClosedLoopReferenceSlope(),
                _wristMotor.getMotionMagicIsRunning(),
                _wristMotor.getMotorVoltage()),
        _wristMotor);

    CTREUtil.attempt(() -> _wristMotor.setPosition(Radians.of(-1.05)), _wristMotor);
    CTREUtil.attempt(() -> _leftMotor.setPosition(Radians.of(0)), _leftMotor);

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);
    FaultLogger.register(_wristMotor);

    _rightMotor.setControl(new Follower(WristevatorConstants.leftMotorId, true));

    setDefaultCommand(holdInPlace());

    SysId.displayRoutine(
        "Elevator", _elevatorRoutine, () -> getHeight() >= 35, () -> getHeight() <= 0.5);
    SysId.displayRoutine("Wrist", _wristRoutine, () -> getAngle() >= 0.5, () -> getAngle() <= -0.2);

    if (Robot.isSimulation()) {
      _homeSwitchSim = new DIOSim(_homeSwitch);

      _elevatorSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(2),
              WristevatorConstants.elevatorGearRatio,
              Units.lbsToKilograms(9.398),
              WristevatorConstants.drumRadius.in(Meters),
              0,
              WristevatorConstants.maxElevatorHeight.in(Rotations)
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

  @Logged(name = "Elevator Height")
  public double getHeight() {
    return _heightGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Wrist Angle")
  public double getAngle() {
    return _angleGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Home Switch")
  public boolean homeSwitch() {
    return !_homeSwitch.get();
  }

  /** Whether the wristevator is open for manual control or not. */
  public boolean isManual() {
    return _isManual;
  }

  /** Indicate switch to manual control. */
  public Command switchToManual() {
    return Commands.runOnce(
            () -> {
              _isManual = true;
              _finishedLatestProfiles = false;
            })
        .withName("Switch To Manual");
  }

  private Command holdInPlace() {
    return run(() -> {
          _leftMotor.setControl(_heightSetter);
          _wristMotor.setControl(_angleSetter);
        })
        .withName("Hold In Place");
  }

  // distance between current position and supplied setpoint
  private double distance(Setpoint b) {
    var x1 = getHeight();
    var x2 = b.getHeight().in(Radians);

    var y1 = getAngle();
    var y2 = b.getAngle().in(Radians);

    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  // refresh the references of the talonfx profiles
  private void refreshProfileReferences() {
    BaseStatusSignal.refreshAll(
        _elevatorReference, _elevatorReferenceSlope, _wristReference, _wristReferenceSlope);

    _isMotionMagic = false;

    if (!_motionMagicTrigger.getAsBoolean()) return;

    if (_elevatorReference.getTimestamp().getTime() > _motionMagicTimestampThreshold
        && _elevatorReferenceSlope.getTimestamp().getTime() > _motionMagicTimestampThreshold
        && _wristReference.getTimestamp().getTime() > _motionMagicTimestampThreshold
        && _wristReferenceSlope.getTimestamp().getTime() > _motionMagicTimestampThreshold) {
      _isMotionMagic = true;
    }
  }

  /** Finds the next setpoint variable given the previous setpoint variable and the goal. */
  private void findNextSetpoint(Setpoint goal) {
    // if we haven't finished the previous profiles
    if (!_finishedLatestProfiles) {
      Intermediate closest = Intermediate.INFINITY;

      // find the closest intermediate vertex
      for (Intermediate intermediate : Intermediate.values()) {
        if (distance(intermediate) < distance(closest)) {
          closest = intermediate;
        }
      }

      _latestSetpoint = closest;

      return;
    }

    if (WristevatorConstants.setpointMap.containsKey(Pair.of(_latestSetpoint, goal))) {
      _latestSetpoint = WristevatorConstants.setpointMap.get(Pair.of(_latestSetpoint, goal));

      return;
    }

    _latestSetpoint = goal;
  }

  /** Find new constraints for the motion magic control requests. */
  private void findProfileConstraints(Setpoint setpoint) {
    _elevatorMaxSetpoint.position = getHeight();
    _elevatorMaxSetpoint.velocity = 0;

    _wristMaxSetpoint.position = getAngle();
    _wristMaxSetpoint.velocity = 0;

    _elevatorMaxGoal.position = setpoint.getHeight().in(Radians);
    _elevatorMaxGoal.velocity = 0;

    _wristMaxGoal.position = setpoint.getAngle().in(Radians);
    _wristMaxGoal.velocity = 0;

    _elevatorMaxProfile.calculate(0, _elevatorMaxSetpoint, _elevatorMaxGoal);
    _wristMaxProfile.calculate(0, _wristMaxSetpoint, _wristMaxGoal);

    double elevatorTime = _elevatorMaxProfile.totalTime();
    double wristTime = _wristMaxProfile.totalTime();

    TrapezoidProfile fasterProfile =
        wristTime > elevatorTime ? _elevatorMaxProfile : _wristMaxProfile;
    TrapezoidProfile slowerProfile =
        fasterProfile.equals(_elevatorMaxProfile) ? _wristMaxProfile : _elevatorMaxProfile;

    double fasterDistance = 0;
    double slowerVel = 0;
    double slowerAccel = 0;

    if (fasterProfile == _elevatorMaxProfile) {
      fasterDistance = setpoint.getHeight().in(Radians) - getHeight();
      slowerVel = _wristMaxConstraints.maxVelocity;
      slowerAccel = _wristMaxConstraints.maxAcceleration;
    } else {
      fasterDistance = setpoint.getAngle().in(Radians) - getAngle();
      slowerVel = _elevatorMaxConstraints.maxVelocity;
      slowerAccel = _elevatorMaxConstraints.maxAcceleration;
    }

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
            : _elevatorMaxConstraints;

    var wristConstraints =
        fasterProfile.equals(_wristMaxProfile)
            ? new Constraints(adjustedVel, adjustedAccel)
            : _wristMaxConstraints;

    // re-assign profile constraints to motors
    _heightSetter.Velocity = Units.radiansToRotations(elevatorConstraints.maxVelocity);
    _heightSetter.Acceleration = Units.radiansToRotations(elevatorConstraints.maxAcceleration);

    _angleSetter.Velocity = Units.radiansToRotations(wristConstraints.maxVelocity);
    _angleSetter.Acceleration = Units.radiansToRotations(wristConstraints.maxAcceleration);
  }

  /** Drives the wristevator to a goal setpoint, going to any intermediate setpoints if needed. */
  public Command setGoal(Setpoint goal) {
    return Commands.runOnce(() -> _wristevatorGoalSetter.accept(goal))
        .andThen(
            run(() -> {
                  // move towards the next setpoint
                  _leftMotor.setControl(_heightSetter.withPosition(_latestSetpoint.getHeight()));
                  _wristMotor.setControl(_angleSetter.withPosition(_latestSetpoint.getAngle()));

                  if (!_isMotionMagic) return;

                  // once the next setpoint is reached, re-find the next one
                  if (_finishedLatestProfiles) {
                    findNextSetpoint(goal);
                    findProfileConstraints(_latestSetpoint);
                  }

                  _finishedLatestProfiles =
                      (MathUtil.isNear(
                              _latestSetpoint.getHeight().in(Rotations),
                              _elevatorReference.getValueAsDouble(),
                              0.001)
                          && MathUtil.isNear(0, _elevatorReferenceSlope.getValueAsDouble(), 0.001)
                          && MathUtil.isNear(
                              _latestSetpoint.getAngle().in(Rotations),
                              _wristReference.getValueAsDouble(),
                              0.001)
                          && MathUtil.isNear(0, _wristReferenceSlope.getValueAsDouble(), 0.001));
                })
                .beforeStarting(
                    setSpeeds(() -> 0, () -> 0)
                        .until(
                            () ->
                                MathUtil.isNear(0, getElevatorVelocity(), 0.01)
                                    && MathUtil.isNear(0, getWristVelocity(), 0.01))
                        .andThen(
                            () -> {
                              _isManual = false;

                              findNextSetpoint(goal);
                              findProfileConstraints(_latestSetpoint);

                              _finishedLatestProfiles = false;
                            }))
                .until(() -> _finishedLatestProfiles && _latestSetpoint == goal))
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
          _leftMotor.setControl(
              _elevatorVelocitySetter.withVelocity(
                  Units.radiansToRotations(elevatorSpeed.getAsDouble())));
          _wristMotor.setControl(
              _wristVelocitySetter.withVelocity(
                  Units.radiansToRotations(wristSpeed.getAsDouble())));
        })
        .withName("Set Speeds");
  }

  private void setElevatorVoltage(double volts) {
    _leftMotor.setControl(_elevatorVoltageSetter.withOutput(volts));
  }

  private void setWristVoltage(double volts) {
    _wristMotor.setControl(_wristVoltageSetter.withOutput(volts));
  }

  @Override
  public void periodic() {
    super.periodic();

    refreshProfileReferences();

    // hard limits
    _heightSetter.LimitReverseMotion = homeSwitch();
    _elevatorVelocitySetter.LimitReverseMotion = homeSwitch();

    DogLog.log(
        "Wristevator/Elevator Reference",
        Units.rotationsToRadians(_elevatorReference.getValueAsDouble()));
    DogLog.log(
        "Wristevator/Wrist Reference",
        Units.rotationsToRadians(_wristReference.getValueAsDouble()));
    DogLog.log(
        "Wristevator/Elevator Reference Slope",
        Units.rotationsToRadians(_elevatorReferenceSlope.getValueAsDouble()));
    DogLog.log(
        "Wristevator/Wrist Reference Slope",
        Units.rotationsToRadians(_wristReferenceSlope.getValueAsDouble()));

    DogLog.log("Wristevator/Latest Setpoint", _latestSetpoint.toString());
  }

  @Override
  public void simulationPeriodic() {
    _homeSwitchSim.setValue(getHeight() == 0);

    _elevator.setLength(
        Units.radiansToRotations(getHeight()) * WristevatorConstants.drumCircumference.in(Meters));
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
