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
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.Timer;
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
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.WristevatorConstants;
import frc.robot.Constants.WristevatorConstants.Intermediate;
import frc.robot.Constants.WristevatorConstants.Preset;
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

  private final Constraints _elevatorMaxConstraints =
      new Constraints(
          WristevatorConstants.maxElevatorSpeed.in(RadiansPerSecond),
          WristevatorConstants.maxElevatorAcceleration.in(RadiansPerSecondPerSecond));

  private final State _elevatorMaxGoal = new State();
  private State _elevatorMaxSetpoint = new State();

  private final Constraints _wristMaxConstraints =
      new Constraints(
          WristevatorConstants.maxWristSpeed.in(RadiansPerSecond),
          WristevatorConstants.maxWristAcceleration.in(RadiansPerSecondPerSecond));

  private final State _wristMaxGoal = new State();
  private State _wristMaxSetpoint = new State();

  private final TrapezoidProfile _elevatorMaxProfile =
      new TrapezoidProfile(_elevatorMaxConstraints);

  private final TrapezoidProfile _wristMaxProfile = new TrapezoidProfile(_wristMaxConstraints);

  private final DigitalInput _homeSwitch = new DigitalInput(WristevatorConstants.homeSwitch);

  private Setpoint _nextSetpoint = HOME;

  @Logged(name = "Is Manual")
  private boolean _isManual = false;

  private final Timer _profileTimer = new Timer();
  private double _profileTime = 0;

  private DIOSim _homeSwitchSim;

  private ElevatorSim _elevatorSim;
  private SingleJointedArmSim _wristSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Wristevator() {
    var leftMotorConfigs = new TalonFXConfiguration();
    var rightMotorConfigs = new TalonFXConfiguration();
    var wristMotorConfigs = new TalonFXConfiguration();

    leftMotorConfigs.Slot0.kV = WristevatorConstants.elevatorkV.in(Volts.per(RotationsPerSecond));
    leftMotorConfigs.Slot0.kA =
        WristevatorConstants.elevatorkA.in(Volts.per(RotationsPerSecondPerSecond));

    leftMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.elevatorGearRatio;

    wristMotorConfigs.Slot0.kV = WristevatorConstants.wristkV.in(Volts.per(RotationsPerSecond));
    wristMotorConfigs.Slot0.kA =
        WristevatorConstants.wristkA.in(Volts.per(RotationsPerSecondPerSecond));

    wristMotorConfigs.Feedback.SensorToMechanismRatio = WristevatorConstants.wristGearRatio;

    CTREUtil.attempt(() -> _leftMotor.getConfigurator().apply(leftMotorConfigs), _leftMotor);
    CTREUtil.attempt(() -> _rightMotor.getConfigurator().apply(rightMotorConfigs), _rightMotor);
    CTREUtil.attempt(() -> _wristMotor.getConfigurator().apply(wristMotorConfigs), _wristMotor);

    FaultLogger.register(_leftMotor);
    FaultLogger.register(_rightMotor);
    FaultLogger.register(_wristMotor);

    _rightMotor.setControl(new Follower(WristevatorConstants.leftMotorId, true));

    setDefaultCommand(idle());

    Translation2d[] presets = new Translation2d[Preset.values().length];
    Translation2d[] intermediates = new Translation2d[Intermediate.values().length];

    for (int i = 0; i < Preset.values().length; i++) {
      presets[i] =
          new Translation2d(
              Preset.values()[i].getHeight().in(Radians),
              Preset.values()[i].getAngle().in(Radians)
                  + WristevatorConstants.minWristAngle.abs(Radians));
    }

    for (int i = 0; i < Intermediate.values().length; i++) {
      intermediates[i] =
          new Translation2d(
              Intermediate.values()[i].getHeight().in(Radians),
              Intermediate.values()[i].getAngle().in(Radians)
                  + WristevatorConstants.minWristAngle.abs(Radians));
    }

    DogLog.log("Wristevator/Presets", presets);
    DogLog.log("Wristevator/Intermediates", intermediates);

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
    System.out.println(_profileTimer.get());
    return _profileTimer.hasElapsed(_profileTime) && _nextSetpoint == setpoint;
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
    }
    else {
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

    var wristContraints =
        fasterProfile.equals(_wristMaxProfile)
            ? new Constraints(adjustedVel, adjustedAccel)
            : _wristMaxConstraints;

    // re-assign profile constraints to motors
    _heightSetter.Velocity = Units.radiansToRotations(elevatorConstraints.maxVelocity);
    _heightSetter.Acceleration = Units.radiansToRotations(elevatorConstraints.maxAcceleration);

    _angleSetter.Velocity = Units.radiansToRotations(wristContraints.maxVelocity);
    _angleSetter.Acceleration = Units.radiansToRotations(wristContraints.maxAcceleration);

    _profileTime = slowerProfile.totalTime();
    _profileTimer.restart();
  }

  /** Finds the next setpoint variable given the previous setpoint variable and the goal. */
  private void findNextSetpoint(Setpoint goal) {
    // if we just came from manual or are in between verticies, go to an intermediate
    if (_isManual || !atSetpoint(_nextSetpoint)) {
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

    if (WristevatorConstants.setpointMap.containsKey(Pair.of(_nextSetpoint, goal))) {
      _nextSetpoint = WristevatorConstants.setpointMap.get(Pair.of(_nextSetpoint, goal));

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
    return Commands.runOnce(() -> _isManual = true).withName("Switch To Manual");
  }

  /** Drives the wristevator to a goal setpoint, going to any intermediate setpoints if needed. */
  public Command setGoal(Setpoint goal) {
    return run(() -> {
          // once the next setpoint is reached, re-find the next one
          if (atSetpoint(_nextSetpoint)) {
            findNextSetpoint(goal);
            findProfileConstraints(_nextSetpoint);
          }

          // log what's generated by wpilib
          DogLog.log(
              "Wristevator/Non-Adjusted Desired Elevator Speed", _elevatorMaxSetpoint.velocity);
          DogLog.log("Wristevator/Non-Adjusted Desired Wrist Speed", _wristMaxSetpoint.velocity);

          _elevatorMaxSetpoint =
              _elevatorMaxProfile.calculate(
                  Robot.kDefaultPeriod, _elevatorMaxSetpoint, _elevatorMaxGoal);
          _wristMaxSetpoint =
              _wristMaxProfile.calculate(Robot.kDefaultPeriod, _wristMaxSetpoint, _wristMaxGoal);

          // travel to next setpoint
          _leftMotor.setControl(_heightSetter.withPosition(_nextSetpoint.getHeight()));
          _wristMotor.setControl(_angleSetter.withPosition(_nextSetpoint.getAngle()));
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
        .until(() -> atSetpoint(goal))
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

  @Override
  public void periodic() {
    super.periodic();

    DogLog.log(
        "Wristevator/Elevator Reference Slope",
        Units.rotationsToRadians(_leftMotor.getClosedLoopReferenceSlope().getValueAsDouble()));
    DogLog.log(
        "Wristevator/Wrist Reference Slope",
        Units.rotationsToRadians(_wristMotor.getClosedLoopReferenceSlope().getValueAsDouble()));
    DogLog.log(
        "Wristevator/Elevator Reference",
        Units.rotationsToRadians(_leftMotor.getClosedLoopReference().getValueAsDouble()));
    DogLog.log(
        "Wristevator/Wrist Reference",
        Units.rotationsToRadians(_wristMotor.getClosedLoopReference().getValueAsDouble()));

    DogLog.log("Wristevator/Next Setpoint", _nextSetpoint.toString());

    DogLog.log(
        "Wristevator/Position",
        new Translation2d(
            getHeight(), getAngle() + WristevatorConstants.minWristAngle.abs(Radians)));
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
