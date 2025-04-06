package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.utils.SysId;

public class Intake extends AdvancedSubsystem {
  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("pivot", 0.5, 0.1);

  private final MechanismLigament2d _intake =
      _root.append(new MechanismLigament2d("intake", 0.5, 0, 3, new Color8Bit(Color.kBlue)));

  private final TalonFX _actuatorMotor =
      new TalonFX(IntakeConstants.actuatorMotorId, Constants.canivore);
  private final TalonFX _feedMotor = new TalonFX(IntakeConstants.feedMotorId, Constants.canivore);

  private final MotionMagicVoltage _actuatorPositionSetter = new MotionMagicVoltage(0);
  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);

  private final VoltageOut _actuatorVoltageSetter = new VoltageOut(0);
  private final VoltageOut _feedVoltageSetter = new VoltageOut(0);

  private final StatusSignal<Angle> _actuatorPositionGetter = _actuatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private final StatusSignal<Current> _feedCurrentGetter = _feedMotor.getStatorCurrent();

  private final SysIdRoutine _actuatorRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5),
              Volts.of(3),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setActuatorVoltage(volts.in(Volts)), null, this));

  private final SysIdRoutine _feedRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> setFeedVoltage(volts.in(Volts)), null, this));

  private final Trigger _retrievedAlgae =
      new Trigger(
              () ->
                  _feedCurrentGetter.getValue().in(Amps)
                      > IntakeConstants.algaeCurrentThreshold.in(Amps))
          .debounce(0.1);

  private boolean _hasAlgae = false;

  private SingleJointedArmSim _actuatorSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public Intake() {
    setDefaultCommand(stow());

    _retrievedAlgae.and(intakeAlgae()::isScheduled).onTrue(runOnce(() -> _hasAlgae = true));
    new Trigger(holdAlgae()::isScheduled).onFalse(runOnce(() -> _hasAlgae = false));
    new Trigger(() -> _hasAlgae).whileTrue(holdAlgae());

    var feedMotorConfigs = new TalonFXConfiguration();
    var actuatorMotorConfigs = new TalonFXConfiguration();

    // feed configs
    feedMotorConfigs.Slot0.kS = IntakeConstants.feedkS.in(Volts);
    feedMotorConfigs.Slot0.kV = IntakeConstants.feedkV.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Slot0.kP = IntakeConstants.feedkP.in(Volts.per(RotationsPerSecond));

    feedMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.feedGearRatio;

    // actuator configs
    actuatorMotorConfigs.Slot0.kS = IntakeConstants.actuatorkS.in(Volts);
    actuatorMotorConfigs.Slot0.kG = IntakeConstants.actuatorkG.in(Volts);
    actuatorMotorConfigs.Slot0.kV = IntakeConstants.actuatorkV.in(Volts.per(RotationsPerSecond));
    actuatorMotorConfigs.Slot0.kA =
        IntakeConstants.actuatorkA.in(Volts.per(RotationsPerSecondPerSecond));

    actuatorMotorConfigs.Slot0.kP = IntakeConstants.actuatorkP.in(Volts.per(Rotations));

    actuatorMotorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    actuatorMotorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.actuatorGearRatio;

    actuatorMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.actuatorStowed.plus(Radians.of(0.15)).in(Rotations);
    actuatorMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.actuatorOut.minus(Radians.of(0.15)).in(Rotations);

    actuatorMotorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    actuatorMotorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    actuatorMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.actuatorVelocity.in(RotationsPerSecond);
    actuatorMotorConfigs.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.actuatorAcceleration.in(RotationsPerSecondPerSecond);

    actuatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);

    CTREUtil.attempt(
        () -> _actuatorMotor.getConfigurator().apply(actuatorMotorConfigs), _actuatorMotor);
    CTREUtil.attempt(
        () -> _actuatorMotor.setPosition(IntakeConstants.actuatorStowed), _actuatorMotor);

    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);
    CTREUtil.attempt(() -> _actuatorMotor.optimizeBusUtilization(), _actuatorMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _feedMotor.getPosition(),
                _feedMotor.getVelocity(),
                _feedMotor.getMotorVoltage()),
        _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _actuatorMotor.getPosition(),
                _actuatorMotor.getVelocity(),
                _actuatorMotor.getMotorVoltage()),
        _actuatorMotor);

    FaultLogger.register(_feedMotor);
    FaultLogger.register(_actuatorMotor);

    SysId.displayRoutine(
        "Actuator", _actuatorRoutine, () -> getAngle() >= 1.8, () -> getAngle() <= -0.4);
    SysId.displayRoutine("Intake Feed", _feedRoutine);

    if (Robot.isSimulation()) {
      _actuatorSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              IntakeConstants.actuatorGearRatio,
              SingleJointedArmSim.estimateMOI(
                  IntakeConstants.intakeLength.in(Meters), Units.lbsToKilograms(12)),
              IntakeConstants.intakeLength.in(Meters),
              IntakeConstants.actuatorStowed.in(Radians),
              IntakeConstants.actuatorOut.in(Radians),
              false,
              IntakeConstants.actuatorStowed.in(Radians));

      startSimThread();
    }
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              final double batteryVoltage = RobotController.getBatteryVoltage();

              var actuatorMotorSimState = _actuatorMotor.getSimState();

              actuatorMotorSimState.setSupplyVoltage(batteryVoltage);

              _actuatorSim.setInputVoltage(
                  actuatorMotorSimState.getMotorVoltageMeasure().in(Volts));

              _actuatorSim.update(deltaTime);

              actuatorMotorSimState.setRawRotorPosition(
                  Units.radiansToRotations(
                      _actuatorSim.getAngleRads() * IntakeConstants.actuatorGearRatio));

              actuatorMotorSimState.setRotorVelocity(
                  Units.radiansToRotations(
                      _actuatorSim.getVelocityRadPerSec() * IntakeConstants.actuatorGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Intake Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return _actuatorPositionGetter.refresh().getValue().in(Radians);
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  @Logged(name = "Has Algae")
  public boolean hasAlgae() {
    return _hasAlgae;
  }

  // set the actuator angle and feed speed.
  private Command set(double actuatorAngle, double feedSpeed) {
    return run(
        () -> {
          _actuatorMotor.setControl(
              _actuatorPositionSetter.withPosition(Units.radiansToRotations(actuatorAngle)));
          _feedMotor.setControl(
              _feedVelocitySetter.withVelocity(Units.radiansToRotations(feedSpeed)));
        });
  }

  /** Stow the intake into the robot. */
  public Command stow() {
    return set(IntakeConstants.actuatorStowed.in(Radians), 0).withName("Stow");
  }

  /** Intake a coral off of the ground. */
  public Command intake() {
    return set(
            IntakeConstants.actuatorOut.in(Radians), IntakeConstants.feedSpeed.in(RadiansPerSecond))
        .withName("Intake");
  }

  /** Outtake onto the ground. */
  public Command outtake() {
    return set(
            IntakeConstants.actuatorOut.in(Radians),
            -IntakeConstants.feedSpeed.in(RadiansPerSecond))
        .withName("Outtake");
  }

  /** Holds an algae in the intake. */
  public Command holdAlgae() {
    return run(
        () -> {
          _actuatorMotor.setControl(
              _actuatorPositionSetter.withPosition(
                  Units.radiansToRotations(IntakeConstants.intakeAlgae.in(Radians))));
          _feedMotor.setControl(
              _feedVoltageSetter.withOutput(IntakeConstants.algaeStallVolts.in(Volts)));
        });
  }

  /** Intakes algae of the ground. */
  public Command intakeAlgae() {
    return set(
        IntakeConstants.intakeAlgae.in(Radians),
        -IntakeConstants.algaeFeedSpeed.in(RadiansPerSecond));
  }

  /** Outtakes algae into the processor. */
  public Command outtakeAlgae() {
    return Commands.sequence(
        set(
                IntakeConstants.scoreAlgae.in(Radians),
                -IntakeConstants.algaeFeedSpeed.in(RadiansPerSecond))
            .until(() -> MathUtil.isNear(IntakeConstants.scoreAlgae.in(Radians), getAngle(), 0.1)),
        set(
            IntakeConstants.scoreAlgae.in(Radians),
            IntakeConstants.algaeFeedSpeed.in(RadiansPerSecond)));
  }

  private void setActuatorVoltage(double volts) {
    _actuatorMotor.setControl(_actuatorVoltageSetter.withOutput(volts));
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

    _intake.setAngle(Math.toDegrees(getAngle()));

    SmartDashboard.putData("Intake Visualizer", _mech);
  }

  @Override
  public void close() {
    _actuatorMotor.close();
    _feedMotor.close();

    _simNotifier.close();
  }
}
