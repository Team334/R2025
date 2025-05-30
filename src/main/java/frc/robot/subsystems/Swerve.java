// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;
import static frc.robot.Robot.getWristevatorGoal;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import frc.lib.InputStream;
import frc.lib.SelfChecked;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldLocation;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.AlignPoses;
import frc.robot.utils.AlignPoses.AlignSide;
import frc.robot.utils.HolonomicController;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawDetection;
import frc.robot.utils.SysId;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

@Logged(strategy = Strategy.OPT_IN)
public class Swerve extends TunerSwerveDrivetrain implements Subsystem, SelfChecked {
  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable =
      new FaultsTable(
          NetworkTableInstance.getDefault().getTable("Self Check"),
          getName() + " Faults"); // TODO: watch out unit tests

  private boolean _hasError = false;

  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  private final SwerveDriveBrake _brakeRequest = new SwerveDriveBrake();

  // auton request for choreo
  private final ApplyFieldSpeeds _fieldSpeedsRequest = new ApplyFieldSpeeds();

  // sysid requests
  private final SysIdSwerveTranslation _translationSysIdRequest = new SysIdSwerveTranslation();
  private final SysIdSwerveSteerGains _steerSysIdRequest = new SysIdSwerveSteerGains();
  private final SysIdSwerveRotation _rotationSysIdRequest = new SysIdSwerveRotation();

  // SysId routine for characterizing translation. This is used to find PID gains for the drive
  // motors.
  private final SysIdRoutine _sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              Seconds.of(5), // Use default timeout (10 s)
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_translationSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
  private final SysIdRoutine _sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_steerSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for finding MOI of the robot.
  private final SysIdRoutine _sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Radian Per Second
              Volts.of(7), // Radians
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_rotationSysIdRequest.withRotationalRate(volts.in(Volts))),
              null,
              this));

  private double _lastSimTime = 0;
  private Notifier _simNotifier;

  @Logged(name = "Driver Chassis Speeds")
  private final ChassisSpeeds _driverChassisSpeeds = new ChassisSpeeds();

  @Logged(name = "Is Field Oriented")
  private boolean _isFieldOriented = true;

  @Logged(name = "Is Open Loop")
  private boolean _isOpenLoop = true;

  @Logged(name = "Ignore Vision Estimates")
  private boolean _ignoreVisionEstimates = false;

  private List<VisionPoseEstimate> _estimates = new ArrayList<VisionPoseEstimate>();

  private double _alignEstimateDistance = Double.MAX_VALUE;

  private Translation2d _alignOdomCompensation = null;

  private HolonomicController _poseController = new HolonomicController();

  private boolean _hasAppliedDriverPerspective;

  @Logged(name = VisionConstants.lowerLeftArducamName)
  private final VisionPoseEstimator _lowerLeftArducam =
      VisionPoseEstimator.buildFromConstants(
          VisionConstants.lowerLeftArducam, this::getHeadingAtTime);

  @Logged(name = VisionConstants.lowerRightArducamName)
  private final VisionPoseEstimator _lowerRightArducam =
      VisionPoseEstimator.buildFromConstants(
          VisionConstants.lowerRightArducam, this::getHeadingAtTime);

  private final List<VisionPoseEstimator> _cameras = List.of(_lowerLeftArducam, _lowerRightArducam);

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<>();
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<>();

  private final List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  private final Set<Pose3d> _detectedTags = new HashSet<>();

  private final VisionSystemSim _visionSystemSim;

  /**
   * Creates a new CommandSwerveDrivetrain.
   *
   * @param drivetrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN
   *     Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical
   *     across all modules (PID constants, wheel radius, etc), and constants unique to each module
   *     (location, device ids, etc).
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    super(drivetrainConstants, SwerveConstants.odometryFrequency.in(Hertz), moduleConstants);

    _robotCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _fieldCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    // closed loop vel always in auto
    _fieldSpeedsRequest.withDriveRequestType(DriveRequestType.Velocity);

    registerTelemetry(
        state -> {
          DogLog.log("Swerve/Pose", state.Pose);
          DogLog.log("Swerve/Raw Heading", state.RawHeading);
          DogLog.log("Swerve/Speeds", state.Speeds);
          DogLog.log("Swerve/Desired Speeds", getKinematics().toChassisSpeeds(state.ModuleTargets));
          DogLog.log("Swerve/Module States", state.ModuleStates);
          DogLog.log("Swerve/Desired Module States", state.ModuleTargets);

          double totalDaqs = state.SuccessfulDaqs + state.FailedDaqs;
          totalDaqs = totalDaqs == 0 ? 1 : totalDaqs;

          DogLog.log("Swerve/Odometry Success %", state.SuccessfulDaqs / totalDaqs * 100);
          DogLog.log("Swerve/Odometry Period", state.OdometryPeriod);
        });

    autonomous().onTrue(Commands.runOnce(() -> _ignoreVisionEstimates = false));

    teleop().onTrue(Commands.runOnce(() -> _ignoreVisionEstimates = false));

    SmartDashboard.putData(
        "RESET PRACTICE FIELD", Commands.runOnce(() -> resetRotation(Rotation2d.fromDegrees(0))));

    // display all sysid routines
    SysId.displayRoutine("Swerve Translation", _sysIdRoutineTranslation);
    SysId.displayRoutine("Swerve Steer", _sysIdRoutineSteer);
    SysId.displayRoutine("Swerve Rotation", _sysIdRoutineRotation);

    registerFallibles();

    if (Robot.isSimulation()) {
      startSimThread();

      _visionSystemSim = new VisionSystemSim("Vision System Sim");
      _visionSystemSim.addAprilTags(FieldConstants.tagLayout);

      _lowerLeftArducam
          .getCameraSim()
          .prop
          .setCalibration(800, 600, Rotation2d.fromDegrees(72.7315316587));

      _lowerRightArducam
          .getCameraSim()
          .prop
          .setCalibration(800, 600, Rotation2d.fromDegrees(72.7315316587));

      _cameras.forEach(cam -> _visionSystemSim.addCamera(cam.getCameraSim(), cam.robotToCam));
    } else {
      _visionSystemSim = null;
    }
  }

  // COPIED FROM ADVANCED SUBSYSTEM

  /**
   * Returns the name of the command that's currently requiring this subsystem. Is "None" when the
   * command in null.
   */
  @Logged(name = "Current Command")
  public final String currentCommandName() {
    if (getCurrentCommand() != null) {
      return getCurrentCommand().getName();
    }

    return "None";
  }

  /** Adds a new fault under this subsystem. */
  private final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    DogLog.logFault(fault.toString());

    _faults.add(fault);
    _faultsTable.set(_faults);
  }

  /** Clears this subsystem's faults. */
  public final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Returns the faults belonging to this subsystem. */
  public final Set<Fault> getFaults() {
    return _faults;
  }

  /** Returns whether this subsystem contains the following fault. */
  public final boolean hasFault(String description, FaultType faultType) {
    return _faults.contains(new Fault(description, faultType));
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {
    Command selfCheck =
        sequence(runOnce(this::clearFaults), selfCheck().until(this::hasError))
            .withName(getName() + " Self Check");
    return selfCheck;
  }

  private void registerFallibles() {
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
      FaultLogger.register(module.getDriveMotor());
      FaultLogger.register(module.getSteerMotor());
      FaultLogger.register(module.getEncoder());
    }

    FaultLogger.register(getPigeon2());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - _lastSimTime;
              _lastSimTime = currentTime;

              // use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    _simNotifier.setName("Swerve Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  /** Toggles the field oriented boolean. */
  public Command toggleFieldOriented() {
    return brake()
        .withTimeout(0.5)
        .andThen(runOnce(() -> _isFieldOriented = !_isFieldOriented))
        .withName("Toggle Field Oriented");
  }

  /** Brakes the swerve drive (modules form an "X" formation). */
  public Command brake() {
    return run(() -> setControl(_brakeRequest)).withName("Brake");
  }

  /** Resets the heading to face away from the alliance wall. */
  public Command resetHeading() {
    return runOnce(
        () -> {
          Rotation2d rotation =
              DriverStation.getAlliance()
                  .map(
                      allianceColor ->
                          allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero)
                  .orElse(Rotation2d.kZero);

          resetRotation(rotation);
        });
  }

  /**
   * Creates a new Command that drives the chassis.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public Command drive(InputStream velX, InputStream velY, InputStream velOmega) {
    return run(() -> {
          drive(velX.get(), velY.get(), velOmega.get());
        })
        .withName("Drive");
  }

  /**
   * Drives the swerve drive. Open loop/field oriented behavior is configured with {@link
   * #_isOpenLoop} and {@link #_isFieldOriented}.
   *
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    _driverChassisSpeeds.vxMetersPerSecond = velX;
    _driverChassisSpeeds.vyMetersPerSecond = velY;
    _driverChassisSpeeds.omegaRadiansPerSecond = velOmega;

    if (_isFieldOriented) {
      setControl(
          _fieldCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  _isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    } else {
      setControl(
          _robotCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  _isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    }
  }

  /**
   * Sets the chassis state to the given {@link SwerveSample} to aid trajectory following.
   *
   * @param sample The SwerveSample.
   */
  public void followTrajectory(SwerveSample sample) {
    var desiredSpeeds = sample.getChassisSpeeds();
    var desiredPose = sample.getPose();

    desiredSpeeds = _poseController.calculate(desiredSpeeds, desiredPose, getPose());

    setControl(
        _fieldSpeedsRequest
            .withSpeeds(desiredSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /** Finds the proper align pose and tag when aligning. */
  private Pair<AlignPoses, Integer> findAlignment(FieldLocation location) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    AlignPoses alignGoal = FieldConstants.reefFlush;
    AlignPoses alignBaseGoal = FieldConstants.reefFlush;

    int alignTag = FieldConstants.reefTag;

    double minDistance = Double.MAX_VALUE;

    Pose2d robotPose =
        getPose()
            .rotateAround(
                FieldConstants.fieldCenter,
                alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);

    switch (location) {
      case REEF:
        if (getWristevatorGoal() == L1 || getWristevatorGoal() == L4) {
          alignBaseGoal = FieldConstants.reefNotFlush;
        } else {
          alignBaseGoal = FieldConstants.reefFlush;
        }

        for (int i = 0; i < 6; i++) {
          AlignPoses goal =
              alignBaseGoal.rotateAround(
                  FieldConstants.reefCenter, Rotation2d.fromDegrees(-60).times(i));

          if (robotPose.minus(goal.getCenter()).getTranslation().getNorm() < minDistance) {
            alignGoal = goal;
            minDistance = robotPose.minus(goal.getCenter()).getTranslation().getNorm();

            alignTag = FieldConstants.reefTag + i;
          }
        }

        break;

      case HUMAN:
        alignBaseGoal = FieldConstants.human;

        for (int i = 0; i < 2; i++) {
          AlignPoses goal =
              alignBaseGoal.transform(new Translation2d(0, -6.26 * i), Rotation2d.kZero);

          goal =
              goal.rotateAround(
                  goal.getCenter().getTranslation(), Rotation2d.fromDegrees(106).times(i));

          if (robotPose.minus(goal.getCenter()).getTranslation().getNorm() < minDistance) {
            alignGoal = goal;
            minDistance = robotPose.minus(goal.getCenter()).getTranslation().getNorm();

            alignTag = FieldConstants.humanTag - i;
          }
        }

        break;

      case PROCESSOR:
        alignGoal = FieldConstants.processor;
        alignTag = FieldConstants.processorTag;
        break;

      default:
        break;
    }

    alignGoal =
        alignGoal.rotateAround(
            FieldConstants.fieldCenter,
            alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);

    alignTag =
        alliance == Alliance.Blue ? alignTag : FieldConstants.tagCorrespondences.get(alignTag);

    return Pair.of(alignGoal, alignTag);
  }

  /** Backup function that'll manually prepare for reef alignment. */
  public Command resetToReefTag() {
    return Commands.run(
            () -> {
              var tag = findAlignment(FieldLocation.REEF).getSecond();

              _newEstimates.stream()
                  .map(e -> e.singleTagEstimates())
                  .flatMap(e -> Arrays.stream(e))
                  .forEach(
                      e -> {
                        if (e.tag() != tag) return;

                        if (e.distance() < _alignEstimateDistance) {
                          _ignoreVisionEstimates = true;

                          var pose =
                              samplePoseAt(Utils.fpgaToCurrentTime(e.timestamp()))
                                  .orElse(getPose());

                          _alignOdomCompensation =
                              e.pose().toPose2d().getTranslation().minus(pose.getTranslation());

                          _alignEstimateDistance = e.distance();
                        }
                      });
            })
        .until(() -> _alignOdomCompensation != null);
  }

  /** Align to a coral. */
  public Command pieceAlign() {
    return defer(
            () -> {
              Angle tx = Degrees.of(-LimelightHelpers.getTX(VisionConstants.limelightName));
              Angle ty = Degrees.of(LimelightHelpers.getTY(VisionConstants.limelightName));

              RawDetection[] rawDetections =
                  LimelightHelpers.getRawDetections(VisionConstants.limelightName);
              double sideProportions = 0;

              if (rawDetections.length != 0) {
                Rectangle2d coralBox =
                    new Rectangle2d(
                        new Translation2d(rawDetections[0].corner0_X, rawDetections[0].corner0_Y),
                        new Translation2d(rawDetections[0].corner2_X, rawDetections[0].corner2_Y));

                sideProportions = coralBox.getYWidth() / coralBox.getXWidth();
              }

              double groundDistance =
                  (VisionConstants.robotToLimelight.getZ())
                      * Math.tan(
                          (Math.PI / 2)
                              - (VisionConstants.robotToLimelight.getRotation().getY()
                                  - ty.in(Radians)));

              Rotation2d groundAngle =
                  new Rotation2d(
                      Math.atan2(
                          groundDistance * Math.sin(tx.in(Radians)),
                          (groundDistance - VisionConstants.robotToLimelight.getX())
                              * Math.cos(tx.in(Radians))));

              var pose =
                  sideProportions < 1.3
                      ? getPose()
                          .transformBy(
                              new Transform2d(
                                  -groundDistance * groundAngle.getCos(),
                                  -groundDistance * groundAngle.getSin(),
                                  groundAngle.plus(
                                      sideProportions >= 1.3
                                          ? Rotation2d.fromDegrees(45)
                                          : Rotation2d.kZero)))
                      : getPose();

              return driveTo(pose);
            })
        .unless(() -> LimelightHelpers.getTargetCount(VisionConstants.limelightName) == 0)
        .withName("Piece Align");
  }

  /** Aligns to the specified field location. */
  public Command fieldAlign(FieldLocation location, AlignSide side) {
    return defer(
            () -> {
              var alignment = findAlignment(location);

              return alignTo(alignment.getFirst().getPose(side), alignment.getSecond());
            })
        .withName("Field Align");
  }

  /** Aligns to a pose using trig estimate for the robot pose that uses the specified tag id. */
  public Command alignTo(Pose2d goalPose, int tag) {
    return sequence(
            driveTo(goalPose).until(() -> _alignOdomCompensation != null),
            driveTo(
                goalPose,
                () -> {
                  // if it drove all the way to the goal pose but never got a trig estimate
                  if (_alignOdomCompensation == null) return getPose();

                  return new Pose2d(
                      getPose().getTranslation().plus(_alignOdomCompensation), getHeading());
                }))
        .raceWith(
            Commands.run(
                () -> {
                  // update align odom compensation if there are new trig estimates
                  _newEstimates.stream()
                      .map(e -> e.singleTagEstimates())
                      .flatMap(e -> Arrays.stream(e))
                      .forEach(
                          e -> {
                            if (e.tag() != tag) return;

                            if (e.distance() > VisionConstants.trigMaxDistance.in(Meters)) return;

                            if (e.distance() < _alignEstimateDistance) {
                              _ignoreVisionEstimates = true;

                              var pose =
                                  samplePoseAt(Utils.fpgaToCurrentTime(e.timestamp()))
                                      .orElse(getPose());

                              _alignOdomCompensation =
                                  e.pose().toPose2d().getTranslation().minus(pose.getTranslation());

                              _alignEstimateDistance = e.distance();
                            }
                          });
                }))
        .finallyDo(
            () -> {
              _alignEstimateDistance = Double.MAX_VALUE;
              _alignOdomCompensation = null;
              _ignoreVisionEstimates = false; // TODO: reset to old value instead of this
            })
        .withName("Align To");
  }

  /** Drives the robot in a straight line to some given goal pose. Uses the global pose estimate. */
  public Command driveTo(Pose2d goalPose) {
    return driveTo(goalPose, this::getPose);
  }

  /** Drives the robot in a straight line to some given goal pose. */
  private Command driveTo(Pose2d goalPose, Supplier<Pose2d> robotPose) {
    return run(() -> {
          ChassisSpeeds speeds = _poseController.calculate(robotPose.get());

          setControl(_fieldSpeedsRequest.withSpeeds(speeds));
        })
        .beforeStarting(
            () ->
                _poseController.reset(
                    robotPose.get(),
                    goalPose,
                    ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading())))
        .until(_poseController::isFinished)
        .withName("Drive To");
  }

  /** Wrapper for getting estimated pose. */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /** Wrapper for getting estimated heading. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Returns the robot's estimated rotation at the given timestamp. This timestamp must be in FPGA
   * time.
   */
  public Rotation2d getHeadingAtTime(double timestamp) {
    return samplePoseAt(Utils.fpgaToCurrentTime(timestamp)).orElse(getPose()).getRotation();
  }

  /** Wrapper for getting current robot-relative chassis speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  // updates pose estimator with vision
  private void updateVisionPoseEstimates() {
    _acceptedEstimates.clear();
    _rejectedEstimates.clear();

    _newEstimates.clear();

    _detectedTags.clear();

    _estimates.clear();

    for (VisionPoseEstimator cam : _cameras) {
      cam.update();

      _estimates = cam.getNewEstimates();

      // TEMPORARY CAMERA PLACEMENT VISUALIZATION:
      // DogLog.log(
      //     "Swerve/" + cam.camName + " Position",
      //     new Pose3d(getPose())
      //         .transformBy(new Transform3d(0.0, 0.0, 0.1, Rotation3d.kZero))
      //         .transformBy(cam.robotToCam));

      // add estimates to arrays and update detected tags
      _estimates.forEach(
          (estimate) -> {
            // add all detected tag poses
            for (int id : estimate.detectedTags()) {
              FieldConstants.tagLayout.getTagPose(id).ifPresent(pose -> _detectedTags.add(pose));
            }

            // add robot poses to their corresponding arrays
            if (estimate.isValid()) _acceptedEstimates.add(estimate);
            else _rejectedEstimates.add(estimate);
          });

      _newEstimates.addAll(_acceptedEstimates);
      _newEstimates.addAll(_rejectedEstimates);
    }
  }

  @Override
  public void periodic() {
    updateVisionPoseEstimates();

    if (!_hasAppliedDriverPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);

                _hasAppliedDriverPerspective = true;
              });
    }

    DogLog.log(
        "Swerve/Accepted Estimates",
        _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));
    DogLog.log(
        "Swerve/Rejected Estimates",
        _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));

    DogLog.log("Swerve/Detected Tags", _detectedTags.toArray(Pose3d[]::new));

    if (!_ignoreVisionEstimates) {
      _acceptedEstimates.sort(VisionPoseEstimate.sorter);

      _acceptedEstimates.forEach(
          (e) -> {
            var stdDevs = e.stdDevs();
            addVisionMeasurement(
                e.pose().toPose2d(),
                Utils.fpgaToCurrentTime(e.timestamp()),
                VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
          });
    }
  }

  @Override
  public void simulationPeriodic() {
    _visionSystemSim.update(getPose());
  }

  // TODO: add self check routines
  private Command selfCheckModule(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    return shiftSequence();
  }

  @Override
  public Command selfCheck() {
    return shiftSequence(
        // check all modules individually
        selfCheckModule("Front Left", getModule(0)),
        selfCheckModule("Front Right", getModule(1)),
        selfCheckModule("Back Left", getModule(2)),
        selfCheckModule("Back Right", getModule(3)));
  }

  @Override
  public void close() {
    super.close();

    _cameras.forEach(cam -> cam.close());

    _simNotifier.close();
  }
}
