// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.AlignPoses;
import frc.robot.utils.AlignPoses.AlignSide;
import frc.robot.utils.HolonomicController;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SysId;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
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
  private final SysIdSwerveRotation _rotationSysIdRequest =
      new SysIdSwerveRotation(); // how does this work?

  // SysId routine for characterizing translation. This is used to find PID gains for the drive
  // motors.
  private final SysIdRoutine _sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              state ->
                  SignalLogger.writeString(
                      "Swerve SysId Translation Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_translationSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
  private final SysIdRoutine _sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              state ->
                  SignalLogger.writeString("Swerve SysId Steer Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_steerSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for characterizing rotation.
  private final SysIdRoutine _sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              // This is in radians per second², but SysId only supports "volts per second"
              Volts.of(Math.PI / 6).per(Second),
              // This is in radians per second, but SysId only supports "volts"
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              state ->
                  SignalLogger.writeString(
                      "Swerve SysId Rotation Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                // output is actually radians per second, but SysId only supports "volts"
                setControl(_rotationSysIdRequest.withRotationalRate(output.in(Volts)));
                // also log the requested output for SysId
                SignalLogger.writeDouble("Swerve Rotational Rate", output.in(Volts));
              },
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
  private boolean _ignoreVisionEstimates = true; // for sim for now

  private AlignPoses _alignGoal = new AlignPoses(Pose2d.kZero);

  private Pose2d _pieceAlignPose;

  private HolonomicController _poseController = new HolonomicController();

  private boolean _hasAppliedDriverPerspective;

  // cameras and vision measurements
  @Logged(name = VisionConstants.arducamOneName)
  private final VisionPoseEstimator _arducamOne =
      VisionPoseEstimator.buildFromConstants(VisionConstants.arducamOne, this::getHeadingAtTime);

  private final List<VisionPoseEstimator> _cameras = List.of(_arducamOne);

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<>();
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<>();

  private final List<VisionPoseEstimate> _allEstimates = new ArrayList<>();

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

    _poseController.setTolerance(Meters.of(0.1), Rotation2d.fromDegrees(0));

    // display all sysid routines
    SysId.displayRoutine("Swerve Translation", _sysIdRoutineTranslation);
    SysId.displayRoutine("Swerve Steer", _sysIdRoutineSteer);
    SysId.displayRoutine("Swerve Rotation", _sysIdRoutineRotation);

    registerFallibles();

    if (Robot.isSimulation()) {
      startSimThread();

      _visionSystemSim = new VisionSystemSim("Vision System Sim");
      _visionSystemSim.addAprilTags(FieldConstants.tagLayout);

      _arducamOne
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

  /** Resets the heading to zero */
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

    // go through a couple of steps to ensure that input speeds are actually achievable
    ChassisSpeeds tempSpeeds = _driverChassisSpeeds;
    SwerveModuleState[] tempStates;

    // TODO: this might be too many memory allocations

    if (_isFieldOriented)
      tempSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(tempSpeeds, getHeading());

    tempStates = getKinematics().toSwerveModuleStates(tempSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(tempStates, SwerveConstants.maxTranslationalSpeed);
    tempSpeeds = getKinematics().toChassisSpeeds(tempStates);

    if (_isFieldOriented)
      tempSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(tempSpeeds, getHeading());

    velX = tempSpeeds.vxMetersPerSecond;
    velY = tempSpeeds.vyMetersPerSecond;
    velOmega = tempSpeeds.omegaRadiansPerSecond;

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

  /** Make the chassis align to a piece. */
  public Command alignToPiece() {
    return runOnce(
            () -> {
              double tx = LimelightHelpers.getTX(VisionConstants.limelightName);
              double ty = LimelightHelpers.getTY(VisionConstants.limelightName);

              double groundDistance =
                  (VisionConstants.robotToLimelight.getY()
                          + SwerveConstants.chassisHeight.in(Meters))
                      * Math.tan(VisionConstants.robotToLimelight.getRotation().getY() - ty);
              Rotation2d groundAngle = getHeading().plus(Rotation2d.fromDegrees(tx));

              _pieceAlignPose =
                  getPose()
                      .transformBy(
                          new Transform2d(
                              groundDistance * groundAngle.getCos(),
                              groundDistance * groundAngle.getSin(),
                              Rotation2d.fromDegrees(tx)));
            })
        .andThen(defer(() -> driveTo(_pieceAlignPose)))
        .withName("Align To Piece");
  }

  /**
   * Aligns to a {@link AlignPoses} to the correct side.
   *
   * @return Drive to the correct pose.
   */
  public Command alignTo(AlignPoses alignGoal, AlignSide side) {
    return runOnce(
            () -> {
              Pose2d pose = getPose();
              Optional<Alliance> alliance = DriverStation.getAlliance();

              AlignPoses rotated =
                  alliance.get() == Alliance.Red
                      ? alignGoal.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
                      : alignGoal;

              _alignGoal = rotated;

              if (alignGoal == FieldConstants.reef) {
                double minDistance = Double.MAX_VALUE;

                var reefCenter =
                    alliance.get() == Alliance.Red
                        ? FieldConstants.reefCenter.rotateAround(
                            FieldConstants.fieldCenter, Rotation2d.k180deg)
                        : FieldConstants.reefCenter;

                for (int i = 0; i < 6; i++) {
                  var goal = rotated.rotateAround(reefCenter, Rotation2d.fromDegrees(60).times(i));

                  if (pose.minus(goal.getCenter()).getTranslation().getNorm() < minDistance) {
                    _alignGoal = goal;
                    minDistance = pose.minus(goal.getCenter()).getTranslation().getNorm();
                  }
                }
              }

              if (alignGoal == FieldConstants.human) {
                double minDistance = Double.MAX_VALUE;

                for (int i = 0; i < 2; i++) {
                  AlignPoses offset = alignGoal.offset(0, -6.26 * i);

                  offset =
                      alliance.get() == Alliance.Red
                          ? offset.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
                          : offset;

                  offset =
                      offset.rotateAround(
                          offset.getCenter().getTranslation(),
                          Rotation2d.fromDegrees(106).times(i));

                  if (pose.minus(offset.getCenter()).getTranslation().getNorm() < minDistance) {
                    _alignGoal = offset;
                    minDistance = pose.minus(offset.getCenter()).getTranslation().getNorm();
                  }
                }
              }
            })
        .andThen(defer(() -> driveTo(_alignGoal.getPose(side))))
        .withName("Align To");
  }

  /** Drives the robot in a straight line to some given goal pose. */
  public Command driveTo(Pose2d goalPose) {
    return run(() -> {
          ChassisSpeeds speeds = _poseController.calculate(getPose(), goalPose);

          setControl(_fieldSpeedsRequest.withSpeeds(speeds));
        })
        .beforeStarting(
            () ->
                _poseController.reset(
                    getPose(),
                    goalPose,
                    ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading())))
        .until(_poseController::atGoal)
        .withName("Drive To");
  }

  /**
   * Returns the robot's field-relative pose using trig. This estimate uses the side-view and
   * top-view right triangles formed between the robot and the specified tag to estimate the robot's
   * pose. For this to work, a tag must by in view of one of the {@link VisionPoseEstimator}s.
   *
   * @param tagId The tag id to use for the trig estimate.
   * @return The trig estimate (which also gets latency compensated). If the tag doesn't exist or
   *     isn't in view of any of the cameras, the normal pose estimator pose is returned as a
   *     default.
   */
  public Pose2d getTrigPose(int tagId) {
    if (FieldConstants.tagLayout.getTagPose(tagId).isEmpty()) return getPose();

    VisionPoseEstimate[] tagIsVisible =
        _allEstimates.stream()
            .filter(e -> Arrays.asList(e.detectedTags()).contains((Object) tagId))
            .toArray(VisionPoseEstimate[]::new);

    if (tagIsVisible.length == 0) return getPose();

    VisionPoseEstimate visionEstimate = tagIsVisible[0];

    Pose3d tagPose = FieldConstants.tagLayout.getTagPose(tagId).get();

    double distance = visionEstimate.avgTagDistance();

    double tx = visionEstimate.tx();
    double ty = visionEstimate.ty();

    // TODO: trig over here using the above values
    Pose2d trigPose = Pose2d.kZero;

    Pose2d oldPose =
        samplePoseAt(Utils.fpgaToCurrentTime(visionEstimate.timestamp())).orElse(getPose());

    return trigPose.transformBy(getPose().minus(oldPose));
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

    _allEstimates.clear();

    _detectedTags.clear();

    for (VisionPoseEstimator cam : _cameras) {
      cam.update();

      var estimates = cam.getNewEstimates();

      DogLog.log(
          "Swerve/" + cam.camName + " Position", new Pose3d(getPose()).transformBy(cam.robotToCam));

      // process estimates
      estimates.forEach(
          (estimate) -> {
            // add all detected tag poses
            for (int id : estimate.detectedTags()) {
              FieldConstants.tagLayout.getTagPose(id).ifPresent(pose -> _detectedTags.add(pose));
            }

            // add robot poses to their corresponding arrays
            if (estimate.isValid()) _acceptedEstimates.add(estimate);
            else _rejectedEstimates.add(estimate);
          });

      _allEstimates.addAll(_acceptedEstimates);
      _allEstimates.addAll(_rejectedEstimates);
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
    _visionSystemSim.update(getPose()); // TODO: this might require a seperate wheel-only odom
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
