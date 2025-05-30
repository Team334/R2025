package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class HolonomicController {
  // generate the path for the robot to follow
  private final ProfiledPIDController _translationProfile =
      new ProfiledPIDController(0, 0, 0, new Constraints(1, 2));
  private final ProfiledPIDController _headingProfile =
      new ProfiledPIDController(0, 0, 0, new Constraints(Math.PI, Math.PI * 2));

  private Vector<N2> _translationDirection = VecBuilder.fill(0, 0);

  private Pose2d _startPose = Pose2d.kZero;
  private Pose2d _goalPose = Pose2d.kZero;

  // used to follow the path
  private final PIDController _xController = new PIDController(0.8, 0, 0.01);
  private final PIDController _yController = new PIDController(0.8, 0, 0.01);

  private final PIDController _headingController = new PIDController(1.0, 0, 0.01);

  public HolonomicController() {
    _headingProfile.enableContinuousInput(-Math.PI, Math.PI);
    _headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Whether the translation and rotation profiles have completed. */
  public boolean isFinished() {
    return MathUtil.isNear(
            _translationProfile.getGoal().position,
            _translationProfile.getSetpoint().position,
            0.001)
        && MathUtil.isNear(
            _translationProfile.getGoal().velocity,
            _translationProfile.getSetpoint().velocity,
            0.001)
        && MathUtil.isNear(
            _headingProfile.getGoal().position, _headingProfile.getSetpoint().position, 0.001)
        && MathUtil.isNear(
            _headingProfile.getGoal().velocity, _headingProfile.getSetpoint().velocity, 0.001);
  }

  /**
   * Resets the translation and rotation profiles given the current speeds, pose, and the goal pose.
   */
  public void reset(Pose2d currentPose, Pose2d goalPose, ChassisSpeeds currentSpeeds) {
    // vector where head is at goal pose and tail is at current pose
    _translationDirection =
        VecBuilder.fill(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());

    _translationProfile.reset(
        0,
        _translationDirection.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / _translationDirection.norm());

    _headingProfile.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

    _xController.reset();
    _yController.reset();

    _headingController.reset();

    _startPose = currentPose;
    _goalPose = goalPose;
  }

  /**
   * Samples the motion profiles at the next timestep. The motion profile ends at the desired goal
   * pose at a chassis speeds of 0.
   *
   * @param currentPose The current pose of the chassis necessary for PID.
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    _headingProfile.calculate(
        currentPose.getRotation().getRadians(),
        _goalPose
            .getRotation()
            .getRadians()); // measurement might matter for continous heading thing?
    _translationProfile.calculate(
        0,
        _translationDirection
            .norm()); // measurement doesn't matter, handled by individual controllers

    Pose2d setpoint =
        new Pose2d(
            _startPose.getX()
                + _translationDirection
                    .unit()
                    .times(_translationProfile.getSetpoint().position)
                    .get(0),
            _startPose.getY()
                + _translationDirection
                    .unit()
                    .times(_translationProfile.getSetpoint().position)
                    .get(1),
            new Rotation2d(_headingProfile.getSetpoint().position));

    return calculate(
        new ChassisSpeeds(
            _translationDirection.unit().times(_translationProfile.getSetpoint().velocity).get(0),
            _translationDirection.unit().times(_translationProfile.getSetpoint().velocity).get(1),
            _headingProfile.getSetpoint().velocity),
        setpoint,
        currentPose);
  }

  /**
   * Modifies some base chassis speeds the drive is currently traveling at to bring the drive closer
   * to a desired pose.
   *
   * @param baseSpeeds The field-relative speed the drive is already traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose of the drive.
   * @return New modified speeds.
   */
  public ChassisSpeeds calculate(ChassisSpeeds baseSpeeds, Pose2d desiredPose, Pose2d currentPose) {
    DogLog.log("Auto/Controller Desired Pose", desiredPose);
    DogLog.log("Auto/Controller Reference Pose", currentPose);

    return baseSpeeds.plus(
        new ChassisSpeeds(
            _xController.calculate(currentPose.getX(), desiredPose.getX()),
            _yController.calculate(currentPose.getY(), desiredPose.getY()),
            _headingController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians())));
  }
}
