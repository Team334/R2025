package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
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
  private final ProfiledPIDController _translationProfile =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              MetersPerSecond.of(3).in(MetersPerSecond),
              MetersPerSecondPerSecond.of(4).in(MetersPerSecondPerSecond)));
  private final ProfiledPIDController _headingProfile =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              RadiansPerSecond.of(Math.PI).in(RadiansPerSecond),
              RadiansPerSecondPerSecond.of(Math.PI * 2).in(RadiansPerSecondPerSecond)));

  private Vector<N2> _translationDirection = VecBuilder.fill(0, 0);

  private Pose2d _startPose = Pose2d.kZero;
  private double _goalHeading = 0;

  private final PIDController _xController = new PIDController(0, 0, 0);
  private final PIDController _yController = new PIDController(0, 0, 0);

  private final PIDController _headingController = new PIDController(0, 0, 0);

  public HolonomicController() {
    _headingController.enableContinuousInput(-Math.PI, Math.PI);
    _headingProfile.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Whether the chassis profiles have been completed or not. */
  public boolean isFinished() {
    return _translationProfile.getSetpoint().equals(_translationProfile.getGoal())
        && _headingProfile.getSetpoint().equals(_headingProfile.getGoal());
  }

  /** Resets the PID controllers. */
  public void reset() {
    _xController.reset();
    _yController.reset();

    _headingController.reset();
  }

  /**
   * Resets the motion profiles and the PID controllers.
   *
   * @param currentPose The current pose.
   * @param goalPose The goal pose.
   * @param currentSpeeds The current field-relative speeds of the chassis.
   */
  public void reset(Pose2d currentPose, Pose2d goalPose, ChassisSpeeds currentSpeeds) {
    _translationDirection =
        VecBuilder.fill(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());

    _translationProfile.reset(
        0,
        _translationDirection.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / _translationDirection.norm());
    _headingProfile.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

    reset();

    _startPose = currentPose;
    _goalHeading = goalPose.getRotation().getRadians();
  }

  /**
   * Samples the motions profiles at the next timestep, finding chassis speeds based on the profiles
   * and PID correction.
   *
   * @param currentPose The current pose.
   * @return Field-relative speeds for the chassis.
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    _translationProfile.calculate(
        0,
        _translationDirection.norm()); // measurement doesn't matter, handled by xy PID controllers
    _headingProfile.calculate(
        currentPose.getRotation().getRadians(), _goalHeading); // TODO: why does measurement matter?

    Vector<N2> setpointPosition =
        _translationDirection.unit().times(_translationProfile.getSetpoint().position);
    Vector<N2> setpointVelocity =
        _translationDirection.unit().times(_translationProfile.getSetpoint().velocity);

    Pose2d setpointPose =
        new Pose2d(
            _startPose.getX() + setpointPosition.get(0),
            _startPose.getY() + setpointPosition.get(1),
            new Rotation2d(_headingProfile.getSetpoint().position));

    return calculate(
        new ChassisSpeeds(
            setpointVelocity.get(0),
            setpointVelocity.get(1),
            _headingProfile.getSetpoint().velocity),
        setpointPose,
        currentPose);
  }

  /**
   * Modifies some base field-relative chassis speeds the chassis is currently traveling at to bring
   * it closer to the desired pose.
   *
   * @param baseSpeeds The field-relative speed the chassis is already traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose.
   * @return New modified field-relative speeds.
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
