// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.WristevatorConstants.Intermediate.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.AlignPoses;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;
import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Frequency simUpdateFrequency = Hertz.of(200);

  public static final String canivore = "CTRE";

  public static class Ports {
    public static final int driverController = 0;
    public static final int operatorController = 1;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Map<Integer, Integer> tagCorrespondences = new HashMap<Integer, Integer>();

    static {
      tagCorrespondences.put(-1, -1);
      tagCorrespondences.put(13, 1);
      tagCorrespondences.put(12, 2);
      tagCorrespondences.put(18, 7);
      tagCorrespondences.put(17, 8);
      tagCorrespondences.put(22, 9);
      tagCorrespondences.put(21, 10);
      tagCorrespondences.put(20, 11);
      tagCorrespondences.put(19, 6);
      tagCorrespondences.put(16, 3);
      tagCorrespondences.put(14, 4);
      tagCorrespondences.put(15, 5);
    }

    public static final Translation2d reefCenter =
        new Translation2d(Inches.of(176.75).in(Meters), Inches.of(158.5).in(Meters));

    public static final Translation2d fieldCenter =
        new Translation2d(tagLayout.getFieldLength() / 2, tagLayout.getFieldWidth() / 2);

    public static final Translation2d humanCenter =
        new Translation2d(Inches.of(47.93).in(Meter), Inches.of(158.28).in(Meters));

    public static final int reefTag = 17;

    public static final AlignPoses reef =
        new AlignPoses(
            new Pose2d(3.68, 2.95, Rotation2d.fromDegrees(60)),
            new Pose2d(3.839, 2.885, Rotation2d.fromDegrees(60)),
            new Pose2d(3.249, 3.778, Rotation2d.fromDegrees(0))
                .rotateAround(reefCenter, Rotation2d.fromDegrees(60)));

    public static final int humanTag = 13;

    public static final AlignPoses human =
        new AlignPoses(
            new Pose2d(
                Inches.of(23.14).in(Meters),
                Inches.of(264.66).in(Meters),
                new Rotation2d(Degrees.of(126))),
            new Pose2d(
                Inches.of(41.17).in(Meters),
                Inches.of(282.69).in(Meters),
                new Rotation2d(Degrees.of(126))),
            new Pose2d(
                Inches.of(64.61).in(Meters),
                Inches.of(294.86).in(Meters),
                new Rotation2d(Degrees.of(126))));

    public static final int processorTag = 16;

    public static final AlignPoses processor =
        new AlignPoses(
            new Pose2d(
                Inches.of(233.7).in(Meters), Inches.of(16.2).in(Meters), Rotation2d.kCW_90deg));

    public static final AlignPoses cage =
        new AlignPoses(
            new Pose2d(
                Inches.of(324.95).in(Meters), Inches.of(285.84).in(Meters), Rotation2d.kCCW_90deg),
            new Pose2d(
                Inches.of(324.95).in(Meters), Inches.of(241.89).in(Meters), Rotation2d.kCCW_90deg),
            new Pose2d(
                Inches.of(324.95).in(Meters), Inches.of(200.16).in(Meters), Rotation2d.kCCW_90deg));
  }

  public static class VisionConstants {

    public static final String lowerLeftArducamName = "lower-left-arducam";
    public static final String lowerRightArducamName = "lower-right-arducam";
    public static final String upperLeftArducamName = "upper-left-arducam";
    public static final String upperRightArducamName = "upper-right-arducam";
    public static final String limelightName = "limelight-main";

    public static final double[] singleTagBaseStdDevs = new double[] {5, 5, 5};
    public static final double[] multiTagBaseStdDevs = new double[] {1, 1, 1};

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.1;

    public static final Distance trigMaxDistance = Meters.of(1.2);

    public static final VisionPoseEstimatorConstants lowerLeftArducam =
        new VisionPoseEstimatorConstants(
            lowerLeftArducamName,
            new Transform3d(
                new Translation3d(0.345, 0.285, 0.232),
                new Rotation3d(0, -Units.degreesToRadians(18.6), 0)),
            0.2,
            0.2,
            2.5,
            4.5);

    public static final VisionPoseEstimatorConstants lowerRightArducam =
        new VisionPoseEstimatorConstants(
            lowerRightArducamName,
            new Transform3d(
                new Translation3d(0.345, -0.285, 0.232),
                new Rotation3d(0, -Units.degreesToRadians(17.8), 0)),
            0.2,
            0.2,
            2.5,
            4.5);

    public static final VisionPoseEstimatorConstants upperLeftArducam =
        new VisionPoseEstimatorConstants(
            upperLeftArducamName,
            new Transform3d(
                new Translation3d(0.154, 0.273, 0.972),
                new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
            0.2,
            0.2,
            2.5,
            4.5);

    public static final VisionPoseEstimatorConstants upperRightArducam =
        new VisionPoseEstimatorConstants(
            upperRightArducamName,
            new Transform3d(
                new Translation3d(0.154, -0.273, 0.972),
                new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
            0.2,
            0.2,
            2.5,
            4.5);

    public static final Transform3d robotToLimelight =
        new Transform3d(0.063, 0, 0.968, new Rotation3d(0, Units.degreesToRadians(45), Math.PI));
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Distance driveRadius =
        Meters.of(
            Math.sqrt(
                Math.pow(TunerConstants.FrontLeft.LocationX, 2)
                    + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));

    public static final LinearVelocity maxTranslationalSpeed = MetersPerSecond.of(3.5);
    public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(Math.PI * 2);

    public static final LinearVelocity translationalDeadband = maxTranslationalSpeed.times(0.01);
    public static final AngularVelocity rotationalDeadband = maxAngularSpeed.times(0.01);

    public static final Distance pathingDistanceThreshold = Meters.of(0.4);
  }

  public static class IntakeConstants {
    public static final int feedMotorId = 15;
    public static final int actuatorMotorId = 8;

    public static final Voltage feedkS = Volts.of(0.32749);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.22873);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.62406);

    public static final Voltage actuatorkG = Volts.of(0.2127);
    public static final Voltage actuatorkS = Volts.of(0.14445);

    public static final Per<VoltageUnit, AngularVelocityUnit> actuatorkV =
        Volts.per(RotationsPerSecond).ofNative(5.8131);
    public static final Per<VoltageUnit, AngularAccelerationUnit> actuatorkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.10315);

    public static final Per<VoltageUnit, AngleUnit> actuatorkP =
        Volts.per(Rotations).ofNative(2.804);

    public static final AngularVelocity actuatorVelocity = RotationsPerSecond.of(2);
    public static final AngularAcceleration actuatorAcceleration =
        RotationsPerSecondPerSecond.of(3);

    public static final double feedGearRatio = 32 / 18.0;
    public static final double actuatorGearRatio = 50;

    public static final Distance intakeLength = Inches.of(15);

    public static final Angle actuatorStowed = Radians.of(2.26);
    public static final Angle actuatorOut = Radians.of(-0.34);

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(50);
  }

  public static class WristevatorConstants {
    /** Represents a setpoint for the wristevator. */
    public static interface Setpoint {
      /** The angle of the wrist. */
      public Angle getAngle();

      /** The angle turned of the elevator drum. */
      public Angle getHeight();
    }

    /** Wristevator presets. */
    public static enum Preset implements Setpoint {
      HOME(Radians.of(-1.06), Radians.of(0)),
      HUMAN(Radians.of(-0.457), Radians.of(17.984)),
      PROCESSOR(Radians.of(-1.06), Radians.of(0)),

      L1(Radians.of(-0.233), Radians.of(2.095)),
      L2(Radians.of(-0.793), Radians.of(14.769)),
      L3(Radians.of(-1.06), Radians.of(26.828)),
      L4(Radians.of(1.282), Radians.of(38.2)),

      LOWER_ALGAE(Radians.of(-0.877), Radians.of(17.969)),
      UPPER_ALGAE(Radians.of(-1.06), Radians.of(30));

      private final Angle _angle;
      private final Angle _height;

      private Preset(Angle angle, Angle height) {
        _angle = angle;
        _height = height;
      }

      @Override
      public Angle getAngle() {
        return _angle;
      }

      @Override
      public Angle getHeight() {
        return _height;
      }
    }

    /** Wristevator intermediate setpoints. */
    public static enum Intermediate implements Setpoint {
      INFINITY(Radians.of(Integer.MAX_VALUE), Radians.of(Integer.MAX_VALUE)),
      I1(Radians.of(-1.06), Radians.of(3)),
      I2(Radians.of(-1.06), Radians.of(36.92)),
      I3(Radians.of(-1.06), Radians.of(30));

      private final Angle _angle;
      private final Angle _height;

      private Intermediate(Angle angle, Angle height) {
        _angle = angle;
        _height = height;
      }

      @Override
      public Angle getAngle() {
        return _angle;
      }

      @Override
      public Angle getHeight() {
        return _height;
      }
    }

    public static final HashMap<Pair<Setpoint, Setpoint>, Setpoint> setpointMap = new HashMap<>();

    static {
      // going to a upwards wrist angle from home
      setpointMap.put(Pair.of(HOME, L1), I1);
      setpointMap.put(Pair.of(HOME, L4), I2);

      // going up to l4
      setpointMap.put(Pair.of(I1, L4), I2);
      setpointMap.put(Pair.of(L1, L4), I2);
      setpointMap.put(Pair.of(L2, L4), I2);
      setpointMap.put(Pair.of(L3, L4), I2);
      setpointMap.put(Pair.of(LOWER_ALGAE, L4), I2);
      setpointMap.put(Pair.of(UPPER_ALGAE, L4), I2);
      setpointMap.put(Pair.of(HUMAN, L4), I2);
      setpointMap.put(Pair.of(PROCESSOR, L4), I2);

      // going down from l4
      setpointMap.put(Pair.of(L4, L3), I2);
      setpointMap.put(Pair.of(L4, L2), I2);
      setpointMap.put(Pair.of(L4, L1), I2);
      setpointMap.put(Pair.of(L4, HOME), I2);
      setpointMap.put(Pair.of(L4, UPPER_ALGAE), I2);
      setpointMap.put(Pair.of(L4, LOWER_ALGAE), I2);
      setpointMap.put(Pair.of(L4, HUMAN), I2);
      setpointMap.put(Pair.of(L4, PROCESSOR), I2);
    }

    public static final AngularVelocity maxWristSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity maxElevatorSpeed = RotationsPerSecond.of(14);

    public static final AngularVelocity manualWristSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity manualElevatorSpeed = RotationsPerSecond.of(3);

    public static final AngularAcceleration maxWristAcceleration =
        RotationsPerSecondPerSecond.of(3);
    public static final AngularAcceleration maxElevatorAcceleration =
        RotationsPerSecondPerSecond.of(15);

    public static final int homeSwitch = 7;

    public static final int leftMotorId = 12;
    public static final int rightMotorId = 9;
    public static final int wristMotorId = 13;

    public static final double elevatorGearRatio = 9;

    public static final Distance drumRadius = Inches.of(1.504 / 2);
    public static final Distance drumCircumference = drumRadius.times(2 * Math.PI);

    public static final Angle minElevatorHeight = Radians.of(0);
    public static final Angle maxElevatorHeight = Radians.of(38.5);

    public static final Distance manipulatorLength = Meters.of(0.18415);

    public static final Angle minWristAngle = Radians.of(-1.1);
    public static final Angle maxWristAngle = Radians.of(1.32);

    public static final double wristGearRatio = 33.75;

    public static final Voltage elevatorkS = Volts.of(0.057311);
    public static final Voltage elevatorkG = Volts.of(0.32228);
    public static final Per<VoltageUnit, AngularVelocityUnit> elevatorkV =
        Volts.per(RotationsPerSecond).ofNative(1.1185);
    public static final Per<VoltageUnit, AngularAccelerationUnit> elevatorkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.026679);

    public static final Per<VoltageUnit, AngleUnit> elevatorkP =
        Volts.per(Rotations).ofNative(6.7372); // 15 from sysid

    public static final Voltage wristkS = Volts.of(0.1);
    public static final Voltage wristkG = Volts.of(0.3);
    public static final Per<VoltageUnit, AngularVelocityUnit> wristkV =
        Volts.per(RotationsPerSecond).ofNative(4.05);
    public static final Per<VoltageUnit, AngularAccelerationUnit> wristkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.1566);

    public static final Per<VoltageUnit, AngleUnit> wristkP =
        Volts.per(Rotations).ofNative(13.082); // 17.221 from sysid
  }

  public static class SerializerConstants {
    public static final int frontBeamPort = 6;

    public static final Voltage feedkS = Volts.of(0.30489);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.28324);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.41102);

    // ka = 0.0091039

    public static final double feedGearRatio = 70.0 / 30;

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(20);
    public static final AngularVelocity passoffSpeed = RadiansPerSecond.of(35);

    public static final int feedMotorId = 10;
  }

  public static class ManipulatorConstants {
    public static final int leftMotorId = 14;
    public static final int rightMotorId = 11;

    public static final int coralBeam = 9;
    public static final int algaeBeam = 4;

    public static final AngularVelocity intakeSlowSpeed = RadiansPerSecond.of(16);
    public static final AngularVelocity intakeFastSpeed = RadiansPerSecond.of(30);

    public static final AngularVelocity outtakeSpeed = RadiansPerSecond.of(-40);

    public static final AngularVelocity passoffSpeed = RadiansPerSecond.of(10);

    public static final Voltage leftFlywheelkS = Volts.of(0.44229);
    public static final Per<VoltageUnit, AngularVelocityUnit> leftFlywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.26);
    public static final Per<VoltageUnit, AngularVelocityUnit> leftFlywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.16712);

    // left wheel ka = 0.009767

    public static final Voltage rightFlywheelkS = Volts.of(0.24209);
    public static final Per<VoltageUnit, AngularVelocityUnit> rightFlywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.29905);
    public static final Per<VoltageUnit, AngularVelocityUnit> rightFlywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.02354);

    // right wheel meow ka = 0.0096618

    public static final double flywheelGearRatio = 3;

    public static final Voltage holdAlgaeVoltage = Volts.of(0.6);
  }
}
