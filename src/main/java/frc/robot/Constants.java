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
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.AlignPoses;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;
import java.util.HashMap;

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
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Translation2d reefCenter =
        new Translation2d(Inches.of(176.75).in(Meters), Inches.of(158.5).in(Meters));

    public static final Translation2d fieldCenter =
        new Translation2d(tagLayout.getFieldLength() / 2, tagLayout.getFieldWidth() / 2);

    public static final Translation2d humanCenter =
        new Translation2d(Inches.of(47.93).in(Meter), Inches.of(158.28).in(Meters));

    public static final int blueReefTag = 18;
    public static final int redReefTag = 7;

    public static final AlignPoses reef =
        new AlignPoses(
            new Pose2d(Inches.of(101.3).in(Meters), Inches.of(170.5).in(Meters), Rotation2d.kZero),
            new Pose2d(Inches.of(101.3).in(Meters), Inches.of(158.5).in(Meters), Rotation2d.kZero),
            new Pose2d(Inches.of(101.3).in(Meters), Inches.of(146.5).in(Meters), Rotation2d.kZero));

    public static final int blueHumanTag = 13;
    public static final int redHumanTag = 1;

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

    public static final int blueProcessorTag = 16;
    public static final int redProcessorTag = 3;

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

    public static final double[] singleTagBaseStdDevs = new double[] {5, 5, 5};
    public static final double[] multiTagBaseStdDevs = new double[] {1, 1, 1};

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.01;

    public static final Distance trigMaxDistance = Meters.of(1.5);

    public static final VisionPoseEstimatorConstants lowerLeftArducam =
        new VisionPoseEstimatorConstants(
            lowerLeftArducamName,
            new Transform3d(
                new Translation3d(0.345, 0.285, 0.102),
                new Rotation3d(0, -Units.degreesToRadians(16.96), 0)),
            0.2,
            0.0001,
            3,
            7);

    public static final VisionPoseEstimatorConstants lowerRightArducam =
        new VisionPoseEstimatorConstants(
            lowerRightArducamName,
            new Transform3d(
                new Translation3d(0.345, -0.285, 0.102),
                new Rotation3d(0, -Units.degreesToRadians(16.96), 0.2)),
            0.2,
            0.0001,
            3,
            7);

    public static final VisionPoseEstimatorConstants upperLeftArducam =
        new VisionPoseEstimatorConstants(
            upperLeftArducamName,
            new Transform3d(
                new Translation3d(0.154, 0.273, 0.809),
                new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
            0.2,
            0.0001,
            3,
            7);

    public static final VisionPoseEstimatorConstants upperRightArducam =
        new VisionPoseEstimatorConstants(
            upperRightArducamName,
            new Transform3d(
                new Translation3d(0.154, -0.273, 0.809),
                new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
            0.2,
            0.0001,
            3,
            7);
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Distance driveRadius =
        Meters.of(
            Math.sqrt(
                Math.pow(TunerConstants.FrontLeft.LocationX, 2)
                    + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));

    public static final LinearVelocity maxTranslationalSpeed = MetersPerSecond.of(3.632);
    public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(Math.PI);

    // respecting wheel COF and max motor torque (this can be obtained from choreo probably)
    public static final LinearAcceleration maxTranslationalAcceleration =
        MetersPerSecondPerSecond.of(14.715);
    public static final AngularAcceleration maxAngularAcceleration =
        RadiansPerSecondPerSecond.of(Math.PI * 3);

    public static final LinearVelocity translationalDeadband = maxTranslationalSpeed.times(0.1);
    public static final AngularVelocity rotationalDeadband = maxAngularSpeed.times(0.1);

    public static final Distance pathingThreshold = Meters.of(1.5);
  }

  public static class IntakeConstants {
    public static final int feedMotorId = 15;
    public static final int actuatorMotorId = 8;

    public static final Voltage feedkS = Volts.of(0.23179);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.22463);

    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.34897);

    public static final Voltage actuatorkG = Volts.of(0.18859);
    public static final Voltage actuatorkS = Volts.of(0.070134);

    public static final Per<VoltageUnit, AngularVelocityUnit> actuatorkV =
        Volts.per(RotationsPerSecond).ofNative(5.7349);
    public static final Per<VoltageUnit, AngularAccelerationUnit> actuatorkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.057226);

    public static final Per<VoltageUnit, AngleUnit> actuatorkP =
        Volts.per(Rotations).ofNative(12.934);

    public static final AngularVelocity actuatorVelocity = RotationsPerSecond.of(5);
    public static final AngularAcceleration actuatorAcceleration =
        RotationsPerSecondPerSecond.of(5);

    public static final double feedGearRatio = 32 / 18.0;
    public static final double actuatorGearRatio = 50;

    public static final Distance intakeLength = Inches.of(15);

    public static final Angle actuatorStowed = Radians.of(2.33874);
    public static final Angle actuatorOut = Radians.of(-0.736);

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
      HOME(Radians.of(0), Radians.of(0)),
      HUMAN(Radians.of(0.476), Radians.of(18.27)),
      PROCESSOR(Radians.of(-0.1), Radians.of(5)),

      L1(Radians.of(0.436), Radians.of(6)),
      L2(Radians.of(-0.3), Radians.of(19)),
      L3(Radians.of(-0.3), Radians.of(28)),
      L4(Radians.of(-0.1), Radians.of(5)),

      LOWER_ALGAE(Radians.of(0.051), Radians.of(19)),
      UPPER_ALGAE(Radians.of(0.023), Radians.of(30));

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
      I1(Radians.of(0), Radians.of(3));

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
      setpointMap.put(Pair.of(HOME, L1), I1);
      setpointMap.put(Pair.of(HOME, L4), I1);
    }

    public static final AngularVelocity maxWristSpeed = RotationsPerSecond.of(4);
    public static final AngularVelocity maxElevatorSpeed = RotationsPerSecond.of(12);

    public static final AngularVelocity manualWristSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity manualElevatorSpeed = RotationsPerSecond.of(3);

    public static final AngularAcceleration maxWristAcceleration =
        RotationsPerSecondPerSecond.of(6);
    public static final AngularAcceleration maxElevatorAcceleration =
        RotationsPerSecondPerSecond.of(8);

    public static final int homeSwitch = 6;

    public static final int leftMotorId = 12;
    public static final int rightMotorId = 9;
    public static final int wristMotorId = 13;

    public static final double elevatorGearRatio = 9;

    public static final Distance drumRadius = Inches.of(1.504 / 2);
    public static final Distance drumCircumference = drumRadius.times(2 * Math.PI);

    public static final Angle minElevatorHeight = Radians.of(0);
    public static final Angle maxElevatorHeight = Radians.of(35);

    public static final Distance manipulatorLength = Meters.of(0.18415);

    public static final Angle minWristAngle = Radians.of(-0.32);
    public static final Angle maxWristAngle = Radians.of(0.67);

    public static final double wristGearRatio = 33.75;

    public static final Voltage elevatorkS = Volts.of(0.023571);
    public static final Voltage elevatorkG = Volts.of(0.29129);
    public static final Per<VoltageUnit, AngularVelocityUnit> elevatorkV =
        Volts.per(RotationsPerSecond).ofNative(1.1142);
    public static final Per<VoltageUnit, AngularAccelerationUnit> elevatorkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.0275);

    public static final Per<VoltageUnit, AngleUnit> elevatorkP =
        Volts.per(Rotations).ofNative(12.705); // 15 from sysid

    public static final Voltage wristkS = Volts.of(0.1);
    public static final Voltage wristkG = Volts.of(0.21);
    public static final Per<VoltageUnit, AngularVelocityUnit> wristkV =
        Volts.per(RotationsPerSecond).ofNative(4.05);
    public static final Per<VoltageUnit, AngularAccelerationUnit> wristkA =
        Volts.per(RotationsPerSecondPerSecond).ofNative(0.03);

    public static final Per<VoltageUnit, AngleUnit> wristkP =
        Volts.per(Rotations).ofNative(13.082); // 17.221 from sysid
  }

  public static class SerializerConstants {
    public static final int frontBeamPort = 9;
    public static final int backBeamPort = 2;

    public static final Voltage feedkS = Volts.of(0.28915);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.28474);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.41102);

    public static final double feedGearRatio = 70.0 / 30;

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(20);

    public static final int feedMotorId = 10;
  }

  public static class ManipulatorConstants {
    public static final int leftMotorId = 11;
    public static final int rightMotorId = 14;

    public static final int coralBeam = 8;
    public static final int algaeBeam = 4;

    public static final AngularVelocity fastFeedSpeed = RadiansPerSecond.of(50);
    public static final AngularVelocity slowFeedSpeed = RadiansPerSecond.of(16);

    public static final AngularVelocity passoffSpeed = RadiansPerSecond.of(10);

    public static final Voltage leftFlywheelkS = Volts.of(0.2593);
    public static final Per<VoltageUnit, AngularVelocityUnit> leftFlywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.14628);
    public static final Per<VoltageUnit, AngularVelocityUnit> leftFlywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.16712);

    public static final Voltage rightFlywheelkS = Volts.of(0.13616);
    public static final Per<VoltageUnit, AngularVelocityUnit> rightFlywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.13792);
    public static final Per<VoltageUnit, AngularVelocityUnit> rightFlywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.02354);

    public static final double flywheelGearRatio = 36.0 / 30;

    public static final double holdAlgaeVoltage = 0.6;
  }
}
