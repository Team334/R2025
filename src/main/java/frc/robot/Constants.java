// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.generated.TunerConstants;
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

  public static final String canivore = "Drivetrain";

  public static class Ports {
    public static final int driverController = 0;
    public static final int operatorController = 1;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public static class VisionConstants {
    public static final String blueArducamName = "blue-arducam";

    public static final double[] singleTagBaseStdDevs = new double[] {5, 5, 5};
    public static final double[] multiTagBaseStdDevs = new double[] {1, 1, 1};

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.01;

    public static final VisionPoseEstimatorConstants blueArducam =
        new VisionPoseEstimatorConstants(
            blueArducamName,
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()),
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
  }

  public static class IntakeConstants {
    public static final int feedMotorId = 8;
    public static final int actuatorMotorId = 9;

    // kv is the voltage necessary to spin the pivot 1 rad/s
    // ka is the voltage necessary to accel the pivot 1 rad/s^2
    public static final Per<VoltageUnit, AngularVelocityUnit> actuatorkV =
        VoltsPerRadianPerSecond.ofNative(0.9497114636959001);
    public static final Per<VoltageUnit, AngularAccelerationUnit> actuatorkA =
        VoltsPerRadianPerSecondSquared.ofNative(0.1);

    public static final AngularVelocity actuatorVelocity = RadiansPerSecond.of(12.63541660674576);
    public static final AngularAcceleration actuatorAcceleration = RadiansPerSecondPerSecond.of(24);

    public static final double feedGearRatio = 1;
    public static final double actuatorGearRatio = 50;

    public static final Distance intakeLength = Inches.of(15);

    public static final Angle maxAngle = Radians.of(Math.PI);
    public static final Angle minAngle = Radians.of(Math.PI / 2);

    public static final Angle actuatorStowed = Radians.of(Math.PI / 2);
    public static final Angle actuatorOut = Radians.of(Math.PI);

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(1);
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
      HUMAN(Radians.of(1), Radians.of(20)),
      PROCESSOR(Radians.of(-1), Radians.of(30)),

      L1(Radians.of(-0.5), Radians.of(5)),
      L2(Radians.of(-0.7), Radians.of(20)),
      L3(Radians.of(-0.7), Radians.of(40)),
      L4(Radians.of(-1), Radians.of(90)),

      LOWER_ALGAE(Radians.of(-1), Radians.of(50)),
      UPPER_ALGAE(Radians.of(-1), Radians.of(60));

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
      I1(Radians.of(0), Radians.of(50)),
      I2(Radians.of(-1), Radians.of(30)),
      I3(Radians.of(1), Radians.of(50));

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

    public static final HashMap<Pair<Preset, Preset>, Preset> setpointMap = new HashMap<>();

    static {
      // TODO: actually find setpoint map and put it here
    }

    public static final AngularVelocity maxWristSpeed = RadiansPerSecond.of(14.039351785273068);
    public static final AngularVelocity maxElevatorSpeed = RadiansPerSecond.of(70.19675892636535);

    public static final AngularAcceleration maxWristAcceleration = RadiansPerSecondPerSecond.of(20);
    public static final AngularAcceleration maxElevatorAcceleration =
        RadiansPerSecondPerSecond.of(90);

    public static final int homeSwitch = 0;

    public static final int leftMotorId = 10;
    public static final int rightMotorId = 11;
    public static final int wristMotorId = 12;

    public static final double elevatorGearRatio = 9;

    public static final Distance drumRadius = Inches.of(1.504 / 2);
    public static final Distance drumCircumference = drumRadius.times(2 * Math.PI);

    public static final Angle minElevatorHeight = Radians.of(0);
    public static final Angle maxElevatorHeight = Radians.of(100);

    public static final Distance manipulatorLength = Meters.of(0.18415);

    public static final Angle minWristAngle = Radians.of(-Math.PI / 2);
    public static final Angle maxWristAngle = Radians.of(Math.PI / 2);

    public static final double wristGearRatio = 45;

    // elevator feedforward for the DRUM
    // kv is the voltage necessary to spin the drum 1 rad/s
    // ka is the voltage necessary to accel the drum 1 rad/s^2 (lower since there are 2 motors, the
    // torque is doubled at a voltage)
    public static final Per<VoltageUnit, AngularVelocityUnit> elevatorkV =
        VoltsPerRadianPerSecond.ofNative(0.18);
    public static final Per<VoltageUnit, AngularAccelerationUnit> elevatorkA =
        VoltsPerRadianPerSecondSquared.ofNative(0);

    // wrist feedforward for the pivot (after the gear ratio)
    // kv is the voltage necessary to spin the pivot 1 rad/s
    // ka is the voltage necessary to accel the pivot 1 rad/s^2
    public static final Per<VoltageUnit, AngularVelocityUnit> wristkV =
        VoltsPerRadianPerSecond.ofNative(0.8);
    public static final Per<VoltageUnit, AngularAccelerationUnit> wristkA =
        VoltsPerRadianPerSecondSquared.ofNative(0);
  }

  public static class SerializerConstants {
    public static final int frontBeamPort = 1;
    public static final int backBeamPort = 2;

    public static final int feedMotorId = 13;
  }

  public static class ManipulatorConstants {
    public static final int leftMotorId = 14;
    public static final int rightMotorId = 15;

    public static final int beamPort = 3;
    public static final int switchPort = 4;

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(1);

    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkV =
        VoltsPerRadianPerSecond.ofNative(1);
    public static final double flywheelGearRatio = 1;

    public static final double holdAlgaeVoltage = 0; // need to find
  }
}
