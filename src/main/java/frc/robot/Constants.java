// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String canivore = "CTRE";

  public static final Frequency simUpdateFrequency = Hertz.of(200);

  public static class Ports {
    public static final int driverController = 0;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  public static class VisionConstants {
    public static final double[] singleTagBaseStdDevs = new double[] {5, 5, 5};
    public static final double[] multiTagBaseStdDevs = new double[] {1, 1, 1};

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.01;
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
        RotationsPerSecondPerSecond.of(5);

    public static final double feedGearRatio = 32 / 18.0;
    public static final double actuatorGearRatio = 50;

    public static final Distance intakeLength = Inches.of(15);

    public static final Angle actuatorStowed = Radians.of(0.92559260823);
    public static final Angle actuatorOut = Radians.of(3.48159274025);

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(55);
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
}
