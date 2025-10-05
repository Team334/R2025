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
  public static final String canivore = "CTRE";

  public static final Frequency simUpdateFrequency = Hertz.of(200);

  public static class Ports {
    public static final int driverController = 0;
  }

  public static enum Piece {
    CORAL,
    ALGAE,
    NONE
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

    public static final Angle actuatorStowed = Radians.of(0.973);
    public static final Angle actuatorOut = Radians.of(3.52);

    public static final AngularVelocity feedSpeed = RadiansPerSecond.of(30); // TODO: increase back
  }

  public static class SerializerConstants {
    public static final int coralBeamPort = 6;

    public static final Voltage feedkS = Volts.of(0.30489);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkV =
        Volts.per(RotationsPerSecond).ofNative(0.28324);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedkP =
        Volts.per(RotationsPerSecond).ofNative(0.41102);

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

    public static final AngularVelocity algaeOuttakeSpeed = RadiansPerSecond.of(-30);
    public static final AngularVelocity algaeIntakeSpeed = RadiansPerSecond.of(50);

    public static final AngularVelocity coralOuttakeSpeed = RadiansPerSecond.of(-40);
    public static final AngularVelocity coralIntakeSpeed = RadiansPerSecond.of(40);

    public static final AngularVelocity humanIntakeSpeed = RadiansPerSecond.of(20);

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

    // right wheel ka = 0.0096618

    public static final double flywheelGearRatio = 3;

    public static final Voltage holdAlgaeVoltage = Volts.of(0.8);
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
      HUMAN(Radians.of(-0.457), Radians.of(16.96)),
      PROCESSOR(Radians.of(-1.06), Radians.of(6.182)),

      L1(Radians.of(-0.233), Radians.of(2.095)),
      L2(Radians.of(-0.793), Radians.of(14.769)),
      L3(Radians.of(-1.06), Radians.of(26.828)),
      L4(Radians.of(1.282), Radians.of(38.2)),

      LOWER_ALGAE(Radians.of(-1.114), Radians.of(19.201)),
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
      I1(Radians.of(-1.06), Radians.of(1.8)),
      I2(Radians.of(-1.06), Radians.of(36.92)),
      I3(Radians.of(-1.06), Radians.of(30)),
      I4(Radians.of(-1.06), Radians.of(10));

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
      // going up to l4
      setpointMap.put(Pair.of(HOME, L4), I2);
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

      // going to a upwards wrist angle from home
      setpointMap.put(Pair.of(HOME, L1), I1);

      // going down to home
      setpointMap.put(Pair.of(I2, HOME), I1);
      setpointMap.put(Pair.of(L3, HOME), I1);
      setpointMap.put(Pair.of(L2, HOME), I1);
      setpointMap.put(Pair.of(L1, HOME), I1);
      setpointMap.put(Pair.of(UPPER_ALGAE, HOME), I1);
      setpointMap.put(Pair.of(LOWER_ALGAE, HOME), I1);
      setpointMap.put(Pair.of(HUMAN, HOME), I1);
      setpointMap.put(Pair.of(PROCESSOR, HOME), I1);
      setpointMap.put(Pair.of(HOME, HOME), I1);

      // going down to processor
      setpointMap.put(Pair.of(UPPER_ALGAE, PROCESSOR), I4);
      setpointMap.put(Pair.of(LOWER_ALGAE, PROCESSOR), I4);
    }

    public static final AngularVelocity maxWristSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity maxElevatorSpeed = RotationsPerSecond.of(17);

    public static final AngularVelocity manualWristSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity manualElevatorSpeed = RotationsPerSecond.of(3);

    public static final AngularAcceleration maxWristAcceleration =
        RotationsPerSecondPerSecond.of(3);
    public static final AngularAcceleration maxElevatorAcceleration =
        RotationsPerSecondPerSecond.of(17);

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
