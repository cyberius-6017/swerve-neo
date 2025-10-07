// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class OperatorConstants {

    public static final int driverDriveTrainPort = 0;
    // public static final int driverMechanismsPort = 1;

  }


  public static final class Swerve {

    public static final double stickDeadband = 0.1;
    public static final boolean invNavX = false; 

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final double wheelBase = Units.inchesToMeters(26);
    public static final double baseRadius = Units.inchesToMeters(18.4);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    //public static final double openLoopRamp = 0.25;
    //public static final double closedLoopRamp = 0.0;

  
    public static final double driveGearRatio = (153.0 / 25.0); // 8.14:1
    public static final double angleGearRatio = (150.0 / 7.0); // 21.42:1

    public static final double drivePPkP = 2.5;
    public static final double drivePPkI = 0.0;
    public static final double drivePPkD = 0.001;
    
    public static final double steerPPkP = 2.5;
    public static final double steerPPkI = 0.0;
    public static final double steerPPkD = 0.001;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 5.5; // meters per second
    public static final double maxAngularVelocity = 12.5;


    // public static final SwerveDriveKinematics swerveKinematics =
    //     new SwerveDriveKinematics(
    //         new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    //         new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    //         new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
    //         new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));
    
    public static final SwerveDriveKinematics swerveOdoKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 40;
    public static final int driveContinuousCurrentLimit = 80;

    /* Navigate */ 

    public static final double alignKP = 1.0;
    public static final double alignKI = 0.0;
    public static final double alignKD = 0.0;
    public static final double alignFF = 0.0;

    /* Steer Motor PID Values */
    public static final double angleKP = 0.02;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.03;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.0;
    public static final double driveKV = 0.0;
    public static final double driveKA = 0.0;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean CANcoderInv = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 20;
      public static final Rotation2d encoderOffset = Rotation2d.fromDegrees(33.2);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, encoderOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;  
      public static final int canCoderID = 30;
      public static final Rotation2d encoderOffset = Rotation2d.fromDegrees(267.7);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, encoderOffset);
    }

    /* Back Left Module - Module 2 */ 
    public static final class Mod2 {
      public static final int driveMotorID = 51;
      public static final int angleMotorID = 52;
      public static final int canCoderID = 50;
      public static final Rotation2d encoderOffset = Rotation2d.fromDegrees(180);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, encoderOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 40;
      public static final Rotation2d encoderOffset = Rotation2d.fromDegrees(180.0);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, encoderOffset);
    }
  }
}