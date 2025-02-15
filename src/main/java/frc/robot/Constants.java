// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */



                    //   I've moved everything that you shouldn't ever really need to adjust far to the side

public final class Constants {
  public static final class DriveConstants {
    
    
    //                #### THESE ARE ONLY THE VALUES YOU NEED TO CHANGE OFTEN ####
    
            // Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather the allowed maximum speed for the robot
    public static double kMaxSpeedMetersPerSecond = 4.8; //max 4.8 with modules used in 2024

            // This refers to the actual top speed of the robot, used by pathplanner to drive the robot (can be lowered if somebody don't trust it ehm Gaskill)
    public static double kTrueMaxSpeedMetersPerSecond = 4.8;
    
            //Sets the high and max turning speed, seperate variable so you don't have to swap two values
    private static double maxTurnSpeed = 1.35 * Math.PI; // Max 2
            //Sets low turning speed
    public static double kLowTurnSpeed = 0.50 * Math.PI;  // Min 0.01
    
      //Used by robot to set max angular speed, adjust maxTurnSpeed variable to change this
    public static double kMaxAngularSpeed = maxTurnSpeed; 
      //Used by drive susbsytem to change to high turning, change max turn speed to adjust this. 
    public static double kHighTurnSpeed = maxTurnSpeed;

    

    public static final double kDirectionSlewRate = 2.4; // 1.2 radians per second
    public static final double kMagnitudeSlewRate = 3.6; // 1.8 percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // 2.0 percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    
 //                #### END OF THE VALUES YOU NEED TO CHANGE OFTEN UNTIL NEAR THE END FOR THE EXAMPLE SUBSYSTEM ####




                                                                                // Distance between front and back wheels on robot
                                                                                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                                                                    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                                                                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                                                                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                                                                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                                                                                // Angular offsets of the modules relative to the chassis in radians
                                                                                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
                                                                                public static final double kFrontRightChassisAngularOffset = 0;
                                                                                public static final double kBackLeftChassisAngularOffset = Math.PI;
                                                                                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 11;

    // LETS THE ROBOT KNOW THE GYRO IS REVERSED
    public static final boolean kGyroReversed = false;


    // LETS PATHPLANNER WORK
        // Drive base radius in meters. Distance from robot center to furthest module.
    public static final double kDriveBaseRadius = 0.584; //Leftover from last year
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                                                                              // Calculations required for driving motor conversion factors and feed forward
                                                                              public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                                                                              public static final double kWheelDiameterMeters = 0.0762;
                                                                              public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                                                                              // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
                                                                              public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                                                                              public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                                                                                  / kDrivingMotorReduction;

                                                                              public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                                                                  / kDrivingMotorReduction; // meters
                                                                              public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                                                                  / kDrivingMotorReduction) / 60.0; // meters per second

                                                                              public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                                                                              public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

                                                                              public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                                                                              public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                                                                              public static final double kDrivingP = 0.04;
                                                                              public static final double kDrivingI = 0;
                                                                              public static final double kDrivingD = 0;
                                                                              public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                                                                              public static final double kDrivingMinOutput = -1;
                                                                              public static final double kDrivingMaxOutput = 1;

                                                                              public static final double kTurningP = 1;
                                                                              public static final double kTurningI = 0;
                                                                              public static final double kTurningD = 0;
                                                                              public static final double kTurningFF = 0;
                                                                              public static final double kTurningMinOutput = -1;
                                                                              public static final double kTurningMaxOutput = 1;

                                                                              public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                                                                              public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 50; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
  }


  //                    #### CHANGE WHICH PORTS THE CONTROLLERS ARE CONNECTED TO AND ADD DEADBANDS HERE ####
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class keybindConstants {
      // Button bindings to adjust the turning rate of the robot (something we control during the match)
    public static final int kTurnSpeedLow = 3;
    public static final int kTurnSpeedHigh = 4;

      // Button bindings for Pre-match and Trouble shooting

        //Resets the gyro
      public static final int kResetGyro = 7;
      
        //Sets wheels straight
      public static final int kWheelStraightAlign = 8;
        
        //wheels in x position, driver top middle button
      public static final int kWheelXAlign = Button.kR1.value;


  }

//     You shouldn't need to touch this
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

//                     #### EXAMPLE SUBYSTEM CONSTANTS ####
  public static final class exampleSubsystemConstants {
    public static final int kShooterBottomID = 31;
    public static final int kShooterTopID = 32;
    public static final int kAmpRPM = 100;
    public static final int kSubwooferRPM = 10000;
    public static final int kPodiumRPM = 15000;
    public static final int kShooterSuck = -100;
  }


 
}
