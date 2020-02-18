/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class PixyCam {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double setPoint = 0;
    public static final double setPercentTolerance = 0.01;
  }

  /* CAN IDs */
    public static final int SHOOTER_MOTER_ID = 1;
  
  /* Left Motor */
    public static final int MOTOR_LEFTFRONT = 1;
    public static final int MOTOR_LEFTREAR = 1;
  

    /* Right Motor */
    public static final int MOTOR_RIGHTFRONT = 0;
    public static final int MOTOR_RIGHTREAR = 0;

    /* VICTORSPX */
    public static final int VICTORSPX_LEFT_DRIVE = 1;
    public static final int VICTORSPX_RIGHT_DRIVE = 2;
    public static final int VICTORSPX_WINCH = 3;
    public static final int VICTORSPX_HOOK_LIFT = 4;
    //public static final int VICTORSPX_SHOOT = 3;
    //public static final int VICTORSPX_SHOOT_2 = 4;
    public static final int VICTORSPX_INTAKE = 5;
    public static final int VICTORSPX_INTAKE_2 = 6;

    /* Double Solenoid */
    public static final int DOUBLESOLENOID_INTAKE_1 = 0;
    public static final int DOUBLESOLENOID_INTAKE_2 = 1;

    /* Joystick port */
    public static final int  J_STICK_DRIVER = 0;
    public static final int  J_STICK_CONTROL = 1;

    /* Joystick */
    public static final int BUTTON_RIGHT = 5; 
    public static final int BUTTON_LEFT = 6;
    public static final int BUTTON_X = 3; 
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_A = 1; 
    public static final int BUTTON_B = 2;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_LEFTSTICK = 9;
    public static final int BUTTON_RIGHTSTICK = 10;

    public static final int AXIS_LEFT_X = 0; // NEEDS TO BE 0
    public static final int AXIS_LEFT_Y = 1; //NEEDS TO BE 1
    public static final int AXIS_LEFT_TRIGGER = 2;
    public static final int AXIS_RIGHT_TRIGGER = 3;
    public static final int AXIS_RIGHT_X = 4;
    public static final int AXIS_RIGHT_Y = 5;

    /* Pneumatics */
    public static final int CAN_COMPRESSOR = 0;
    public static final int DOUBLESOLENOID_1 = 1;
    public static final int DOUBLESOLENOID_1_REVERSE = 2;
    public static final int DOUBLESOLENOID_2 = 3;
    public static final int DOUBLESOLENOID_2_REVERSE = 4;

    public static final class ShooterConstants {
      public static boolean kHasIntake = false;
      
      public static int kIntakeMotorPort = 7;
      public static int kConveyorMotor1Port = 7;
      public static int kConveyorMotor2Port = 7;



    //Shooter

    public static final 
    double shoot_speed = 1;
    }
  

    public static final class DriveConstants {
      public static final int kLeftMotor1Port = 0;
      public static final int kLeftMotor2Port = 1;
      public static final int kRightMotor1Port = 2;
      public static final int kRightMotor2Port = 3;
  
      public static final int[] kLeftEncoderPorts = new int[]{0, 1};
      public static final int[] kRightEncoderPorts = new int[]{2, 3};
      public static final boolean kLeftEncoderReversed = false;
      public static final boolean kRightEncoderReversed = true;
  
      public static final double kTrackwidthMeters = 0.69;
      public static final DifferentialDriveKinematics kDriveKinematics =
          new DifferentialDriveKinematics(kTrackwidthMeters);
  
      public static final int kEncoderCPR = 1024;
      public static final double kWheelDiameterMeters = 0.15;
      public static final double kEncoderDistancePerPulse =
          // Assumes the encoders are directly mounted on the wheel shafts
          (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
  
      public static final boolean kGyroReversed = true;
  
      // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
      // These characterization values MUST be determined either experimentally or theoretically
      // for *your* robot's drive.
      // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
      // values for your robot.
      public static final double ksVolts = 0.22;
      public static final double kvVoltSecondsPerMeter = 1.98;
      public static final double kaVoltSecondsSquaredPerMeter = 0.2;
  
      // Example value only - as above, this must be tuned for your drive!
      public static final double kPDriveVel = 8.5;
    }
  
    public static final class OIConstants {
      public static final int kDriverControllerPort = 1;
    }
  
    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
    }
}
