// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final Pose2d REEF_POSE3D_BLUE = new Pose2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.50), Rotation2d.kZero);
  public static final Pose2d REEF_POSE3D_RED = new Pose2d(Units.inchesToMeters(448.65), Units.inchesToMeters(158.50), Rotation2d.kZero);

  public static final int[] REEF_FIDUCIALIDS_BLUE = new int[]{18,19,20,21,22,17};
  public static final int[] REEF_FIDUCIALIDS_RED = new int[]{10,9,8,7,6,11};
  
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class VisionConstants {
    public static final Translation3d KAprilCamFromGyro = new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(-8.25), Units.inchesToMeters(19));
    //public static final Translation3d KAprilCamFromGyro = new Translation3d(0, 0, 0);
    public static final Transform3d reefOffset_Right = new Transform3d(Units.inchesToMeters(29), Units.inchesToMeters(12), Units.inchesToMeters(6),new Rotation3d(0, 0, Math.PI));
    public static final Transform3d reefOffset_Left = new Transform3d(Units.inchesToMeters(29), Units.inchesToMeters(-6), Units.inchesToMeters(6),new Rotation3d(0, 0, Math.PI));

    public static final Pose3d[][] kReefGoalPoses = new Pose3d[22][2];
    
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class AlgaeIntakeConstants
  {

    public static final int ANGLE_MOTOR_ID = 16;
    public static final int INTAKE_MOTOR_ID = 17;
    
    public static final double INTAKE_DEPLOY_ANGLE = 0.85;

    public static final double kINTAKE_SPEED = 3;
    public static final double kHOLD_SPEED = 0.5;
    public static final double kOUTTAKE_SPEED = -1;

    public static final double kDEPLOY_SPEED = 5;
    public static final double kRETRACT_SPEED = -2;
    
  }

  public static class LEDConstants
  {

    public static final int LEFT_LED_COUNT = 49;
    public static final int RIGHT_LED_COUNT = 49;
    public static final int LEFT_LED_PORT = 0;
    public static final int RIGHT_LED_PORT = 1;

    public static final Color kNoAlgaeColor = Color.kWheat;
    //public static final Color8Bit kNoNoteColor = new Color8Bit(Color.kWhite);
    public static final Color kYesAlgaeColor = Color.kGhostWhite;
    public static final Color kAlignColor = Color.kBlack;
    public static final Color kClimbFinishColor = Color.kPurple;
    public static final Color kClimbReadyColor = Color.kPurple;
    public static final Color kDatabitsColor = new Color(2,255,4);
    public static final Color kOffColor = Color.kBlack;
    public static final Color kErrorColor = Color.kHotPink;
    
  }

  public static class ClimberConstants
  {

    public static final int LEADER_MOTOR_ID = 19;
    public static final int FOLLOWER_MOTOR_ID = 18;
    
    //Prep angle
    public static final double CLIMB_PANGLE = 270.0;
    //Stiction angle
    public static final double CLIMB_VANGLE = 232.0;
    //Finish angle
    public static final double CLIMB_FANGLE = 222.0;
    //Best Angle(for the rest of the match)
    public static final double CLIMB_BANGLE = 180.0;



    public static final double kPREPARE_SPEED = 0.1;
    public static final double kCLIMB_SPEED = -0.5;
    public static final double kCLIMB_STICTION_SPEED = -1.0;

    public static final double kLOCKED_SERVO_POS = 1;
    
    public static final double kRELEASED_SERVO_POS = 0;
    public static final int SERVO_ID = 9;
    
  }

  public static class CoolArmConstants
  {

    public static final int angleCANID = 15;
    public static final int elevatorCANID = 14;

    //Gains for the Arm angle controllers, both FF and PID
    public static final double kSAngle = 0;
    public static final double kGAngle = 0.35;//0.085 gains for no coral and set()  // 0.35
    public static final double kVAngle = 0;
    public static final double kPAngle = 0.04;//0.0025 gains for no coral and set() //0.04
    public static final double kIAngle = 0;//0.001 gains for no coral and set()
    public static final double kDAngle = 0;

    //Gains for the Arm elevator controllers, PID
    public static final double kPElevator = 0.5// 1 is better than 0.5
    ; //1 is good, but have to be sad and go slow
    public static final double kIElevator = 0;
    public static final double kDElevator = 0;

    public static final double kMaxElevatorPos = -25.476;
    


    public static final double kL1PrepAngleSP = 147;
    public static final double kL2PrepAngleSP = 202;
    public static final double kL3PrepAngleSP = 229;
    public static final double kL4PrepAngleSP = 229;

    public static final double kTravelAngleSP = 88;
    public static final double kPickupAngleSP = 88;
    //public static final double kPlaceAngleSPChange = -30;
    public static final double kPlaceAngleSP = 165;



    public static final double kL1PrepElevatorSP = -9.28;
    public static final double kL2PrepElevatorSP = -1.07;
    public static final double kL3PrepElevatorSP = -8.37;//-10 at 0.5 p
    public static final double kL4PrepElevatorSP = -24;
    public static final double kTravelElevatorSP = -4.3;
    public static final double kPickupElevatorSP = 0;
    public static final double kPlaceElevatorSPChange = 0;

    
  }
}
