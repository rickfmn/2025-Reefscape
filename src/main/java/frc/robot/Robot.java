// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public static boolean isRedAlliance = false;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    

    for (int i = 17; i < 23; i++){
      VisionConstants.kReefGoalPoses[i][0][3] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L4_Left);
      VisionConstants.kReefGoalPoses[i][1][3] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L4_Right);

      VisionConstants.kReefGoalPoses[i][0][2] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L3_Left);
      VisionConstants.kReefGoalPoses[i][1][2] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L3_Right);

      VisionConstants.kReefGoalPoses[i][0][1] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L2_Left);
      VisionConstants.kReefGoalPoses[i][1][1] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L2_Right);

      VisionConstants.kReefGoalPoses[i][0][0] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L1_Left);
      VisionConstants.kReefGoalPoses[i][1][0] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L1_Right);

      
    }

    for (int i = 6; i < 12; i++){
      VisionConstants.kReefGoalPoses[i][0][3] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L4_Left);
      VisionConstants.kReefGoalPoses[i][1][3] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L4_Right);

      VisionConstants.kReefGoalPoses[i][0][2] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L3_Left);
      VisionConstants.kReefGoalPoses[i][1][2] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L3_Right);

      VisionConstants.kReefGoalPoses[i][0][1] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L2_Left);
      VisionConstants.kReefGoalPoses[i][1][1] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L2_Right);

      VisionConstants.kReefGoalPoses[i][0][0] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L1_Left);
      VisionConstants.kReefGoalPoses[i][1][0] = fieldLayout.getTagPose(i).get().transformBy(VisionConstants.reefOffset_L1_Right);
    }


    VisionConstants.kCoralStationPoses[0][0][0] = fieldLayout.getTagPose(12).get().transformBy(VisionConstants.coralStationOffsetLeft);
    VisionConstants.kCoralStationPoses[0][0][1] = fieldLayout.getTagPose(12).get().transformBy(VisionConstants.coralStationOffsetCenter);
    VisionConstants.kCoralStationPoses[0][0][2] = fieldLayout.getTagPose(12).get().transformBy(VisionConstants.coralStationOffsetRight);

    
    VisionConstants.kCoralStationPoses[0][1][0] = fieldLayout.getTagPose(13).get().transformBy(VisionConstants.coralStationOffsetLeft);
    VisionConstants.kCoralStationPoses[0][1][1] = fieldLayout.getTagPose(13).get().transformBy(VisionConstants.coralStationOffsetCenter);
    VisionConstants.kCoralStationPoses[0][1][2] = fieldLayout.getTagPose(13).get().transformBy(VisionConstants.coralStationOffsetRight);

    
    VisionConstants.kCoralStationPoses[1][0][0] = fieldLayout.getTagPose(1).get().transformBy(VisionConstants.coralStationOffsetLeft);
    VisionConstants.kCoralStationPoses[1][0][1] = fieldLayout.getTagPose(1).get().transformBy(VisionConstants.coralStationOffsetCenter);
    VisionConstants.kCoralStationPoses[1][0][2] = fieldLayout.getTagPose(1).get().transformBy(VisionConstants.coralStationOffsetRight);

    VisionConstants.kCoralStationPoses[1][1][0] = fieldLayout.getTagPose(2).get().transformBy(VisionConstants.coralStationOffsetLeft);
    VisionConstants.kCoralStationPoses[1][1][1] = fieldLayout.getTagPose(2).get().transformBy(VisionConstants.coralStationOffsetCenter);
    VisionConstants.kCoralStationPoses[1][1][2] = fieldLayout.getTagPose(2).get().transformBy(VisionConstants.coralStationOffsetRight);

    


    updateAlliance();
    // VisionConstants.kReefGoalPoses[16][0] = fieldLayout.getTagPose(16).get().transformBy(rightOffset);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();



    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }

    m_robotContainer.disabledPeriodic();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    updateAlliance();
    m_robotContainer.setMotorBrake(true);
    // m_robotContainer.updateDriveEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  public void updateAlliance(){
    isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red; 
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    updateAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setDriveMode();

    m_robotContainer.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
