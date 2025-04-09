// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActiveDriveToPose extends Command {


  public enum GoalType {
    Reef_Right,
    Reef_Left,
    Coral_Station,
    Algae_Removal
  }

  private GoalType goalType = GoalType.Algae_Removal;

  private SwerveSubsystem drivetrain;
  private SignalLights signalLights;
  private Pose2d goalPose2d = Pose2d.kZero;
  private boolean isRight = true;
  private Transform2d poseError = Transform2d.kZero;

  private Timer loopTimer = new Timer();
  private boolean inAuto = true;
  private boolean atTolerance = false;
  private Timer timeAtTolerance = new Timer();

  private PIDController positionController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionState = new State(0, 0);
  private TrapezoidProfile positionTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);

  private PIDController positionXController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionXState = new State(0, 0);
  private TrapezoidProfile positionXTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);

  private PIDController positionYController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionYState = new State(0, 0);
  private TrapezoidProfile positionYTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);
  
  private PIDController rotationController = new PIDController(AutonConstants.rotationKP, AutonConstants.rotationKI, AutonConstants.rotationKD);
  private TrapezoidProfile.State previousRotationState = new State(0, 0);
  private TrapezoidProfile positionRotationProfile = new TrapezoidProfile(AutonConstants.rotationPIDConstraints);
  //TODO: add a trapezoid profile to the rotation

  /** Creates a new ActiveDriveToGoalPose. */
  public ActiveDriveToPose(SwerveSubsystem swerveSubsystem,SignalLights lights,boolean inAutonomous, GoalType goal) {
    drivetrain = swerveSubsystem;
    signalLights = lights;

    goalType = goal;

    isRight = false;
    if(goalType == GoalType.Reef_Right){
      isRight = true;
    }

    inAuto = inAutonomous;
    

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    
    if( !(inAutonomous && (goalType == GoalType.Coral_Station)) ){
      addRequirements(swerveSubsystem);
    }
    
    
    
    
    //SmartDashboard.putData(positionController);
    
    SmartDashboard.putData(rotationController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(goalType == GoalType.Reef_Left || goalType == GoalType.Reef_Right){
      goalPose2d = drivetrain.getBestReefTargetByPose(isRight ? 1: 0);
    }
    else if (goalType == GoalType.Coral_Station){
      goalPose2d = drivetrain.getBestCoralStationByPose(0);
    }
    else{
      goalPose2d = drivetrain.getBestAlgaeRemovalTargetByPose();
    }

    atTolerance = false;
    
    poseError = drivetrain.getPose().minus(goalPose2d);
    drivetrain.goalPose2d = goalPose2d;
    
    ChassisSpeeds currentSpeeds = drivetrain.getRobotVelocity();

    previousPositionXState.position = poseError.getX();
    previousPositionXState.velocity = -currentSpeeds.vxMetersPerSecond;//might need to be negative

    previousPositionYState.position = poseError.getY();
    previousPositionYState.velocity = -currentSpeeds.vyMetersPerSecond;//might need to be negative

    loopTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    poseError = currentPose.minus(goalPose2d);
    Translation2d translationError = poseError.getTranslation();

    ChassisSpeeds currentSpeeds = drivetrain.getRobotVelocity();


    // TrapezoidProfile.State currentPositionXState = new State(translationError.getX(), -currentSpeeds.vxMetersPerSecond);
    
    // TrapezoidProfile.State currentPositionYState = new State(translationError.getY(), -currentSpeeds.vyMetersPerSecond);


    previousPositionXState = positionXTrapezoidProfile.calculate(loopTimer.get(), previousPositionXState, new State(0,0));
    double positionXPIDOutput = positionXController.calculate(translationError.getX(), 0);

    previousPositionYState = positionYTrapezoidProfile.calculate(loopTimer.get(), previousPositionYState, new State(0,0));
    double positionYPIDOutput = positionYController.calculate(translationError.getY(), 0);


    // if(!atTolerance){

    //   if(Math.abs(poseError.getX()) > 0.05){
    //     positionXPIDOutput = Math.max(0.1, Math.abs(positionXPIDOutput)) * Math.signum(positionXPIDOutput);
    //   }

    //   if(Math.abs(poseError.getY()) > 0.05){
    //     positionYPIDOutput = Math.max(0.1, Math.abs(positionYPIDOutput)) * Math.signum(positionYPIDOutput);

    //   }


    // }
    



    double rotationPIDOutput = rotationController.calculate(poseError.getRotation().getRadians(), 0);
    
    ChassisSpeeds rrSpeeds = new ChassisSpeeds(positionXPIDOutput,positionYPIDOutput, rotationPIDOutput);


    drivetrain.setChassisSpeeds(rrSpeeds);
    // drivetrain.drive(translationSpeeds, rotationPIDOutput, false);

    loopTimer.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds(0,0,0));
  }


  public boolean atToleranceFromGoal(){
    double angleError = poseError.getRotation().getDegrees();
    double positionErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    

    if(goalType == GoalType.Coral_Station){
      return (Math.abs(angleError) < 10.0) && positionErrorMagnitude < 0.20;
    
    }
    else{
      return (Math.abs(angleError) < 1.0) && positionErrorMagnitude < 0.035;
    }
    
  }

  public boolean readyToPlace(){
    boolean nowAtTolerance = atToleranceFromGoal();
    if(!atTolerance && nowAtTolerance){
      timeAtTolerance.restart();
    }
    else if (!nowAtTolerance){
      timeAtTolerance.reset();
      timeAtTolerance.stop();
    }
    atTolerance = nowAtTolerance;

    return timeAtTolerance.hasElapsed(0.15);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean aligned = readyToPlace();

    signalLights.autoAligned = aligned;

    if(inAuto){
      if(goalType == GoalType.Coral_Station){
        return atTolerance;
      }
      return aligned;
    }
    else{
      return false;
    }
    
  }
}
