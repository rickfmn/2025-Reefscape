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

  private SwerveSubsystem drivetrain;
  private SignalLights signalLights;
  private Pose2d goalPose2d = Pose2d.kZero;
  private boolean isRight = true;
  private Transform2d poseError = Transform2d.kZero;

  private Timer loopTimer = new Timer();
  private boolean inAuto = true;

  private PIDController positionController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionState = new State(0, 0);
  private TrapezoidProfile positionTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);
  
  private PIDController rotationController = new PIDController(AutonConstants.rotationKP, AutonConstants.rotationKI, AutonConstants.rotationKD);
  private TrapezoidProfile.State previousRotationState = new State(0, 0);
  private TrapezoidProfile positionRotationProfile = new TrapezoidProfile(AutonConstants.rotationPIDConstraints);
  private boolean targetIsReef = false;
  //TODO: add a trapezoid profile to the rotation

  /** Creates a new ActiveDriveToGoalPose. */
  public ActiveDriveToPose(SwerveSubsystem swerveSubsystem,SignalLights lights,boolean rightReef,boolean inAutonomous,boolean reefTarget) {
    drivetrain = swerveSubsystem;
    signalLights = lights;
    isRight = rightReef;

    inAuto = inAutonomous;
    targetIsReef = reefTarget;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    
    if( !(inAutonomous && !reefTarget) ){
      addRequirements(swerveSubsystem);
    }
    
    
    
    
    SmartDashboard.putData(positionController);
    
    SmartDashboard.putData(rotationController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(targetIsReef){
      goalPose2d = drivetrain.getBestReefTargetByPose(isRight ? 1: 0);
    }
    else{
      goalPose2d = drivetrain.getBestCoralStationByPose(0);
    }
    
    poseError = drivetrain.getPose().minus(goalPose2d);
    drivetrain.goalPose2d = goalPose2d;
    
    double translationErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    previousPositionState.position = translationErrorMagnitude;
    previousPositionState.velocity = drivetrain.getSpeedMagnitudeMpS();
    loopTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    poseError = currentPose.minus(goalPose2d);
    Translation2d translationError = poseError.getTranslation();

    Rotation2d angleToGoalPose = translationError.getAngle();
    double translationErrorMagnitude = translationError.getDistance(Translation2d.kZero);

    //drivetrain.setChassisSpeeds(new ChassisSpeeds(-1 * translationError.getX(),-1 * translationError.getY(), -1 * poseError.getRotation().getRadians()));

    TrapezoidProfile.State currentPositionState = new TrapezoidProfile.State(translationErrorMagnitude,drivetrain.getSpeedMagnitudeMpS());

    previousPositionState = positionTrapezoidProfile.calculate(loopTimer.get(), previousPositionState, new State(0,0));

    double positionPIDOutput = positionController.calculate(translationErrorMagnitude, previousPositionState.position);

    // double positionPIDOutput = positionController.calculate(translationErrorMagnitude, 0);

    Translation2d translationSpeeds = new Translation2d(positionPIDOutput, angleToGoalPose);

    double rotationPIDOutput = rotationController.calculate(poseError.getRotation().getRadians(), 0);
    
    ChassisSpeeds rrSpeeds = new ChassisSpeeds(translationSpeeds.getX(),translationSpeeds.getY(), rotationPIDOutput);

    drivetrain.setChassisSpeeds(rrSpeeds);

    loopTimer.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.lock();
  }


  public boolean atToleranceFromGoal(){
    double angleError = poseError.getRotation().getDegrees();
    double positionErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    

    if(targetIsReef){
      return (Math.abs(angleError) < 2.0) && positionErrorMagnitude < 0.02;
    
    }
    else{
      return (Math.abs(angleError) < 5) && positionErrorMagnitude < 0.05;
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean aligned = atToleranceFromGoal();

    signalLights.autoAligned = aligned;

    if(inAuto){
      return aligned;
    }
    else{
      return false;
    }
    
  }
}
