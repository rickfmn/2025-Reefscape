// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.CoolArm.ArmAction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPlace extends Command {

  private CoolArm coolArm;
  private SwerveSubsystem swerveSubsystem;
  private Timer backupDelayTimer = new Timer();
  private boolean algaeRemoval = false;
  /** Creates a new AutoPlace. */
  public AutoPlace(CoolArm arm, SwerveSubsystem swerve,boolean attemptAlgaeRemoval) {
    coolArm = arm;
    swerveSubsystem = swerve;
    algaeRemoval = attemptAlgaeRemoval;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm,swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coolArm.SetArmAction(ArmAction.Place);
    backupDelayTimer.restart();
    driveReverse();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coolArm.AtElevatorAndArmSetpoints() || (algaeRemoval ? backupDelayTimer.hasElapsed(1.35) : backupDelayTimer.hasElapsed(1));
  }

  public void driveReverse(){
    if(algaeRemoval ? backupDelayTimer.hasElapsed(0.75) : backupDelayTimer.hasElapsed(0.5)){
      if(algaeRemoval){
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(-2,1,0.25));

      }
      else{
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(-2,0,0));

      }

    }
    else{
      swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0,0,0));

    }
  }
}
