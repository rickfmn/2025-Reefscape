// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.channels.Pipe;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ActiveDriveToPose.GoalType;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.CoolArm.ArmAction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPickup extends Command {
  private CoolArm coolArm;

  private Timer pickupTimer = new Timer();
  private Timer coralSensorDebounceTimer = new Timer();
  private Timer routineTimeoutTimer = new Timer();

  private boolean hasCoral = false;
  private boolean hadCoralInPickupBin = false;

  /** Creates a new AutoCoralStationRoutine. */
  public AutoPickup(CoolArm arm) {

    coolArm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coolArm.SetArmAction(ArmAction.Travel);
    pickupTimer.reset();
    pickupTimer.stop();
    routineTimeoutTimer.restart();
    hasCoral = false;
    hadCoralInPickupBin = false;
    
    //System.out.println("Do I have Coral? " + hasCoral);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    hasCoral = coralSensorDebounceTimer.hasElapsed(0.125);
    

    if(hasCoral || routineTimeoutTimer.hasElapsed(6)){
      
      if(!pickupTimer.isRunning())
      {
        coolArm.SetArmAction(ArmAction.Pickup);
      }




      pickupTimer.restart();

      
    }

    if(coolArm.HasCoralInPickupBin()){
      if(!hadCoralInPickupBin){
        coralSensorDebounceTimer.restart();
      }
    }
    else{
      coralSensorDebounceTimer.reset();
      coralSensorDebounceTimer.stop();
    }
    hadCoralInPickupBin = coolArm.HasCoralInPickupBin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("done already" + interrupted);
    System.out.println("Command was interrupted : " + routineTimeoutTimer.hasElapsed(8));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!coolArm.HasCoralInPickupBin() && pickupTimer.hasElapsed(0.5)) || pickupTimer.hasElapsed(1) || routineTimeoutTimer.hasElapsed(8);
  }
}
