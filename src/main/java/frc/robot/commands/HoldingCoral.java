// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoolArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldingCoral extends Command {
  private CoolArm coolArm;
  private Timer debounceTimer;
  /** Creates a new HoldingCoral. */
  public HoldingCoral(CoolArm arm) {
    coolArm = arm;
    debounceTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debounceTimer.stop();
    debounceTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(coolArm.HasCoralInGripper() && !debounceTimer.isRunning()){
      debounceTimer.restart();
    }
    else if(!coolArm.HasCoralInGripper()){
      debounceTimer.stop();
      debounceTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debounceTimer.hasElapsed(0.125);
  }
}
