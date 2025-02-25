// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.CoolArm.ArmAction;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousPlace extends Command {
  private CoolArm coolArm;
  private Timer placeTimer = new Timer();
  /** Creates a new AutonomousPlace. */
  public AutonomousPlace(CoolArm arm) {
    coolArm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coolArm.SetArmAction(ArmAction.Place);
    placeTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Command travelCommand = new WaitCommand(0.25).andThen(new InstantCommand(() -> coolArm.SetArmAction(ArmAction.Travel)));
    travelCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coolArm.AtElevatorAndArmSetpoints() || placeTimer.hasElapsed(0.5);
  }
}
