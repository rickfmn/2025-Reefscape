// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ActiveDriveToPose.GoalType;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.CoolArm.ArmAction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousScoreRoutine extends ParallelDeadlineGroup {
  /** Creates a new AutonomousScoreRoutine. */
  public AutonomousScoreRoutine(CoolArm arm,SwerveSubsystem drive, SignalLights lights, GoalType leftOrRight) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new HoldingCoral(arm),
    new SequentialCommandGroup(
      new InstantCommand(() -> arm.SetArmAction(ArmAction.L4)),
      new ActiveDriveToPose(drive, lights, true, leftOrRight),
      new AutoPlace(arm, drive, false),
      
      new WaitCommand(0.25),
      new InstantCommand(() -> arm.SetArmAction(ArmAction.L4)),
      new ActiveDriveToPose(drive, lights, true, leftOrRight),
      new AutoPlace(arm, drive, false)

      
      
    )
    );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
