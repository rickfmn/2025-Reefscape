// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.CoolArm.ArmAction;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAutoCommand extends SequentialCommandGroup {
  /** Creates a new AutoAutoCommand. */
  public AutoAutoCommand(SwerveSubsystem swerve,SignalLights lights,CoolArm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command armL41 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L4), arm);
    Command autoAlign1L = new ActiveDriveToPose(swerve, lights, true, true,true);
    Command place1 = new AutoPlace(arm, swerve);
    Command autoCoralStation1 = new ActiveDriveToPose(swerve, lights, false, true, false);
    Command autoPickup1 = new AutoPickup(arm,swerve);
    Command armL42 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L4), arm);
    Command autoAlign2L = new ActiveDriveToPose(swerve, lights, false, true,true);
    Command place2 = new AutoPlace(arm, swerve);


    addCommands(
      armL41,
      autoAlign1L,
      place1,
      //new ParallelDeadlineGroup(autoPickup1, autoCoralStation1),
      autoCoralStation1,
      autoPickup1,
      armL42,
      autoAlign2L,
      place2
    );
  }
}
