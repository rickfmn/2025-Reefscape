// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
public class CenterAlgaeAuto extends SequentialCommandGroup {
  /** Creates a new AutoAutoCommand. */
  public CenterAlgaeAuto(SwerveSubsystem swerve,SignalLights lights,CoolArm arm,boolean doAlgaeRemoval) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command armL41 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L4), arm);
    Command autoAlign1L = new ActiveDriveToPose(swerve, lights, true, GoalType.Reef_Right);
    Command place1 = new AutoPlace(arm, swerve,doAlgaeRemoval);

    Command autoAlignAlgaeRemoval1 = new ActiveDriveToPose(swerve, lights, true, GoalType.Algae_Removal);
    Command armL3Algae1 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L3), arm);
    Command algaeRemovePlace1 = new AutoPlace(arm, swerve,false);

    Command autoAlignAlgaeRemoval2 = new ActiveDriveToPose(swerve, lights, true, GoalType.Algae_Removal);
    Command armL4Algae2 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L4), arm);
    Command algaeRemovePlace2 = new AutoPlace(arm, swerve,false);

    Command autoAlignAlgaeRemoval3 = new ActiveDriveToPose(swerve, lights, true, GoalType.Algae_Removal);
    Command armL4Algae3 = new InstantCommand(()-> arm.SetArmAction(ArmAction.L4), arm);
    Command algaeRemovePlace3 = new AutoPlace(arm, swerve,false);


    Command driveToLeftSide = new ParallelDeadlineGroup(new WaitCommand(2),new RunCommand(() -> swerve.setChassisSpeeds(new ChassisSpeeds(-0.25, 2, -0.2)), swerve))
    .andThen(new InstantCommand(()-> swerve.setChassisSpeeds(new ChassisSpeeds(0,0,0))));

    Command driveToRightSide = new ParallelDeadlineGroup(new WaitCommand(6),new RunCommand(() -> swerve.setChassisSpeeds(new ChassisSpeeds(-0.25, -1, 0.3)), swerve))
    .andThen(new InstantCommand(()-> swerve.setChassisSpeeds(new ChassisSpeeds(0,0,0))));


    
    

    if(doAlgaeRemoval){
      addCommands(
      new SequentialCommandGroup(armL41,autoAlign1L),
      place1,

      armL3Algae1,
      autoAlignAlgaeRemoval1,
      algaeRemovePlace1,

      driveToLeftSide,

      armL4Algae2,
      autoAlignAlgaeRemoval2,
      algaeRemovePlace2,

      driveToRightSide,

      autoAlignAlgaeRemoval3,
      armL4Algae3,
      algaeRemovePlace3

    );
    }
    else{
      addCommands(
        new SequentialCommandGroup(armL41,autoAlign1L),
        place1
    );
    }

    
  }
}
