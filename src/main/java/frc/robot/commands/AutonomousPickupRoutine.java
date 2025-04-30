// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class AutonomousPickupRoutine extends SequentialCommandGroup {
  /** Creates a new AutonomousPickupRoutine. */
  public AutonomousPickupRoutine(CoolArm arm,SignalLights lights, SwerveSubsystem drive, boolean isSneaking) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands()

    super(
      
  
        new InstantCommand(()-> arm.SetArmAction(ArmAction.Travel),arm),
        new ActiveDriveToPose(drive, lights, true, isSneaking ? GoalType.Coral_Station_SNEAK : GoalType.Coral_Station_Normal),
        new WaitCommand(0.25),
        
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new ParallelRaceGroup(new WaitForCoralInPickupBin(arm),new WaitCommand(0.5)),
            
            new InstantCommand(()-> arm.SetArmAction(ArmAction.Pickup),arm),
            new WaitCommand(0.25)
            ),
          
            ( isSneaking ? new RunCommand(() -> drive.drive(new ChassisSpeeds(2,-1,0))) : new ActiveDriveToPose(drive, lights, true, GoalType.Algae_Removal) )
        ),

        new InstantCommand(()->arm.SetArmAction(ArmAction.L4))
    );
        


    
    // addCommands(new FooCommand(), new BarCommand());
  }
}
