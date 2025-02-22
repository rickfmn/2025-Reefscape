// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoPickup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoolArm;
import frc.robot.subsystems.SignalLights;
import frc.robot.subsystems.CoolArm.ArmAction;
import frc.robot.subsystems.SignalLights.LightSignal;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandJoystick driverJoystick = new CommandJoystick(0);

  final         CommandJoystick copilotBoxController = new CommandJoystick(1);

  
  final         CommandJoystick copilotSNESController = new CommandJoystick(2);

  private final SignalLights signalLights = new SignalLights();


  
  private final CoolArm coolArm = new CoolArm(signalLights);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/Abyss"),coolArm);

  private final SendableChooser<Command> autoSelector;
  // private final Vision vision = drivebase.vision;

  private final Climber climber = new Climber(signalLights);
  private final AlgaeIntake algaeIntake = new AlgaeIntake(signalLights);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings

  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
  //                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
  //                                                                                              OperatorConstants.DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
  //                                                                                              OperatorConstants.RIGHT_X_DEADBAND),
  //                                                                driverXbox.getHID()::getYButtonPressed,
  //                                                                driverXbox.getHID()::getAButtonPressed,
  //                                                                driverXbox.getHID()::getXButtonPressed,
  //                                                                driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getY() * -1,
                                                                () -> driverJoystick.getX() * -1)
                                                            .withControllerRotationAxis(() -> driverJoystick.getTwist() * -0.5)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityPrecise = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverJoystick.getY() * -0.25,
                                                            () -> driverJoystick.getX() * -0.25)
                                                        .withControllerRotationAxis(() -> driverJoystick.getTwist() * -0.5)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
  //                                                                                            driverXbox::getRightY)
  //                                                          .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  //Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  
  Command driveFieldOrientedAnglularVelocityPrecise = drivebase.driveFieldOriented(driveAngularVelocityPrecise);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverJoystick.getY(),
                                                                   () -> -driverJoystick.getX())
                                                               .withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverJoystick.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverJoystick.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    configureAutoNamedCommands();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    
    // Shuffleboard.getTab("Debug 1").addBoolean("Button 3", driverJoystick.button(3));
    
    // Shuffleboard.getTab("Debug 1").addBoolean("Button 4", driverJoystick.button(4));
    // Shuffleboard.getTab("testing").addDouble("Distance to Tag 16", vision::getLatestTag16Distance);
    // driverJoystick.button(14).onTrue(new InstantCommand( () -> this.getTag16Distance()  )) ;
    //driverJoystick.button(14).onTrue(new InstantCommand( () -> drivebase.()  )) ;
    //Shuffleboard.getTab("Tab 7").addDouble("Angle to Reef",()-> drivebase.getBestReefTargetByPose());
    
    autoSelector = AutoBuilder.buildAutoChooser();


    Command simpleDriveForward1 = new ParallelDeadlineGroup(new WaitCommand(6),new RunCommand(() -> drivebase.setChassisSpeeds(new ChassisSpeeds(0.5, 0, 0)), drivebase))
    .andThen(new InstantCommand(()-> drivebase.setChassisSpeeds(new ChassisSpeeds(0,0,0))));

    Command simpleDriveForward2 = new ParallelDeadlineGroup(new WaitCommand(6),new RunCommand(() -> drivebase.setChassisSpeeds(new ChassisSpeeds(0.5, 0, 0)), drivebase))
    .andThen(new InstantCommand(()-> drivebase.setChassisSpeeds(new ChassisSpeeds(0,0,0))));

    Command simpleDriveReverse = new ParallelDeadlineGroup(new WaitCommand(3),new RunCommand(() -> drivebase.setChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)), drivebase))
    .andThen(new InstantCommand(()-> drivebase.setChassisSpeeds(new ChassisSpeeds(0,0,0))));

    autoSelector.addOption("Simple Forward", simpleDriveForward1);
    
    autoSelector.addOption("Simple L1", new ParallelCommandGroup(simpleDriveForward2,new InstantCommand(()->coolArm.SetArmAction(ArmAction.L1), coolArm))
    .andThen(new ParallelCommandGroup(simpleDriveReverse,new InstantCommand(()->coolArm.SetArmAction(ArmAction.L2), coolArm))));

    // autoSelector.addOption("Simple L4", new ParallelCommandGroup(simpleDriveForward,new InstantCommand(()->coolArm.SetArmAction(ArmAction.L4), coolArm))
    // .andThen(new ParallelCommandGroup(simpleDriveReverse,new InstantCommand(()->coolArm.SetArmAction(ArmAction.Place), coolArm))));


    Shuffleboard.getTab("Game HUD").add(autoSelector).withSize(2,1);
  }


  public void configureAutoNamedCommands(){
    NamedCommands.registerCommand("Arm L1", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L1)));
    NamedCommands.registerCommand("Arm L2", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L2)));
    NamedCommands.registerCommand("Arm L3", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L3)));
    NamedCommands.registerCommand("Arm L4", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L4)));

    NamedCommands.registerCommand("Place", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Place)));
    NamedCommands.registerCommand("Pickup Coral", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Pickup)));
    NamedCommands.registerCommand("AutoPickup", new AutoPickup(coolArm));
    
    NamedCommands.registerCommand("Travel Setpoint", new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Place)));


  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverJoystick.button(13).onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverJoystick.button(4).whileTrue(new StartEndCommand(() -> driveToBestTarget(true), ()-> System.out.println("Lined Up Right"),drivebase));
    driverJoystick.button(3).whileTrue(new StartEndCommand(() -> driveToBestTarget(false), () -> System.out.println("Lined UP Left?"),drivebase));
    driverJoystick.button(7).whileTrue(new RunCommand(() -> drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 12)), drivebase));
    // Command autoAim = drivebase.aimAtSpeaker(5);
    // autoAim.addRequirements(drivebase);
    // driverJoystick.button(3).whileTrue(autoAim);

    driverJoystick.button(5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L1)));
    driverJoystick.button(10).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L2)));
    driverJoystick.button(6).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L3)));
    driverJoystick.button(9).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L4)));

    driverJoystick.trigger().whileTrue(driveFieldOrientedAnglularVelocityPrecise);

    driverJoystick.povUp().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Place)));
    
    driverJoystick.povDown().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Pickup)));
    driverJoystick.button(2).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Travel)));
    

    // driverJoystick.button(5).whileTrue(new StartEndCommand(() -> coolArm.SetElevatorMotor(0.1), () -> coolArm.SetElevatorMotor(0), coolArm));
    // driverJoystick.button(8).whileTrue(new StartEndCommand(() -> coolArm.SetElevatorMotor(-0.1), () -> coolArm.SetElevatorMotor(0), coolArm));

    // driverJoystick.button(15).whileTrue(coolArm.sysIdRoutine.dynamic(Direction.kForward));
    // driverJoystick.button(12).whileTrue(coolArm.sysIdRoutine.quasistatic(Direction.kForward));

    // driverJoystick.button(16).whileTrue(coolArm.sysIdRoutine.dynamic(Direction.kReverse));
    // driverJoystick.button(11).whileTrue(coolArm.sysIdRoutine.quasistatic(Direction.kReverse));

    //coolArm.setDefaultCommand(new RunCommand(() -> coolArm.SetAngleSetpoint(driverJoystick.getThrottle() *-90 + 180),coolArm));

    copilotBoxController.button(5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Place)));
    copilotBoxController.button(7).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Travel)));
    // copilotController.povRight().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L2)));
    // copilotController.povDown().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L3)));
    // copilotController.povLeft().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L4)));

    // copilotController.povUp().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Travel)));
    copilotBoxController.button(8).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Pickup)));

    copilotBoxController.axisGreaterThan(3, 0.5).whileTrue(new StartEndCommand(()->coolArm.SetElevatorMotorManual(2), ()->coolArm.SetElevatorMotor(0),coolArm));
    copilotBoxController.axisGreaterThan(2, 0.5).whileTrue(new StartEndCommand(()->coolArm.SetElevatorMotorManual(-2),()->coolArm.SetElevatorMotor(0),coolArm));

    copilotBoxController.button(9).whileTrue(new RunCommand(() -> coolArm.ManualAngleControl(copilotBoxController), coolArm));
   
    copilotBoxController.button(4).whileTrue(new StartEndCommand(() ->  algaeIntake.Intake(), () -> algaeIntake.StopIntake(), algaeIntake));
    copilotBoxController.button(2).whileTrue(new StartEndCommand(() ->  algaeIntake.Outtake(), () -> algaeIntake.StopIntake(), algaeIntake));

    copilotBoxController.button(6).onTrue(new InstantCommand(() -> climber.Prepare(), climber));
    //copilotController.button(6).onTrue(new PrintCommand("Should be preparing"));
    copilotBoxController.button(10).onTrue(new InstantCommand(() -> climber.Climb(), climber));
    copilotBoxController.button(3).onTrue(new InstantCommand(() -> climber.Best(), climber));
    copilotBoxController.button(1).onTrue(new InstantCommand(()->coolArm.DoAction(copilotBoxController)));




    copilotSNESController.button(1).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Place)));
    copilotSNESController.button(2).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Travel)));
    // copilotController.povRight().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L2)));
    // copilotController.povDown().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L3)));
    // copilotController.povLeft().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L4)));

    // copilotController.povUp().onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Travel)));
    copilotSNESController.button(3).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.Pickup)));
   
    copilotSNESController.button(5).whileTrue(new StartEndCommand(() ->  algaeIntake.Intake(), () -> algaeIntake.StopIntake(), algaeIntake));
    copilotSNESController.button(6).whileTrue(new StartEndCommand(() ->  algaeIntake.Outtake(), () -> algaeIntake.StopIntake(), algaeIntake));

    copilotSNESController.button(10).onTrue(new InstantCommand(() -> climber.Prepare(), climber));
    copilotSNESController.button(9).onTrue(new InstantCommand(() -> climber.Climb(), climber));
    copilotSNESController.button(4).onTrue(new InstantCommand(() -> climber.Best(), climber));


    copilotSNESController.axisGreaterThan(0, 0.5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L2)));
    copilotSNESController.axisLessThan(0, -0.5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L3)));
    copilotSNESController.axisGreaterThan(4, 0.5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L1)));
    copilotSNESController.axisLessThan(4, -0.5).onTrue(new InstantCommand(()->coolArm.SetArmAction(CoolArm.ArmAction.L4)));



  }

  public void driveToBestTarget(boolean isRight){
    //int targetID = Vision.Cameras.APRIL_CAM.getLatestBestFiducialIDSeen();
    


    int button = 3;
    if(isRight) button = 4;
    final int buttonFinal = button;
    
    int tagLRIndex = 0;
    if(!isRight) tagLRIndex = 1;

    Pose2d goalPose = drivebase.getBestReefTargetByPose(tagLRIndex);

    if(goalPose == null){
      System.out.println("Oh no, It looks like you didn't see anything");
      return;
    }

    Command pathfindCommand = drivebase.createTrajectoryToPose(goalPose)
    .onlyWhile(driverJoystick.button(buttonFinal));
    pathfindCommand.addRequirements(drivebase);
    pathfindCommand.schedule();

    //System.out.println("Has the pathfinding command finished: " + pathfindCommand.isFinished());
  }

  public void SetUpAutoSelector(){
    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // updateDriveEncoders();
    
    // An example command will be run in autonomous
    return autoSelector.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void disabledPeriodic(){
    coolArm.SetAngleSetpoint(coolArm.absAngleEncoder.getPosition());
    coolArm.SetElevatorSetpoint(coolArm.elevatorEncoder.getPosition());
    coolArm.SetElevatorControlEnabled(false);
    signalLights.periodic();
    //signalLights.SetSignal(LightSignal.databitsAnimated);
  }

  public void disabledInit(){
    signalLights.SetSignal(LightSignal.Idle);
  }



public void updateDriveEncoders() {
    drivebase.resetDriveEncoders();
    drivebase.synchronizeModuleEncoders();
}


public void teleopInit() {
  signalLights.DisablePartyMode();
}
}
