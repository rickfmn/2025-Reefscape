// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.CoolArmConstants;
import frc.robot.subsystems.SignalLights.LightSignal;;

public class CoolArm extends SubsystemBase {

  public static enum ArmAction{
    L1,
    L2,
    L3,
    L4,
    Travel,
    Pickup,
    Place
  }

  private SparkMax angleMotor = new SparkMax(CoolArmConstants.angleCANID, MotorType.kBrushless);
  public SparkAbsoluteEncoder absAngleEncoder = angleMotor.getAbsoluteEncoder();
  private ArmFeedforward armFFController = new ArmFeedforward(CoolArmConstants.kSAngle, CoolArmConstants.kGAngle, CoolArmConstants.kVAngle);
  private PIDController armPIDController = new PIDController(CoolArmConstants.kPAngle, CoolArmConstants.kIAngle, CoolArmConstants.kDAngle);
  private TrapezoidProfile.Constraints trapezoidConstraints_Angle = new TrapezoidProfile.Constraints((65d/0.25d), (65d/0.25d)/(0.25d*0.5d ));
  private TrapezoidProfile.State previousTrapezoidState_Angle = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile angleTrapezoidProfile = new TrapezoidProfile(trapezoidConstraints_Angle);
  private TrapezoidProfile.Constraints trapezoidConstraints_Elevator = new TrapezoidProfile.Constraints((25d/0.75d), (25d/0.75d)/(0.25 ));
  private TrapezoidProfile.State previousTrapezoidState_Elevator = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile elevatorTrapezoidProfile = new TrapezoidProfile(trapezoidConstraints_Elevator);
  private Timer trapezoidTimer = new Timer();
  private double angleSetpoint = 5;
  private double elevatorSetpoint = 0;

  private Timer pickupTimoutTimer = new Timer();
  
  private Timer pickupRetryTimer = new Timer();


  private SparkMax elevatorMotor = new SparkMax(CoolArmConstants.elevatorCANID, MotorType.kBrushless);
  public RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private PIDController elevatorPIDController = new PIDController(CoolArmConstants.kPElevator, CoolArmConstants.kIElevator, CoolArmConstants.kDElevator);
  private boolean elevatorControlEnabled = false;
  private SparkLimitSwitch raiseLimitSwitch = elevatorMotor.getReverseLimitSwitch();
  private SparkLimitSwitch lowerLimitSwitch = elevatorMotor.getForwardLimitSwitch();

  private DigitalInput coralPickupSensor = new DigitalInput(CoolArmConstants.kSensorID);

  private int pickupRetryCounter = 0;

  public ArmAction currentAction = ArmAction.L1;

  public SignalLights signalLights;

  // public SysIdRoutine sysIdRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(Volts.of( 0.15 ).per(Units.Seconds), Volts.of(0.7), Seconds.of(10)),
  //   //new SysIdRoutine.Config(),
  //   new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  /** Creates a new CoolArm. */
  public CoolArm(SignalLights lights) {
    // Creates a SysIdRoutine
    signalLights = lights;
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Absolute Angle", absAngleEncoder::getPosition);
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle ProfileGoal", () -> angleSetpoint);
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle Setpoint", () -> previousTrapezoidState_Angle.position);

    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle Motor Current", angleMotor::getOutputCurrent);
    
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle Motor Output", angleMotor::getAppliedOutput);
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Elevator Position", elevatorEncoder::getPosition);
    Shuffleboard.getTab("Arm Sysid Testing").addDouble("Elevator Setpoint", this::GetElevatorSetpoint);

    Shuffleboard.getTab("Debug 1").addBoolean("Coral Sensor", this::HasCoralInPickupBin);
    armPIDController.setIZone(20);
    elevatorPIDController.setIZone(1);
    

    angleSetpoint = absAngleEncoder.getPosition();
    SetElevatorEncoderPosition(0);
    elevatorSetpoint = elevatorEncoder.getPosition();
    angleMotor.getEncoder().setPosition( -1d *  (( absAngleEncoder.getPosition()-90d ) / 360d ) * 42d);


    //Shuffleboard.getTab("Arm Sysid Testing").add(armPIDController);
    SmartDashboard.putData(armPIDController);
    SmartDashboard.putData(elevatorPIDController);
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double absAngle = absAngleEncoder.getPosition();
    //signalLights.ReceiveArmAction(currentAction);
    
    previousTrapezoidState_Elevator = elevatorTrapezoidProfile.calculate(trapezoidTimer.get(), previousTrapezoidState_Elevator, new TrapezoidProfile.State(elevatorSetpoint,0));
    previousTrapezoidState_Angle = angleTrapezoidProfile.calculate(trapezoidTimer.get(), previousTrapezoidState_Angle, new TrapezoidProfile.State(angleSetpoint,0));
    trapezoidTimer.restart();

    double anglePIDOutput = GetAnglePIDOutput(absAngle);

    if(elevatorEncoder.getPosition() > CoolArmConstants.kMaxPickupBoxElevator && absAngle < CoolArmConstants.kMaxPickupBoxAngle){//if the elevator is above the travel position
      if(anglePIDOutput > 0){
        anglePIDOutput = 0;
      }

      if(currentAction == ArmAction.L1 || currentAction == ArmAction.L2){
        SetElevatorSetpoint(CoolArmConstants.kTravelElevatorSP);
      }
      
      //SetAngleMotor(armPIDController.calculate(absAngle,previousTrapezoidState.position ) + armFFController.calculate( ( (previousTrapezoidState.position - 180) / 180) * Math.PI, 0));
    }
    else if(absAngle > CoolArmConstants.kMaxPickupBoxAngle){
      if(currentAction == ArmAction.L1){
        SetElevatorSetpoint(CoolArmConstants.kL1PrepElevatorSP);
      }else if (currentAction == ArmAction.L2){
        SetElevatorSetpoint(CoolArmConstants.kL2PrepElevatorSP);
      }
      
    }

    //this if statement is to overcome the sticky part of the arm angle control to allow us to consistently place the arm angle on the magnet
    if(absAngle < 105 &&( currentAction == ArmAction.Pickup || currentAction == ArmAction.Travel)){
      anglePIDOutput -= 1.5;
    }
    
    SetAngleMotor(anglePIDOutput);

    if(elevatorControlEnabled){
      //have to reverse this because the setvoltage is reversed and we have to invert this because the PID is smart enough to figure out which way to go
      SetElevatorMotor(-1 * (elevatorPIDController.calculate(elevatorEncoder.getPosition(), previousTrapezoidState_Elevator.position) ) + CoolArmConstants.kElevatorFeedForward);
      
      if((AtElevatorSetpoint(CoolArmConstants.kTravelElevatorSP) && currentAction == ArmAction.Pickup && HasCoralInPickupBin() && AtAngleSetpoint(CoolArmConstants.kTravelAngleSP) && pickupRetryCounter < 3)){
        if(pickupRetryTimer.hasElapsed(0.5) && pickupRetryTimer.isRunning()){
          SetArmAction(ArmAction.Pickup);
          pickupRetryCounter ++;
        //this if statement should try to pick up the coral again if we fail to pick it up the first time
        }
        else{
          pickupRetryTimer.restart();
        }
        
        
      }
    }

    if(raiseLimitSwitch.isPressed()){
      //elevatorEncoder.setPosition(CoolArmConstants.kMaxElevatorPos);

    }
    else if (lowerLimitSwitch.isPressed()){
      elevatorEncoder.setPosition(0);
      if(currentAction == ArmAction.Pickup){
        SetElevatorControlEnabled(true);
        
      }
      //System.out.println("Made it to the lower limit");
    }

    if(pickupTimoutTimer.hasElapsed(0.6) && currentAction == ArmAction.Pickup){
      SetElevatorControlEnabled(true);
    }
    
    
  }

  public double GetAnglePIDOutput(double angle){
    return armPIDController.calculate(angle,previousTrapezoidState_Angle.position ) + armFFController.calculate( ( (previousTrapezoidState_Angle.position - 180) / 180) * Math.PI, 0);

  }

  public boolean AtAngleSetpoint(double sp){
    return Math.abs(absAngleEncoder.getPosition() - sp) < 6;
  }

  public boolean AtElevatorSetpoint(double sp){
    //1.5 is the arbitrary tolerance
    return Math.abs(elevatorEncoder.getPosition() - sp) < 3;
  }

  public boolean AtElevatorAndArmSetpoints(){
    return (AtAngleSetpoint(angleSetpoint) && AtElevatorSetpoint(elevatorSetpoint));
  }

  public double GetElevatorSetpoint(){
    return previousTrapezoidState_Elevator.position;
  }

  

  public void SetArmAction(ArmAction newAction){
    double newAngleSP = absAngleEncoder.getPosition();
    double newElevatorSP = elevatorEncoder.getPosition();
    
    LightSignal newSignal = LightSignal.scoringMode;
    signalLights.ReceiveArmAction(newAction);
    signalLights.autoAligned = false;

    switch(newAction){
      case L1:
        newAngleSP = CoolArmConstants.kL1PrepAngleSP;
        newElevatorSP = CoolArmConstants.kTravelElevatorSP;

        break;
      case L2:
        newAngleSP = CoolArmConstants.kL2PrepAngleSP;
        newElevatorSP = CoolArmConstants.kL2PrepElevatorSP;
        break;
      case L3:
        newAngleSP = CoolArmConstants.kL3PrepAngleSP;
        newElevatorSP = CoolArmConstants.kL3PrepElevatorSP;
        break;
      case L4:
        newAngleSP = CoolArmConstants.kL4PrepAngleSP;
        newElevatorSP = CoolArmConstants.kL4PrepElevatorSP;
        break;
      case Travel:
        newAngleSP = CoolArmConstants.kTravelAngleSP;
        newElevatorSP = CoolArmConstants.kTravelElevatorSP;
        newSignal = LightSignal.loadMode;
        break;
      case Pickup:
        newAngleSP = CoolArmConstants.kPickupAngleSP;
        newElevatorSP = CoolArmConstants.kTravelElevatorSP;
        if(currentAction != ArmAction.Pickup){
          pickupRetryCounter = 0;
        }
        newSignal = LightSignal.loadMode;
        
        break;
      case Place:

        //newAngleSP += CoolArmConstants.kPlaceAngleSPChange;
        //newElevatorSP += CoolArmConstants.kPlaceElevatorSPChange;
        int ifCasesRan = 0;
        if(currentAction == ArmAction.L1){
          newAngleSP = CoolArmConstants.kL2PrepAngleSP;
          newElevatorSP = elevatorSetpoint;
        }
        else if(currentAction == ArmAction.L2){
          newAngleSP = CoolArmConstants.kPlaceAngleSP-10;
          newElevatorSP = 0;
        }
        else if (currentAction == ArmAction.L4){
          newAngleSP = CoolArmConstants.kL4PlaceAngleSP;
          newElevatorSP = elevatorSetpoint;
        }
        else if (currentAction == ArmAction.Travel){
          newAngleSP = CoolArmConstants.kTravelAngleSP;
          newElevatorSP = CoolArmConstants.kTravelHighElevatorSP;
          
        }
        else if (currentAction != ArmAction.Place){
          newAngleSP = CoolArmConstants.kPlaceAngleSP;
          newElevatorSP = elevatorSetpoint;
        }
        else{
          newAngleSP = angleSetpoint;
          newElevatorSP = elevatorSetpoint;
        }

        
        
        break;
    }

    SetAngleSetpoint(newAngleSP);
    SetElevatorSetpoint(newElevatorSP); 
    if(newAction == ArmAction.Pickup){
      SetElevatorMotorManual(-6);
      pickupTimoutTimer.restart();
      pickupRetryTimer.reset();
      pickupRetryTimer.stop();
      
    }
    signalLights.SetSignal(newSignal);
    currentAction = newAction;
  }

  public void SetAngleSetpoint(double sp){

    angleSetpoint = Math.max(90, Math.min(sp, 270));
  }

  public void SetAngleMotor(double speed){
    angleMotor.setVoltage(1 * speed);
  }

  public void SetElevatorSetpoint(double sp){
    elevatorSetpoint = sp;
    SetElevatorControlEnabled(true);
  }

  public void SetElevatorMotor(double voltage){
    //have to drive the motor negative to go up
    elevatorMotor.setVoltage(-1 * voltage);
  }

  public void SetElevatorMotorManual(double voltage){
    SetElevatorControlEnabled(false);
    SetElevatorMotor(voltage);
  }

  public void SetElevatorEncoderPosition(double newValue){
    elevatorEncoder.setPosition(newValue);
  }

  public void SetElevatorControlEnabled(boolean enabled){
    elevatorControlEnabled = enabled;
    if(!enabled){
      SetElevatorMotor(0);
    }
  }

  public void ManualAngleControl(CommandJoystick joystick) {
    SetAngleSetpoint(((joystick.getRawAxis(0) +1) * 90) + 90);
  }

//   public void voltageDrive(Voltage volts){
//       angleMotor.setVoltage(volts);
//   }

//   public void logMotors(SysIdRoutineLog log){
//     log.motor("AngleMotor").voltage(Volts.of( angleMotor.getAppliedOutput()*angleMotor.getBusVoltage() ) ).angularPosition(Degrees.of(absAngleEncoder.getPosition())).angularVelocity(DegreesPerSecond.of(absAngleEncoder.getVelocity()));
// }

  public boolean HasCoralInPickupBin(){
    return !coralPickupSensor.get();
  }

  public void DoAction(CommandJoystick copilotController) {
    ArmAction newAction = currentAction;
    if(copilotController.povUp().getAsBoolean()){
      newAction = ArmAction.L1;
    }    
    else if(copilotController.povRight().getAsBoolean()){
      newAction = ArmAction.L2;
    }
    else if(copilotController.povDown().getAsBoolean()){
      newAction = ArmAction.L3;
    }
    else if(copilotController.povLeft().getAsBoolean()){
      newAction = ArmAction.L4;
    }

    SetArmAction(newAction);

  }
}
