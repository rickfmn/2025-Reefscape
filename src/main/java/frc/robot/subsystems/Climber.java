// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.SignalLights.LightSignal;

public class Climber extends SubsystemBase {

  public static enum ClimbState{
    Best,//for most of the match before we climb
    Preparing,
    Prepared,
    Climbing,
    Climbed
  }

  private SparkMax m_winchLeader = new SparkMax(ClimberConstants.LEADER_MOTOR_ID, MotorType.kBrushless);
  private SparkMax m_winchFollower = new SparkMax(ClimberConstants.FOLLOWER_MOTOR_ID, MotorType.kBrushless);
  private SparkAbsoluteEncoder m_absEncoder = m_winchLeader.getAbsoluteEncoder();
  private Servo m_servo = new Servo(ClimberConstants.SERVO_ID);

  public ClimbState currentState = ClimbState.Best;

  public SignalLights signalLights;

  /** Creates a new Climber. */
  public Climber(SignalLights lights) {
    signalLights = lights;
    currentState = ClimbState.Best;
    Shuffleboard.getTab("Debug 1").addDouble("Abs Climb Encoder", m_absEncoder::getPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch(currentState){
      case Best:

        if(m_absEncoder.getPosition() < ClimberConstants.CLIMB_BANGLE - 2.5){
          setWinch(0.1);
        }
        else if(m_absEncoder.getPosition() > ClimberConstants.CLIMB_BANGLE + 2.5){
          setWinch(-0.1);
        }
        else{
          setWinch(0);
        }

        break;
      case Climbed:
        setWinch(0);
        //TODO: lights party mode
        break;
      case Climbing:
        double absAngle = m_absEncoder.getPosition();
        if(absAngle > ClimberConstants.CLIMB_VANGLE){
          double interpolatedSpeed = ((absAngle - ClimberConstants.CLIMB_VANGLE)/(ClimberConstants.CLIMB_PANGLE - ClimberConstants.CLIMB_VANGLE)) * (ClimberConstants.kCLIMB_STICTION_SPEED - ClimberConstants.kCLIMB_SPEED) + ClimberConstants.kCLIMB_SPEED;
          setWinch(interpolatedSpeed);

        }
        else{
          setWinch(ClimberConstants.kCLIMB_SPEED);
        }
        

        if(m_absEncoder.getPosition() < ClimberConstants.CLIMB_FANGLE){
          setWinch(0);
          currentState = ClimbState.Climbed;
          //signalLights.SetSignal(LightSignal.climbFinish);
        }

        break;
      case Prepared:

        setWinch(0);


        break;
      case Preparing:

        setWinch(ClimberConstants.kPREPARE_SPEED);

        if(m_absEncoder.getPosition() > ClimberConstants.CLIMB_PANGLE){
          setWinch(0);
          currentState = ClimbState.Prepared;
          signalLights.SetSignal(LightSignal.climbPrep);
        }

        break;
      default:

        setWinch(0);

        break;
      
    }
  }


  public void setWinch(double voltage){
    
    m_winchLeader.set(voltage);
    //other motor should follow
  }

  public void Prepare(){
    currentState = ClimbState.Preparing;
    ReleaseServo();
  }

  public void Climb(){
    currentState = ClimbState.Climbing;
    LockServo();
  }

  //go to the position for the coral intake mode
  public void Best(){
    currentState = ClimbState.Best;
    ReleaseServo();
  }

  public void LockServo(){
    m_servo.set(ClimberConstants.kLOCKED_SERVO_POS);
  }

  
  public void ReleaseServo(){
    m_servo.set(ClimberConstants.kRELEASED_SERVO_POS);
  }



}
