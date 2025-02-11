// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class SignalLights extends SubsystemBase {
  
  public AddressableLED leftLEDs;
  public AddressableLED rightLEDs;
  public AddressableLEDBuffer leftLEDBuffer = new AddressableLEDBuffer(LEDConstants.LEFT_LED_COUNT);
  public AddressableLEDBuffer rightLEDBuffer = new AddressableLEDBuffer(LEDConstants.RIGHT_LED_COUNT);

  public boolean hasAlgae = false;

  public Timer animationTimer = new Timer();
  
  public int animationCounter = 0;
  private int animationStepSize = 5;//this is used in party mode and wave color with time

  public LightSignal currentSignal = LightSignal.databits;

  //These are used to know when we toggle or change the LED state.
  private LightSignal previousSignal= LightSignal.hasAlgae;


  public enum LightSignal {
    hasAlgae,
    rightAlign,
    leftAlign,
    climbPrep,
    climbFinish,
    databits,
    databitsAnimated
  }

  /** Creates a new SignalLights. */
  public SignalLights() {
    leftLEDs = new AddressableLED(LEDConstants.LEFT_LED_PORT);
    // rightLEDs = new AddressableLED(LEDConstants.RIGHT_LED_PORT);
    rightLEDs = leftLEDs;

    animationTimer.start();
    animationCounter = 0;

    leftLEDBuffer = new AddressableLEDBuffer(LEDConstants.LEFT_LED_COUNT);
    rightLEDBuffer = new AddressableLEDBuffer(LEDConstants.RIGHT_LED_COUNT);
    //SetArmLEDBufferToSolidColor(LEDConstants.kDatabitsColor);
    leftLEDs.setLength(LEDConstants.LEFT_LED_COUNT);
    rightLEDs.setLength(LEDConstants.RIGHT_LED_COUNT);
    //armLEDs.setData(armLEDBuffer);
    //armLEDs.start();
    currentSignal = LightSignal.databitsAnimated;
  }

  @Override
  public void periodic() {
    boolean ledChanged = false;
    //Determine if we push LED update
    if (previousSignal != currentSignal) 
    {
      previousSignal = currentSignal;
      ledChanged = true;

    }
    // This method will be called once per scheduler run
    switch (currentSignal) {

      case hasAlgae:
          if(hasAlgae){
            SetArmLEDBuffersToSolidColor(LEDConstants.kYesAlgaeColor);
          }
          else{
            SetArmLEDBuffersToSolidColor(LEDConstants.kNoAlgaeColor);
          }
        break;
      case leftAlign:
        SetOneSideLEDBuffersToSolidColor(LEDConstants.kAlignColor, false);
        break;
      case rightAlign:
        SetOneSideLEDBuffersToSolidColor(LEDConstants.kAlignColor, true);
        break;
      case climbPrep:      
        SetArmLEDBuffersToSolidColor(LEDConstants.kClimbReadyColor);
        break;
      case climbFinish:
        PartyMode(animationTimer.get());
        ledChanged = true;
        break;
      case databits:
        SetArmLEDBuffersToSolidColor(LEDConstants.kDatabitsColor);
        break;
      case databitsAnimated:
        WaveColorWithTime(LEDConstants.kDatabitsColor, animationTimer.get());
        //This needs to update every loop
        ledChanged = true;
        break;
    
      default:
        SetArmLEDBuffersToSolidColor(LEDConstants.kErrorColor);
        ledChanged = true;
        break;

      
    }

    //Only push the LED state if it has changed
    if (ledChanged)
    {
      leftLEDs.setData(leftLEDBuffer);
      leftLEDs.start(); 
      rightLEDs.setData(rightLEDBuffer);
      rightLEDs.start(); 
    
    }
  }

  private void WaveColorWithTime(Color color, double timer) {
    animationCounter +=animationStepSize;
    if(animationCounter>180){
      animationCounter = 0;
    }
    //double timerDeg = Units.degreesToRadians(timer);
    for (int i = 0; i < leftLEDBuffer.getLength(); i++) {
      
      double brightness  = ((Math.sin( Units.degreesToRadians( i*animationStepSize + animationCounter ) ) + 1) / 2) / 8;
      leftLEDBuffer.setRGB(i, (int)(brightness * color.red), (int)(brightness * color.green), (int)(brightness * color.blue));
    }

    for (int i = 0; i < rightLEDBuffer.getLength(); i++) {
      
      double brightness  = ((Math.sin( Units.degreesToRadians( i*animationStepSize + animationCounter ) ) + 1) / 2) / 8;
      rightLEDBuffer.setRGB(i, (int)(brightness * color.red), (int)(brightness * color.green), (int)(brightness * color.blue));
    }
   
  }


  private void PartyMode(double timer) {
    animationCounter +=animationStepSize;
    if(animationCounter>180){
      animationCounter = 0;
    }
    //double timerDeg = Units.degreesToRadians(timer);
    for (int i = 0; i < leftLEDBuffer.getLength(); i++) {
      
      leftLEDBuffer.setHSV(i, (animationCounter + (i*animationStepSize))%180, 255, 255);
    }

    for (int i = 0; i < rightLEDBuffer.getLength(); i++) {
      
      rightLEDBuffer.setHSV(i, (animationCounter + (i*animationStepSize))%180, 255, 255);
    }
   
  }

  public void SetArmLEDBuffersToSolidColor(Color color){

    for (var i = 0; i < leftLEDBuffer.getLength(); i++) {
      
      leftLEDBuffer.setLED(i, color);
    }
   
    for (var i = 0; i < rightLEDBuffer.getLength(); i++) {
      
      rightLEDBuffer.setLED(i, color);
    }
  }

  public void SetOneSideLEDBuffersToSolidColor(Color color, boolean isRight){

    for (var i = 0; i < leftLEDBuffer.getLength(); i++) {
      
      leftLEDBuffer.setLED(i, isRight ? LEDConstants.kOffColor : color);
    }
   
    for (var i = 0; i < rightLEDBuffer.getLength(); i++) {
      
      rightLEDBuffer.setLED(i, isRight ? color : LEDConstants.kOffColor);
    }
  }

  public void ReceiveIntakeData(boolean algae){
    hasAlgae = algae;
  }

  public void SetSignal(LightSignal signal){
    currentSignal = signal;
  }

  
}