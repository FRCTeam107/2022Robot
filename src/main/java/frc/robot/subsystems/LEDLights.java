package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDLights extends SubsystemBase {
  private final PWMSparkMax m_LEDLights;
  private int countdownTimer = 0;


  /**
   * Creates a new Climber.
   */
  public LEDLights() {
    super();
    m_LEDLights = new PWMSparkMax(0);
   //m_ultrasonic = new AnalogInput(0);
    //m_color = -0.99;
    m_LEDLights.set(0);
    // SmartDashboard.putNumber("Color of LEDs", m_color);
    //SmartDashboard.putNumber("m_color", m_color);
    
  }

  @Override
  public void periodic() {
    if (countdownTimer >0){
      countdownTimer --;
      if (countdownTimer==0){
        switch (DriverStation.getAlliance()){
          case Blue :
            m_LEDLights.set(0.75);

          case Red:
            m_LEDLights.set(0.61);

          default:
            countdownTimer = 1000;
        }
      }
    }
    
    //SmartDashboard.putNumber("Ultra Inches", UltraInches());
    //m_UltraColor = SmartDashboard.getNumber("Ultra Inches", UltraInches());

    //Color();
    
    //m_color = SmartDashboard.getNumber("Color of LEDs", 0);
    // SmartDashboard.putNumber("Color of LEDs", m_color);
    
    // m_color = SmartDashboard.getNumber("Color of LEDs", m_color);
    
    
    //SmartDashboard.updateValues();
    
    //m_color = "Color of LEDs";
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Climber Position",  m_climber.getEncoder().getPosition());
  
    //m_LEDLights.set(Color());
  
  }

  public void lightsGreen(){
    m_LEDLights.set(0.75);
    countdownTimer = 1000;
  }

  public void lightsYellow(){
    m_LEDLights.set(0.35);
    countdownTimer = 1000;
  }

  public void lightsRed(){
    m_LEDLights.set(0.61);
    countdownTimer = 1000;
  }

  public void lightsBlinkRed(){
    m_LEDLights.set(0.13);
    countdownTimer = 1000;
  }

  public void lightsBlinkGreen(){
    m_LEDLights.set(-0.85);
    countdownTimer = 1000;
  }

  //public void LightUp(){
    //m_LEDLights.set(Color());
    // SmartDashboard.getNumber("Color of LEDs", m_color);
    //SmartDashboard.putNumber("Color of LEDs", m_color); 
      
      //m_color += 0.00001;
      //if (m_color > 1) {m_color = -0.99;}
      // SmartDashboard.putNumber("Color of LEDs", m_color);
       
  //}

  // public double UltraInches(){
  //   double voltageScaleFactor = 5/RobotController.getVoltage5V();
  //   double rawValue = m_ultrasonic.getValue();
  //   SmartDashboard.putNumber("UltraVoltage", rawValue);
  //   double currentDistanceInches = rawValue * voltageScaleFactor * 0.0492;
  //   return currentDistanceInches;
  // }

//   public double Color(){

//     if (m_UltraColor > 38 || m_UltraColor < 12) {
//       m_color = 0.61;
//       //Sets the color to Red if UltraInches is returning less than 30 inches
//     }
//     if (m_UltraColor > 12 && m_UltraColor < 18) {
//       m_color = 0.75;
//       //Sets the color to Dark Green if UltraInches is returning more than 30 inches 
//       //THIS IS OUR IDEAL SHOOTING POSITION
//     }
//     if (m_UltraColor > 18 && m_UltraColor < 28) {
//       m_color = 0.35;
//       //Sets the color to Lime if UltraInches is returning more than 30 inches
//       //THIS IS JUST SHORT OF OUR IDEAL SHOOTING POSTITION
//     }
//     if (m_UltraColor > 28 && m_UltraColor < 38) {
//       m_color = 0.73;
//       //Sets the color to Blue Green if UltraInches is returning more than 30 inches
//       //THIS IS WHEN WE ARE APPROACHING OUT IDEAL SHOOTING POSITION
//     }
    
//     return m_color;
    

// }
}
