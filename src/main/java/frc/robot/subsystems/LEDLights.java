package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.Motors;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
//import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class LEDLights extends SubsystemBase {
private final PWMSparkMax m_LEDLights;
private double m_color;
public double m_UltraColor;
private final AnalogInput m_ultrasonic;


  /**
   * Creates a new Climber.
   */
  public LEDLights() {
    super();
    m_LEDLights = new PWMSparkMax(1);
    m_ultrasonic = new AnalogInput(0);
    m_color = -0.99;
    m_LEDLights.set(0);
    // SmartDashboard.putNumber("Color of LEDs", m_color);
    //SmartDashboard.putNumber("m_color", m_color);
    
  }

  @Override
  public void periodic() {
    

    
    SmartDashboard.putNumber("Ultra Inches", UltraInches());
    m_UltraColor = SmartDashboard.getNumber("Ultra Inches", UltraInches());

    Color();
    
    m_color = SmartDashboard.getNumber("Color of LEDs", 0);
    // SmartDashboard.putNumber("Color of LEDs", m_color);
    
    // m_color = SmartDashboard.getNumber("Color of LEDs", m_color);
    
    
    //SmartDashboard.updateValues();
    
    //m_color = "Color of LEDs";
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Climber Position",  m_climber.getEncoder().getPosition());
  
    m_LEDLights.set(Color());
  
  }

  public void LightUp(){
    m_LEDLights.set(Color());
    // SmartDashboard.getNumber("Color of LEDs", m_color);
    SmartDashboard.putNumber("Color of LEDs", m_color); 
      
      //m_color += 0.00001;
      //if (m_color > 1) {m_color = -0.99;}
      // SmartDashboard.putNumber("Color of LEDs", m_color);
       
  }

  public double UltraInches(){
    double voltageScaleFactor = 5/RobotController.getVoltage5V();
    double rawValue = m_ultrasonic.getValue();
    SmartDashboard.putNumber("UltraVoltage", rawValue);
    double currentDistanceInches = rawValue * voltageScaleFactor * 0.0492;
    return currentDistanceInches;
  }

  public double Color(){

    if (m_UltraColor > 38 || m_UltraColor < 12) {
      m_color = 0.61;
      //Sets the color to Red if UltraInches is returning less than 30 inches
    }
    if (m_UltraColor > 12 && m_UltraColor < 18) {
      m_color = 0.75;
      //Sets the color to Dark Green if UltraInches is returning more than 30 inches 
      //THIS IS OUR IDEAL SHOOTING POSITION
    }
    if (m_UltraColor > 18 && m_UltraColor < 28) {
      m_color = 0.35;
      //Sets the color to Lime if UltraInches is returning more than 30 inches
      //THIS IS JUST SHORT OF OUR IDEAL SHOOTING POSTITION
    }
    if (m_UltraColor > 28 && m_UltraColor < 38) {
      m_color = 0.73;
      //Sets the color to Blue Green if UltraInches is returning more than 30 inches
      //THIS IS WHEN WE ARE APPROACHING OUT IDEAL SHOOTING POSITION
    }
    

    return m_color;
    

}

}
