/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
//import frc.robot.Constants.Solenoids;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
private final CANSparkMax m_climber;

private final double kClimberMaxPosition = 200.000; //268.14905;
private final double kClimberMinPosition = 0.0;
private boolean armIsUp;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    super();

    m_climber = new CANSparkMax(Motors.CLIMBER_ONE, MotorType.kBrushless);

    m_climber.restoreFactoryDefaults();
    m_climber.setIdleMode(IdleMode.kBrake);

    m_climber.getEncoder().setPosition(kClimberMinPosition);

    m_climber.setSoftLimit(SoftLimitDirection.kReverse, (float)kClimberMinPosition);
    m_climber.setSoftLimit(SoftLimitDirection.kForward, (float)kClimberMaxPosition);
    m_climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_climber.enableSoftLimit(SoftLimitDirection.kForward, true);

    //m_Solenoid.set(Value.kForward);
    armIsUp = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Climber Position",  m_climber.getEncoder().getPosition());
  }

  public void runMotor(double speed){
    if (!armIsUp){
    //if (m_Solenoid.get() != Value.kForward) {
      m_climber.set(0);
    }
    else {
      double currentPosition = m_climber.getEncoder().getPosition();
      if (speed > 0 && currentPosition > kClimberMaxPosition) {speed=0;}
      if (speed < 0 && currentPosition < kClimberMinPosition) {speed=0;}
      //SmartDashboard.putNumber("Climber speed", speed);
      m_climber.set(speed);
    }
  }

  public void allowAdditionalMovement(){
    m_climber.getEncoder().setPosition((float)((kClimberMaxPosition - kClimberMinPosition)/2));
  }
  public void setToRetractedPosition(){
    m_climber.getEncoder().setPosition((float)kClimberMinPosition);
  }
  
}
