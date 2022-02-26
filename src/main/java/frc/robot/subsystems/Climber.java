/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
//import frc.robot.Constants.Solenoids;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {

private final WPI_TalonFX m_climber;

// private SparkMaxLimitSwitch m_forwardLimit;
// private SparkMaxLimitSwitch m_reverseLimit;
private final double kClimberMaxPosition = 200.000; //268.14905;
private final double kClimberMinPosition = 0.0;
//private boolean armIsUp;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    super();
    m_climber = new WPI_TalonFX(Motors.CLIMBER_ONE);

    m_climber.configFactoryDefault();
    m_climber.setNeutralMode(NeutralMode.Brake);
    m_climber.setSelectedSensorPosition(1);
    
    
    //m_climber.setlimits

    // m_climber.setIdleMode(IdleMode.kBrake);
    // m_climber.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    // m_climber.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  //  m_climber.getEncoder().setPosition(kClimberMinPosition);

   // m_climber.setSoftLimit(SoftLimitDirection.kReverse, (float)kClimberMinPosition);
    //m_climber.setSoftLimit(SoftLimitDirection.kForward, (float)kClimberMaxPosition);
   // m_climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
   // m_climber.enableSoftLimit(SoftLimitDirection.kForward, true);

    //m_Solenoid.set(Value.kForward);
    //armIsUp = true;

    // m_forwardLimit.enableLimitSwitch(false);
    // m_reverseLimit.enableLimitSwitch(false);
    // SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
    // SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
  }

  @Override
  public void periodic() {
    //m_climber.get
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Climber Position",  m_climber.getEncoder().getPosition());
    // m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
    // m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));
SmartDashboard.putNumber("climberPosition", m_climber.getSelectedSensorPosition());

    //   SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    //   SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
  }

  public void runMotor(double speed){
    m_climber.set(speed);
    SmartDashboard.putNumber("climberSpeed", speed);
    // if (!armIsUp){
    // //if (m_Solenoid.get() != Value.kForward) {
    //   m_climber.set(0);
    // }
    // else {
    //   double currentPosition = m_climber.getEncoder().getPosition();
    //   if (speed > 0 && currentPosition > kClimberMaxPosition) {speed=0;}
    //   if (speed < 0 && currentPosition < kClimberMinPosition) {speed=0;}
    //   //SmartDashboard.putNumber("Climber speed", speed);
    //   m_climber.set(speed);
    // }
    //if (m_reverseLimit.isPressed()){
    //  if(speed>0){m_climber.set(speed);}
    //}
    //else{
    //  m_climber.set(0);
    //}
    //if (m_forwardLimit.isPressed()){
    //  if(speed<0){m_climber.set(speed);}
    //}
    //else{
    //  m_climber.set(0);
    //}
  }

  public void allowAdditionalMovement(){
    m_climber.setSelectedSensorPosition((float)((kClimberMaxPosition - kClimberMinPosition)/2));
  }
  public void setToRetractedPosition(){
    m_climber.setSelectedSensorPosition((float)kClimberMinPosition);
  }
  
}
