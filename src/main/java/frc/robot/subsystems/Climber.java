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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {

private final WPI_TalonFX m_climber, m_climberArm;

public static final class ClimberConstants {
  private static final double armExtendPos = 200.000; //268.14905;
  private static final double armHomePos = 0.0;

    //TODO tune the climber arm PID values
    public static final double kP = 0.04; 
    public static final double kI = 0.0001;
    public static final double kD = 0.0; 
    public static final double kIz = 4000; 
    public static final double kFF = 0;  //.000015; 
}
// private SparkMaxLimitSwitch m_forwardLimit;
// private SparkMaxLimitSwitch m_reverseLimit;
//private boolean armIsUp;


public static final class ClimberArmConstants {
  //TODO run arm motor to extended position, find right position
  public static final double armReachBackPos = 0;
  public static final double armVerticalPos = -170000;
  
  //TODO tune the climber arm PID values
  public static final double kP = 0.04; 
  public static final double kI = 0.0001;
  public static final double kD = 0.0; 
  public static final double kIz = 4000; 
  public static final double kFF = 0;  //.000015; 
  // public static final double kMaxOutput = 1; 
  // public static final double kMinOutput = -1;
  // public static final double maxRPM = 5700;  
  }

/**
   * Creates a new Climber.
   */
  public Climber() {
    super();
    m_climber = new WPI_TalonFX(Motors.CLIMBER_ONE);

    m_climber.configFactoryDefault();
    m_climber.setNeutralMode(NeutralMode.Brake);
    m_climber.setSelectedSensorPosition(ClimberConstants.armHomePos);
    
    m_climber.config_kP(0, ClimberConstants.kP);
    m_climber.config_kI(0, ClimberConstants.kI);
    m_climber.config_kD(0, ClimberConstants.kD);
    m_climber.config_IntegralZone(0, ClimberConstants.kIz);
    m_climber.config_kF(0, ClimberConstants.kFF);


    //m_climber.setlimits

    // m_climber.setIdleMode(IdleMode.kBrake);
    // m_climber.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    // m_climber.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  //  m_climber.getEncoder().setPosition(kClimberMinPosition);

   // m_climber.setSoftLimit(SoftLimitDirection.kReverse, (float)kClimberMinPosition);
    //m_climber.setSoftLimit(SoftLimitDirection.kForward, (float)kClimberMaxPosition);
   // m_climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
   // m_climber.enableSoftLimit(SoftLimitDirection.kForward, true);

    //armIsUp = true;

    m_climberArm = new WPI_TalonFX(Motors.CLIMBER_ARM);
    m_climberArm.configFactoryDefault();
    m_climberArm.setInverted(false);
    m_climberArm.setNeutralMode(NeutralMode.Brake);
    m_climberArm.setSelectedSensorPosition(ClimberArmConstants.armReachBackPos);
    // PID values for INTAKE_ARM
    m_climberArm.config_kP(0, ClimberArmConstants.kP);
    m_climberArm.config_kI(0, ClimberArmConstants.kI);
    m_climberArm.config_kD(0, ClimberArmConstants.kD);
    m_climberArm.config_IntegralZone(0, ClimberArmConstants.kIz);
    m_climberArm.config_kF(0, ClimberArmConstants.kFF);


    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)
    //m_IntakeArm.get

  }

  @Override
  public void periodic() {
    //m_climber.get
    // This method will be called once per scheduler run

SmartDashboard.putNumber("climberPosition", m_climber.getSelectedSensorPosition());

// if upper or lower limit switch is hit, then reset encoder position to upper or lower
    // if (m_climberArm.getSensorCollection().isFwdLimitSwitchClosed()){
    //   m_climberArm.setSelectedSensorPosition(ClimberArmConstants.armReachBackPos);
    // }
    // else if (m_climberArm.getSensorCollection().isRevLimitSwitchClosed()){
    //   m_climberArm.setSelectedSensorPosition(ClimberConstants.armHomePos);
    // }
  }

  public void runMotor(double speed){
    m_climber.set(speed);
   // SmartDashboard.putNumber("climberSpeed", speed);
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

  // public void allowAdditionalMovement(){
  //   m_climber.setSelectedSensorPosition((float)((kClimberMaxPosition - kClimberMinPosition)/2));
  // }
  // public void setToRetractedPosition(){
  //   m_climber.setSelectedSensorPosition((float)kClimberMinPosition);
  // }

  public void moveArmtoReachBack(){
    m_climberArm.set(ControlMode.Position, ClimberArmConstants.armReachBackPos);
  }

  public void moveArmToVertical(){
    m_climberArm.set(ControlMode.Position, ClimberArmConstants.armVerticalPos);
  }

  public void stopArm(){
    m_climberArm.set(ControlMode.PercentOutput, 0);
  }


  public void extendClimber(){
    m_climber.set(ControlMode.Position, ClimberConstants.armExtendPos);
  }

  public void pullClimber(){
    m_climber.set(ControlMode.Position, ClimberConstants.armHomePos);
  }
  public void stopClimber(){
    m_climber.set(ControlMode.PercentOutput, 0);
  }
  
}
