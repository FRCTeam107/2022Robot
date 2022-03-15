/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.DataRecorder.datapoint;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX m_IntakeMotor;
  private final WPI_TalonSRX m_IntakeArm;

  private boolean intakeExtended = false;
  private double m_CurrentSpeed;

  //private DataRecorder m_dataRecorder;
  
  public static final class IntakeArmConstants {
    //TODO run arm motor to extended position, find right position
    public static final double armRetractedPos = 0;
    public static final double armExtendedPos = -610000;
    
    // intake arm PID values
    public static final double kP = 0.04; 
    public static final double kI = 0.0002;
    public static final double kD = 0.0; 
    public static final double kIz = 8000; 
    public static final double kFF = 0;//.000015; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700;  
}

  public static final class IntakeMotorConstants {

    public static final double kP = 0.04; 
    public static final double kI = 0.00;
    public static final double kD = 0; 
    public static final double kIz = 8000; 
    public static final double kFF = 0;//.000015; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700; 

  }
  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_IntakeMotor = new WPI_TalonSRX(Motors.BALL_INTAKE);
    m_IntakeMotor.configFactoryDefault();
    m_IntakeMotor.setInverted(false);
    m_IntakeMotor.configClosedloopRamp(0.5);

    
    m_IntakeMotor.config_kP(0, IntakeMotorConstants.kP);
    m_IntakeMotor.config_kI(0, IntakeMotorConstants.kI);
    m_IntakeMotor.config_kD(0, IntakeMotorConstants.kD);
    m_IntakeMotor.config_IntegralZone(0, IntakeMotorConstants.kIz);
    m_IntakeMotor.config_kF(0, IntakeMotorConstants.kFF);

    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);

    double junk = SmartDashboard.getNumber("intakeSpeed", 25000);
    SmartDashboard.putNumber("intakeSpeed", junk);

    m_CurrentSpeed = 0;
   
    m_IntakeArm = new WPI_TalonSRX(Motors.INTAKE_ARM);
    m_IntakeArm.configFactoryDefault();
    m_IntakeArm.setInverted(false);
    m_IntakeArm.setNeutralMode(NeutralMode.Brake);
    m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
    // PID values for INTAKE_ARM
    m_IntakeArm.config_kP(0, IntakeArmConstants.kP);
    m_IntakeArm.config_kI(0, IntakeArmConstants.kI);
    m_IntakeArm.config_kD(0, IntakeArmConstants.kD);
    m_IntakeArm.config_IntegralZone(0, IntakeArmConstants.kIz);
    m_IntakeArm.config_kF(0, IntakeArmConstants.kFF);


    m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)

    m_IntakeArm.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    
   // m_dataRecorder = null;
    intakeExtended = false;
  }

  // public void setDataRecorder(DataRecorder _dataRecorder){
  //   m_dataRecorder = _dataRecorder;
  // }

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    SmartDashboard.putBoolean("IntakeFwdLimit", m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("IntakeRevLimit", m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed());

    // runMotor(m_CurrentSpeed);
        //m_IntakeMotor.set(ControlMode.PercentOutput, m_CurrentSpeed);
    m_IntakeMotor.set(ControlMode.Velocity, m_CurrentSpeed);

    //NetworkTableInstance.getDefault().getTable("dataRecorder").
    SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeMotorSpeed, m_CurrentSpeed);
    if (intakeExtended) {
      SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 1.0);
    }
    else {
      SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 0.0);
    }
    SmartDashboard.putBoolean("dataRecorder.extended", intakeExtended);


//SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    // if upper or lower limit switch is hit, then reset encoder position to upper or lower
    if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed()){
      m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
      if (!intakeExtended) { stopArm(); }
    }
    else if (m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed()){
      m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armExtendedPos);
      if (intakeExtended) { stopArm(); }
    }
  }
  // public void runMotor(double speed){
  //   //m_IntakeMotor.set(ControlMode.PercentOutput, speed);
  //   m_IntakeMotor.set(ControlMode.Velocity, speed);
  // }
public void extendArm(){
  // m_IntakeArm.set(ControlMode.PercentOutput, 1);
  SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 1.0);
  m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armExtendedPos);
  intakeExtended = true;

  // if (m_dataRecorder != null) {
  //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)1.00);
  // }
}

public void retractArm(){
  //m_IntakeArm.set(ControlMode.PercentOutput, -1);
  SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 0.0);

  m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armRetractedPos);
  intakeExtended = false;
  
  // if (m_dataRecorder != null) {
  //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)0.00);
  // }

}

public void stopArm() {
  m_IntakeArm.set(ControlMode.PercentOutput, 0);
}

public void runIntake(double speed){
    m_CurrentSpeed = speed;
  }

  public void HeimlichManeuver() {
    runIntake( -1 * SmartDashboard.getNumber("intakeSpeed", 25000));
  }
  public void StopIntake() {
    runIntake(0);
  }
  public void StartIntake() {
    runIntake(SmartDashboard.getNumber("intakeSpeed", 25000));
  }

}