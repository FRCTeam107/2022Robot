/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataRecorder extends SubsystemBase {

  public class datapoint{
    // first entry is timestamp in milliseconds
    public static final int FrontLeftDrive = 1;
    public static final int FrontLeftSteer = 2;
    public static final int FrontRightDrive = 3;
    public static final int FrontRightSteer = 4;
    public static final int RearLeftDrive = 5;
    public static final int RearLeftSteer = 6;
    public static final int RearRightDrive = 7;
    public static final int RearRightSteer = 8;
    public static final int ShooterTop = 9;
    public static final int ShooterBottom = 10;
  }

  private double[] datavalues = {0,0,0,0,0,0,0,0,0,0,0}; // same number of datapoints from  list above
  

//private final Servo m_Servo = new Servo(0);

  public DataRecorder() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //datavalues[0] = System.currentTimeMillis();
    SmartDashboard.putNumberArray("recordedValues", datavalues);
  }

  public void startRecording(double position){

  }

   public void endRecording(){

  }

  public void recordValue(Integer ix, double valueToRecord){
    datavalues[ix] = valueToRecord;
  }

}

