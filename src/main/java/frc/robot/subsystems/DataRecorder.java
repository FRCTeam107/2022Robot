/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
//import java.io.FileOutputStream;
import java.io.IOException;
import java.io.Writer;
import java.nio.Buffer;

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

  
  private double[] blankvalues = {0,0,0,0,0,0,0,0,0,0,0};
  private double[] datavalues = blankvalues; // same number of datapoints from  list above

  //private FileInputStream in = null;
 // private FileOutputStream outFile = null;
     private BufferedWriter outBuffer = null;
     private FileWriter outFile = null;

//private final Servo m_Servo = new Servo(0);

  public DataRecorder() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //datavalues[0] = System.currentTimeMillis();
    SmartDashboard.putNumberArray("recordedValues", datavalues);
    //SmartDashboard.putString("DataValues" )

    if (outBuffer==null) {return;}

    String stringdatavalues = datavalues.toString();
    // SmartDashboard.putString("Data Values", stringdatavalues);
  
    try {
      outBuffer.write(stringdatavalues);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  
  
  }

  public void startRecording(){
   
      //outFile = new FileOutputStream("output.txt");
     try {
      datavalues = blankvalues;
      
      File file = new File("/home/lvuser/file.csv");

      if (file.exists()) {

        outFile = new FileWriter(file,true); 
      }
      else {

        outFile = new FileWriter(file);
      }
      
      file.setWritable(true);
      file.setReadable(true);
      
     

      
      outBuffer = new BufferedWriter(outFile);
      

      SmartDashboard.putString("Absolute path", file.getAbsolutePath());



      String stringdatavalues = datavalues.toString();
      // SmartDashboard.putString("Data Values", stringdatavalues);
      
      outBuffer.write(stringdatavalues);

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
     
    

  }

   public void endRecording(){

    if (outFile != null)
    { 
      try {
        outBuffer.flush();
        outBuffer.close();
        outFile.flush();
        outFile.close();
        outBuffer = null;
        outFile = null;
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

  }

  public void recordValue(Integer ix, double valueToRecord){
    if (outFile==null) {return;}

    datavalues[ix] = valueToRecord;
  }

}

