/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataRecorder extends SubsystemBase {

  public class datapoint{
    // first entry is timestamp in milliseconds
    public static final int Drive_X = 1;
    public static final int Drive_Y = 2;
    public static final int Drive_Z = 3;
    public static final int GyroAngle = 4;
    public static final int ShooterTop = 5;
    public static final int ShooterBottom = 6;
    public static final int IntakeUpOrDown = 7;
    public static final int IntakeMotorSpeed = 8;
  }

  
  private double[] blankvalues = {0,0,0,0,0,0,0,0,0,0,0};
  private double[] datavalues = blankvalues; // same number of datapoints from  list above

  //private FileInputStream in = null;
 // private FileOutputStream outFile = null;
     private BufferedWriter outBuffer = null;
     private FileWriter outFile = null;

//private final Servo m_Servo = new Servo(0);
  
  public DataRecorder() {
    String test = SmartDashboard.getString("RecordfileName", "file.csv");
    SmartDashboard.putString("RecordfileName", test);
    SmartDashboard.putBoolean("RecordingOn", false); // default to not recording
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    datavalues[0] = System.currentTimeMillis();
    SmartDashboard.putNumberArray("recordedValues", datavalues);

    boolean recordingOn = SmartDashboard.getBoolean("RecordingOn", false);

    if (recordingOn && outBuffer==null){ startRecording(); }
    if (!recordingOn && outBuffer!=null){ endRecording(); }

    //SmartDashboard.putString("DataValues" )
    if (recordingOn && outBuffer!=null) { writeValuesToFile(); }
    
    }
  
  private void writeValuesToFile(){
     if (outBuffer==null) {return;}
     
     // String stringdatavalues = datavalues.toString();
     StringBuilder line = new StringBuilder();
     for (int i=0; i < datavalues.length; i++) {
        line.append(datavalues[i]);
        if (i != datavalues.length - 1) { line.append(','); }     
     }
     line.append("\n");
     try {
      outBuffer.write(line.toString());
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // SmartDashboard.putString("Data Values", line.toString());
  }
  

  public void startRecording(){
   
      //outFile = new FileOutputStream("output.txt"); 
      String fileNameString = SmartDashboard.getString("RecordfileName", "file.csv");
      File file = new File("/home/lvuser/" + fileNameString);
      try {
        if (file.exists()) {
        outFile = new FileWriter(file,false); 
        }
        else {
          outFile = new FileWriter(file);
        }
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

      file.setWritable(true);
      file.setReadable(true);

      outBuffer = new BufferedWriter(outFile);

      datavalues = blankvalues;
      writeValuesToFile();
  }
  
   public void endRecording(){

    if (outFile != null)
    { 
      try {
        outBuffer.flush();
        outFile.flush();
        outBuffer.close();
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
//    if (outFile==null) {return;}
    datavalues[ix] = valueToRecord;
  }

  // LoadFile method reads all lines of files and returns LIST of doubles
  public List<double[]> LoadFile(String fileName) {

    List<double[]> lines = new ArrayList<double[]>();
    String[] strRow;
    double[] row;
    // string xx = Filesystem.getDeployDirectory().getAbsolutePath()
    try (BufferedReader br = new BufferedReader(new FileReader("/home/lvuser/deploy/" + fileName))) {
      for(String line; (line = br.readLine()) != null; ) {
        //System.out.println(line);
        //SmartDashboard.putString("fileLine", line);
        strRow = line.split(",");
        row = new double[strRow.length - 1];
        for (int i=0; i<strRow.length-1; i++) {
          row[i] = Double.parseDouble(strRow[i]); //(double)test2; //Double.parseDouble(strDatapoint.toString());
        }
        lines.add(row);
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    //SmartDashboard.putString("fileLine", "SUCCESS!");
    return lines;
  }
}

