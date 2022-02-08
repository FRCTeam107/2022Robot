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
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.FilenameFilter;
//import java.io.FileOutputStream;
import java.io.IOException;
import java.io.Writer;
import java.nio.Buffer;
import java.nio.file.FileSystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;
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
    SmartDashboard.putString("RecordfileName", "file.csv");
    
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

     SmartDashboard.putString("Data Values", line.toString());
   
    //  try {
    //    outBuffer.write(stringdatavalues);
    //  } catch (IOException e) {
    //    // TODO Auto-generated catch block
    //    e.printStackTrace();
    //  } 
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //datavalues[0] = System.currentTimeMillis();
    SmartDashboard.putNumberArray("recordedValues", datavalues);
    //SmartDashboard.putString("DataValues" )
    if (outBuffer==null) {return;}
    writeValuesToFile();
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

    //   try {
     
    //   //SmartDashboard.putString("Absolute path", file.getAbsolutePath());

    //  // writeValuesToFile();
    //   String stringdatavalues = datavalues.toString();
    //   // SmartDashboard.putString("Data Values", stringdatavalues);
      
    //  // outBuffer.write(stringdatavalues);

    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
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
  public void testload(){
    List<double[]> lines = LoadFile("Noahsfile.csv");
  }
  public List<double[]> LoadFile(String fileName) {
    // Path filPath = new Path() {
      
    // };
    // List<Double[]> lines = 
    // Files.lines(Pathes.get("/path/to/file.csv"))
    //      .skip(1) // Skip the heading
    //      .map(line -> Arrays.stream(line.split(","))
    //                         .skip(1) // Skip the "station_readings"
    //                         .map(Double::new)
    //                         .toArray(Double[]::new)
    //           )
    //      .collect(Collectors.toList());
    // ////

// read all lines of file
// try (Stream<String> lines = Files.lines(Paths.get("/Users/dshvechikov/file"))) {
//   lines.forEach(System.out::println);
// }

  List<double[]> lines = new ArrayList<double[]>();
  String[] strRow;
  double[] row;
  // string xx = Filesystem.getDeployDirectory().getAbsolutePath()
  try (BufferedReader br = new BufferedReader(new FileReader("/home/lvuser/deploy/" + fileName))) {
    for(String line; (line = br.readLine()) != null; ) {
        //System.out.println(line);
        SmartDashboard.putString("fileLine", line);
        strRow = line.split(",");
        row = new double[strRow.length - 1];
         for (int i=1; i<strRow.length; i++) {
           row[i - 1] = Double.parseDouble(strRow[i]);
         }
      lines.add(row);
    }
  } catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
  }
  SmartDashboard.putString("fileLine", "SUCCESS!");

    return lines;

  }

}

