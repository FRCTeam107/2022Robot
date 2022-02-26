// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.DataRecorder.datapoint;

//import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveDrivetrain extends SubsystemBase {



  //this is where you put the angle offsets you got from the smart dashboard
  // reducing the angle will adjust counter-clockwise direction

  // PRACTICE ROBOT SETTINGS
  // public static double frontLeftOffset = 15.8; //346.90;
  // public static double frontRightOffset = 291.9; //111.9; //70.25;
  // public static double backLeftOffset = 90.7; //273.25;
  // public static double backRightOffset = 75.5;// 255.5; //290.21;

  //COMPETITION ROBOT SETTINGS
  public static double frontLeftOffset = 120;
  public static double frontRightOffset = 349;
  public static double backLeftOffset = 203;
  public static double backRightOffset = 220;


  // //put your can Id's here!
  // public static final int frontLeftDriveId = 1; 
  // public static final int frontLeftCANCoderId = 2; 
  // public static final int frontLeftSteerId = 3;
  // //put your can Id's here!
  // public static final int frontRightDriveId = 4; 
  // public static final int frontRightCANCoderId = 5; 
  // public static final int frontRightSteerId = 6; 
  // //put your can Id's here!

  //   public static final int backLeftDriveId = 10; 
  // public static final int backLeftCANCoderId = 11; 
  // public static final int backLeftSteerId = 12;
  // //put your can Id's here!

  // public static final int backRightDriveId = 7; 
  // public static final int backRightCANCoderId = 8; 
  // public static final int backRightSteerId = 9;   
  public static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  //   new Translation2d(
  //     Units.inchesToMeters(8),
  //     Units.inchesToMeters(15)
  //   ),
  //   new Translation2d(
  //     Units.inchesToMeters(8),
  //     Units.inchesToMeters(-15)
  //   ),
  //   new Translation2d(
  //     Units.inchesToMeters(-8),
  //     Units.inchesToMeters(15)
  //   ),
  //   new Translation2d(
  //     Units.inchesToMeters(-8),
  //     Units.inchesToMeters(-15)
  //   )
  // );

      // Odometry class for tracking robot pose
   SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());
   private SwerveModuleMK3 m_frontLeft, m_frontRight, m_rearLeft, m_rearRight;
   private SwerveModuleMK3[] modules;
   private DataRecorder dataRecorder;

  //  private SwerveModuleMK3[] modules = new SwerveModuleMK3[] {
  //   new SwerveModuleMK3(new TalonFX(frontLeftDriveId), new TalonFX(frontLeftSteerId), new CANCoder(frontLeftCANCoderId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
  //   new SwerveModuleMK3(new TalonFX(frontRightDriveId), new TalonFX(frontRightSteerId), new CANCoder(frontRightCANCoderId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
  //   new SwerveModuleMK3(new TalonFX(backLeftDriveId), new TalonFX(backLeftSteerId), new CANCoder(backLeftCANCoderId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
  //   new SwerveModuleMK3(new TalonFX(backRightDriveId), new TalonFX(backRightSteerId), new CANCoder(backRightCANCoderId), Rotation2d.fromDegrees(backRightOffset))  // Back Right
  // };

  public SwerveDrivetrain() {
   // gyro.reset(); 
  
    m_frontLeft = new SwerveModuleMK3(new TalonFX(Motors.frontLeftDriveId), new TalonFX(Motors.frontLeftSteerId), new CANCoder(Motors.frontLeftCANCoderId), Rotation2d.fromDegrees(frontLeftOffset));
    m_frontRight = new SwerveModuleMK3(new TalonFX(Motors.frontRightDriveId), new TalonFX(Motors.frontRightSteerId), new CANCoder(Motors.frontRightCANCoderId), Rotation2d.fromDegrees(frontRightOffset));
    m_rearLeft = new SwerveModuleMK3(new TalonFX(Motors.backLeftDriveId), new TalonFX(Motors.backLeftSteerId), new CANCoder(Motors.backLeftCANCoderId), Rotation2d.fromDegrees(backLeftOffset));
    m_rearRight = new SwerveModuleMK3(new TalonFX(Motors.backRightDriveId), new TalonFX(Motors.backRightSteerId), new CANCoder(Motors.backRightCANCoderId), Rotation2d.fromDegrees(backRightOffset));
   
    modules = new SwerveModuleMK3[] {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
   
  }

  public void setDataRecorder(DataRecorder _dataRecorder){
    this.dataRecorder = _dataRecorder;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param calibrateGyro button to recalibrate the gyro offset
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
    
    if(calibrateGyro){
      m_gyro.reset(); //recalibrates gyro offset
    }

    if (this.dataRecorder != null)
    {
      this.dataRecorder.recordValue(datapoint.Drive_X, xSpeed);
      this.dataRecorder.recordValue(datapoint.Drive_Y, ySpeed);
      this.dataRecorder.recordValue(datapoint.Drive_Z, rot);
      this.dataRecorder.recordValue(datapoint.GyroAngle, m_gyro.getAngle());
      m_frontLeft.getAngle();
    }

    SwerveModuleState[] states =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      SmartDashboard.putNumber(String.valueOf(i), module.getRawAngle());
     // SmartDashboard.putNumber("speed "+String.valueOf(i), module.get)
      //below is a line to comment out from step 5
      module.setDesiredState(state);
      SmartDashboard.putNumber("gyro Angle", m_gyro.getAngle());
    }
  }
  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),  // frontLeft
        m_rearLeft.getState(),  // frontRight
        m_frontRight.getState(),
        m_rearRight.getState());  
      
        // SmartDashboard.putNumber("FL Dist", m_frontLeft.getDistance() );
        // SmartDashboard.putNumber("BL Dist", m_rearLeft.getDistance() );
        // SmartDashboard.putNumber("FR Dist", m_frontRight.getDistance() );
        // SmartDashboard.putNumber("Br Dist", m_rearRight.getDistance() );
      }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
    /** Resets the drive encoders to currently read a position of 0. */
    // public void resetEncoders() {
    //   m_frontLeft.resetEncoders();
    //   m_rearLeft.resetEncoders();
    //   m_frontRight.resetEncoders();
    //   m_rearRight.resetEncoders();
    // }
  
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      m_gyro.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return m_gyro.getRate(); // * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }


   /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {

    var thetaController =
    new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //SwerveControllerCommand x = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements)

    // sample code utilized the smart speed controllers instead of on-board code in RoboRio 
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            this::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            this::setModuleStates,
            this);

    if (initPose) {
      // Reset odometry to the starting pose of the trajectory.
      var reset =  new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
      return reset.andThen(swerveControllerCommand.andThen(() -> this.drive(0, 0, 0, false, false)));
    }
    else {
      return swerveControllerCommand.andThen(() -> this.drive(0, 0, 0, false, false));
    }
  }
  

    protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
      return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/" + trajectoryName + ".wpilib.json"));
        // Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", trajectoryName + ".wpilib.json")));
    }
    
    public Trajectory loadTrajectoryFromFile(String filename) {
      try {
        return loadTrajectory(filename);
      } catch (IOException e) {
        DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
        return new Trajectory();
      }
    }

  }
