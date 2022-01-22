/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public class UsbPorts{
                public static final int LEFT_STICK = 0;
                public static final int RIGHT_STICK = 1;
                public static final int CONTROLLER_STICK = 2;
                //public static final int CAMERA_PORT = 0;
        }

        //JOYSTICKS
        public class LeftJoystick{
                // public static final int JOYSTICK_ONE = 1;
                public static final int DOWN_SHIFT = 2;
                public static final int INTAKE_IN = 3;
                // public static final int JOYSTICK_FOUR = 4;
                // public static final int JOYSTICK_FIVE = 5;
                //public static final int TURRET_LEFT = 4;
                public static final int GREEN_ZONE = 11;
                public static final int BLUE_ZONE = 12;
                public static final int SYNC_TURRET = 7;
                // public static final int JOYSTICK_EIGHT = 8;
                public static final int RED_ZONE = 15;
                public static final int YELLOW_ZONE = 16;
                // public static final int ORCHESTRA = 13;
        }

        public class RightJoystick{
                public static final int SHOOT = 1;
                public static final int FORCE_SHOT = 2;
                //public static final int TURRET_UP = 3;
                // public static final int JOYSTICK_FOUR = 4;
                // public static final int JOYSTICK_FIVE = 5;
                // public static final int DROP_HOPPER = 6;
                public static final int ENABLE_TURRET = 7;
                public static final int DISABLE_TURRET = 8;
                // public static final int JOYSTICK_SEVEN = 7;
                // public static final int JOYSTICK_EIGHT = 8;
                // public static final int JOYSTICK_NINE = 9;
                // public static final int JOYSTICK_TEN = 10;
                // public static final int JOYSTICK_ELEVEN = 11;
                public static final int TOGGLE_LIMELIGHT = 11;
        }

        public class ControllerJoystick{
                public static final int SHOOT = 1;
                public static final int INTAKE_IN = 2;
                public static final int FILL_HOPPER = 3;
                public static final int INTAKE_EJECT = 5;
                public static final int FORCE_READY = 4;
                //public static final int TURRET_RIGHT = 5;
                public static final int DROP_INTAKE = 6;
                public static final int MANUAL_OVERRIDE = 7;
                public static final int CLIMBER_EXTEND = 8;
                 public static final int CLIMBER_RETRACT = 9;
                 public static final int LOWER_ARM = 10;
                 public static final int RAISE_ARM = 11;
        }

        //MOTORS
        public class Motors{
                public static final int LEFT_DRIVE_ONE = 4;
                public static final int LEFT_DRIVE_TWO = 5;
                public static final int LEFT_DRIVE_THREE = 6;
                public static final int RIGHT_DRIVE_ONE = 1;
                public static final int RIGHT_DRIVE_TWO = 2;
                public static final int RIGHT_DRIVE_THREE = 3;
                public static final int SHOOTER_TOP = 17;
                public static final int SHOOTER_BOTTOM = 18;
                public static final int HOPPER_NEO = 9;
                public static final int BALL_INDEXER = 10;
                public static final int TURRET_LIFT = 11; // Talon SRX
                public static final int CLIMBER_ONE = 12; // CANspark max
                public static final int TURRET_SPIN = 13; // Talon SRX
                public static final int BALL_INTAKE = 14;   
        }

        // SOLENOIDS
        public class Solenoids{
                public static final int INTAKE_REVERSE = 0;
                public static final int INTAKE_FORWARD = 1;
                public static final int CLIMBER_LOWER = 2;
                public static final int CLIMBER_RAISE = 3;
                public static final int SHIFT_REVERSE = 4;
                public static final int SHIFT_FORWARD = 5;
        }

        public static final class ShooterConstants {
                public static final double kP = 0.25; 
                public static final double kI = 0.0005;
                public static final double kD = 0.0001; 
                public static final double kIz = 8000; 
                public static final double kFF = 0;//.000015; 
                // public static final double kMaxOutput = 1; 
                // public static final double kMinOutput = -1;
                // public static final double maxRPM = 5700;  
        }

        // Trajectory / Pathweaver constants
        public static final class DriveConstants {
                public static final int kLeftMotor1Port = 4;
                public static final int kLeftMotor2Port = 5;
                public static final int kLeftMotor3Port = 6;
                
                public static final int kRightMotor1Port = 1;
                public static final int kRightMotor2Port = 2;
                public static final int kRightMotor3Port = 3;
            
                // public static final int[] kLeftEncoderPorts = new int[] {0, 1};
                // public static final int[] kRightEncoderPorts = new int[] {2, 3};
                // public static final boolean kLeftEncoderReversed = false;
                // public static final boolean kRightEncoderReversed = true;
            
                public static final double kTrackwidthMeters = 1.7;// 1.685; // Units.inchesToMeters(27.0);
                public static final DifferentialDriveKinematics kDriveKinematics =
                    new DifferentialDriveKinematics(kTrackwidthMeters);
            
                public static final int kEncoderCPR = 2048; // 4096;
                public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0); // 0.1524;
                private static final double kWheelGearing = 8.16;  // gearing from motor to swerve drive
                public static final double kEncoderDistancePerPulse = 
                      (kWheelDiameterMeters * Math.PI) / (double) ( kEncoderCPR * kWheelGearing) ;
                //Units.inchesToMeters(3.0) * Math.PI / 16592;  // actual pulses per rotation measured
                ;
                  // Assumes the encoders are directly mounted on the wheel shafts
                    //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
            
                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or theoretically
                // for *your* robot's drive.
                // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
                // values for your robot.
                public static final double ksVolts = 0.519; //0.596;
                public static final double kvVoltSecondsPerMeter = 4.13; //4.13; //2.09; //0.452;
                public static final double kaVoltSecondsSquaredPerMeter = 0.255; //0.155; // 0.0255;

                // Example value only - as above, this must be tuned for your drive!
                // public static final double kPDriveVel = 0.965;
                public static final double kPDriveVel = 1.62; //0.8;
              }
              
              public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 1.0;
                public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; //3;
            
                // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
                public static final double kRamseteB = 2;
                public static final double kRamseteZeta = 0.7;
              }
}
