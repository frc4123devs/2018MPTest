package org.frc4123.robot.mptest;

public class Constants {

    //TODO: Check all these values. Will be a progressive task
    //TODO: Clean up all of these - organize by object/subsystem

    public static final int kTimeCubeEject = 2; //In seconds Todo: Hopefully...

    /***********AUTO***********/

    /**********TELEOP**********/

    //PowerCube Manipulator Constants
    public static final float kEjectCubeSpeedMod = 1;
    public static final float kFoldArmsUp = 1;
    public static final float kFoldArmsDwn = -1;
    public static final double kJoyNeutralZone = 0.5;

    //Climber Constants
    public static final float kClimberUpSpeed = -1;

    //Drive Chassis and wheel and motor constants
    public static final double kChassisWheelDiameterInch = 6;
    public static final double kChassisWheelCircumferenceInch = kChassisWheelDiameterInch*Math.PI;

    //TalonSRX's now have multiple PID Loops so we want to select the first one
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 10;

    //Elevator PID parameters
    //TODO Tune these
    public static final double kElevateP = 1;
    public static final double kElevateI = 0;
    public static final double kElevateD = 45;

    //Drive Chassis PID parameters
    //TODO Tune these
    public static final double kDriveLeftP = 0;
    public static final double kDriveLeftI = 0;
    public static final double kDriveLeftD = 0;
    public static final double kDriveRightP = 0;
    public static final double kDriveRightI = 0;
    public static final double kDriveRightD = 0;
    public static final double kLinearClosedLoop_Tolerance_Default = 1024;

    public static final double kHeadingClosedLoop_P = 0;
    public static final double kHeadingClosedLoop_I = 0;
    public static final double kHeadingClosedLoop_D = 0;
    public static final double kHeadingClosedLoop_Tolerance_Default = 1;

    //Elevator

    //public static final int kElevatorDefaultElevateSpeed = 1; TODO Possibly unneeded
    public static final int kElevatorMaxEncPos = 4000;
    public static final int kElevatorHigh = 32000;
    public static final int kElevatorMedium = 16000;
    public static final int kElevatorLow = 3;
    public static final int kIntegralZone = 700;
    public static final int id_descend_limit = 2; //TODO: Find real input number
    public static final int kElevateMaxPos = 30000;
    public static final int kDescendMaxPos = 30000;
    public static final int kElevateAllowableError = 10;


    //Robot Ports - These should match up to TODO: create Google doc to outline motor controller ports

    //Drive CANTalons
    public static final int id_driveLeftMaster = 1;
    public static final int id_driveLeftSlave = 0;
    public static final int id_driveRightMaster = 2;
    public static final int id_driveRightSlave = 3;

    //TODO: Set these to reality once the electronics is laid out. Sparks?
    //PowerCube Manipulator Speed Controllers
    public static final int id_grabber_wheels = 1;
    public static final int id_grabber_flipper_upper = 0;
    //PowerCube Grabber Limit Switch
    public static final int id_intake_limit = 1; //TODO: Find real input ID

    //Elevator Speed Controllers
    public static final int id_elevateMaster = 4;
    public static final int id_elevateSlave = 5;


    public static double kAutoUpdateRate = 1.0 / 50.0;



    // auto stuff to be sorted
    public static final double kTicksPerFoot = 2607.5945876176131812373915790953; //* 2) / 7;
}
