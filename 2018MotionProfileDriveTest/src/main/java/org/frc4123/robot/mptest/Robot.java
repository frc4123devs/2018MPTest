package org.frc4123.robot.mptest;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.File;

public class Robot extends IterativeRobot {

    //Joy
    Joystick drive = new Joystick(0);

    //Drive
    WPI_TalonSRX l_master = new WPI_TalonSRX(Constants.id_driveLeftMaster);
    WPI_TalonSRX l_slave = new WPI_TalonSRX(Constants.id_driveLeftSlave);
    WPI_TalonSRX r_master = new WPI_TalonSRX(Constants.id_driveRightMaster);
    WPI_TalonSRX r_slave = new WPI_TalonSRX(Constants.id_driveRightSlave);

    SpeedControllerGroup left = new SpeedControllerGroup(l_master);
    SpeedControllerGroup right = new SpeedControllerGroup(r_master);

    DifferentialDrive mDrive = new DifferentialDrive(left, right);


    @Override
    public void robotInit() {
        l_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        r_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        l_master.set(ControlMode.PercentOutput, 0);
        l_slave.set(ControlMode.Follower, Constants.id_driveLeftMaster);
        r_master.set(ControlMode.PercentOutput, 0);
        r_slave.set(ControlMode.Follower, Constants.id_driveRightMaster);
        System.out.println("Robot.robotInit");

        l_master.setSensorPhase(true);
        r_master.setSensorPhase(true);

        r_master.setInverted(true);
        r_slave.setInverted(true);


    }

    @Override
    public void disabledInit() {
        mDrive.setSafetyEnabled(true);
        mDrive.stopMotor();
        System.out.println("Robot.disabledInit");
    }

//    class PeriodicRunnable implements java.lang.Runnable {
//        public void run() {
//            l_master.processMotionProfileBuffer();
//            r_master.processMotionProfileBuffer();
//        }
//    }
//    Notifier _notifer = new Notifier(new PeriodicRunnable());

    @Override
    public void autonomousInit() {
        System.out.println("Robot.autonomousInit");

        l_master.setSelectedSensorPosition(0, 0, 10);
        r_master.setSelectedSensorPosition(0, 0, 10);

        l_master.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
        r_master.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);

//        l_master.config_kF(Constants.kPIDLoopIdx, 0.324247, Constants.kTimeoutMs);
//        r_master.config_kF(Constants.kPIDLoopIdx, 0.324247, Constants.kTimeoutMs);

//        l_master.setSelectedSensorPosition(0, 0, 10);
//        r_master.setSelectedSensorPosition(0, 0, 10);

        File leftTraj = new File("/home/lvuser/trajectories/left_detailed.csv");
        Trajectory left = Pathfinder.readFromCSV(leftTraj);
        Trajectory right = Pathfinder.readFromCSV(new File("/home/lvuser/trajectories/right_detailed.csv"));

        l_master.changeMotionControlFramePeriod(10);
        r_master.changeMotionControlFramePeriod(10);


//        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, .050, 60.0);
//        Waypoint[] points = new Waypoint[] {
//                new Waypoint(0 , 0, 0),
//                new Waypoint(1, 1, 0)
////                new Waypoint(10, 20, Pathfinder.d2r(180))
////                new Waypoint(5, 15, Pathfinder.d2r(270)),
////                new Waypoint(15, 5, Pathfinder.d2r(270)),
////                new Waypoint(10, 0, Pathfinder.d2r(180)),
////                new Waypoint(5, 5, Pathfinder.d2r(90)),
////                new Waypoint(10, 10, 0),
////                new Waypoint(0, 0, 0)
//        };
//
//        Trajectory trajectory = Pathfinder.generate(points, config);
//
//        // Wheelbase Width = 0.5m
//        TankModifier modifier = new TankModifier(trajectory).modify(1.7);
//
//        // Do something with the new Trajectories...
//        Trajectory left = modifier.getLeftTrajectory();
//        Trajectory right = modifier.getRightTrajectory();




        l_master.clearMotionProfileTrajectories();
        r_master.clearMotionProfileTrajectories();
        l_master.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs);
        r_master.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs);
//        l_master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
//        r_master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);


        for (int i = 0; i < left.length(); i++) {
            Trajectory.Segment seg = left.get(i);
            TrajectoryPoint point = new TrajectoryPoint();
            point.position = seg.position * Constants.kTicksPerFoot;
            point.velocity = seg.velocity; //* Constants.kTicksPerFoot;
            point.headingDeg = seg.heading;

            point.profileSlotSelect0 = 0;
            point.profileSlotSelect1 = 0;
            point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_20ms;



            point.zeroPos = false;
            if (i == 0) {
                point.zeroPos = true;
            }

            point.isLastPoint = false;
            if ((i + 1) == left.length()) {
                point.isLastPoint = true;
            }

            l_master.pushMotionProfileTrajectory(point);


        }

        for (int i = 0; i < right.length(); i++) {
            Trajectory.Segment seg = right.get(i);
            TrajectoryPoint point = new TrajectoryPoint();
            point.position = seg.position * Constants.kTicksPerFoot;
            point.velocity = seg.velocity * Constants.kTicksPerFoot;
            point.headingDeg = seg.heading;

            point.profileSlotSelect0 = 0;
            point.profileSlotSelect1 = 0;
            point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_20ms;

            point.zeroPos = false;
            if (i == 0) {
                point.zeroPos = true;
            }

            point.isLastPoint = false;
            if ((i + 1) == right.length()) {
                point.isLastPoint = true;
            }

            System.out.println("point.isLastPoint || point.zeroPos = " + (point.isLastPoint || point.zeroPos));
            //System.out.println("pointvel = " + point.velocity);
            r_master.pushMotionProfileTrajectory(point);


            mDrive.setSafetyEnabled(false);
        }


    }

    @Override
    public void teleopInit() {

        mDrive.setSafetyEnabled(true);
    }

    @Override
    public void testInit() {
        mDrive.setSafetyEnabled(true);
    }

    @Override
    public void disabledPeriodic() {
    }

    MotionProfileStatus l_motionProfileStatus = new MotionProfileStatus();
    MotionProfileStatus r_motionProfileStatus = new MotionProfileStatus();

    @Override
    public void autonomousPeriodic() {

        l_master.getMotionProfileStatus(l_motionProfileStatus);
        r_master.getMotionProfileStatus(r_motionProfileStatus);
        System.out.println("l_motionProfileStatus.isUnderrun = " + l_motionProfileStatus.isUnderrun);
        System.out.println("l_motionProfileStatus.topBufferCnt = " + l_motionProfileStatus.topBufferCnt);
        System.out.println("l_motionProfileStatus.btmBufferCnt = " + l_motionProfileStatus.btmBufferCnt);
        System.out.println("l_motionProfileStatus.isLast = " + l_motionProfileStatus.isLast);

        if (l_motionProfileStatus.btmBufferCnt < 64) {
            l_master.processMotionProfileBuffer();
        }
        if (r_motionProfileStatus.btmBufferCnt < 64) {
            r_master.processMotionProfileBuffer();
        }


//        l_master.getMotionProfileStatus()


//        l_master.set(ControlMode.PercentOutput, 0.25);
//        r_master.set(ControlMode.PercentOutput, 0.25);

//        l_master.motio

        if (l_motionProfileStatus.btmBufferCnt > 0 && r_motionProfileStatus.btmBufferCnt > 0) {
            l_master.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
            r_master.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
        }

    }

    @Override
    public void teleopPeriodic() {

        if (drive.getRawButton(1)) {
            System.out.println("resetting pos");
            l_master.setSelectedSensorPosition(0, 0, 10);
            r_master.setSelectedSensorPosition(0, 0, 10);
        }

        mDrive.arcadeDrive(drive.getRawAxis(4), -drive.getRawAxis(1));

    }

    @Override
    public void testPeriodic() {
        if (drive.getRawButton(1)) {
            System.out.println("resetting pos");
            l_master.setSelectedSensorPosition(0, 0, 10);
            r_master.setSelectedSensorPosition(0, 0, 10);
        }
//
//        if (drive.getRawButton(2)){
//            l_master.set(ControlMode.Position, 18000);
//            r_master.set(ControlMode.Position, 18000);
//        } else {
//            mDrive.arcadeDrive(drive.getRawAxis(4), -drive.getRawAxis(1));
//        }
    }
}