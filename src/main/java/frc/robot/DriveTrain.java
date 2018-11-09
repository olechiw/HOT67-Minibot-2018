package frc.robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrain {

    public static final int TALON_LEFT = 1;
    public static final int TALON_RIGHT = 4;
    public static final int TALON_PIGEON = 2;

    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);
    WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(TALON_PIGEON);

    PigeonIMU pigeon = new PigeonIMU(pigeonTalon);

    private double leftEncoder;
    private double rightEncoder;
    private double[] xyz_dps = new double[3];

    /*
     * Motion Profiling Constants
     */

    // Control parameters
    public static final double MAX_VELOCITY = 1;

    // TODO: CHECK UNITS
    public static final class AllowableError {
        public static final int POS = 500;
        public static final int ANGLE = 1;
    }

    public static final class LEFT_PIDF {
        public static final double P = .05;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = .5; // Velocity feed forward, page 17 of motion profiling manual
    }

    public static final class RIGHT_PIDF {
        public static final double P = .05;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = .5; // Velocity feed forward, page 17 of motion profiling manual
    }

    public static final class ANGLE_PID {
        public static final double P = .015;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0; // Turn feed forward, page 17 of motion profiling manual (maybe????)
    }

    // Path
    public static final Waypoint[] MP_Points = new Waypoint[] { new Waypoint(0, 0, Pathfinder.d2r(0)),
            new Waypoint(1, 1, Pathfinder.d2r(90)), new Waypoint(1.7, 1.7, Pathfinder.d2r(0)) };

    // Generated path
    Trajectory leftTrajectory;
    Trajectory rightTrajectory;

    public DriveTrain() {
        leftTalon.setSensorPhase(true);
        leftTalon.setInverted(true);

        leftTalon.selectProfileSlot(0, 0);
        rightTalon.selectProfileSlot(0, 0);

        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        leftTalon.setSelectedSensorPosition(0, 0, 0);
        rightTalon.setSelectedSensorPosition(0, 0, 0);

        leftTalon.set(ControlMode.PercentOutput, 0.0);
        rightTalon.set(ControlMode.PercentOutput, 0.0);

        setupMotionProfiling();
    }

    class PeriodicRunnable implements Runnable {
        public void run() {
            leftTalon.processMotionProfileBuffer();
            rightTalon.processMotionProfileBuffer();
        }
    }

    Notifier MP_Notifier = new Notifier(new PeriodicRunnable());

    private void setupMotionProfiling() {
        //TODO: finish motor config
        /*
        https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionProfileAuxiliary%5BMotionProfileArc%5D/src/org/usfirst/frc/team217/robot/Robot.java
        3.8 FIRMWARE OR GREATER
        */

        // Motion Profile setup
        Trajectory.Config MP_Config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_FAST, 0.020, MAX_VELOCITY, 2.0, 60.0);

        Trajectory MP_Path = Pathfinder.generate(MP_Points, MP_Config);

        double MP_Width = .36;
        TankModifier MP_TankModifier = new TankModifier(MP_Path).modify(MP_Width);

        leftTrajectory = MP_TankModifier.getLeftTrajectory();
        rightTrajectory = MP_TankModifier.getRightTrajectory();

        leftTalon.config_kP(0, LEFT_PIDF.P, 20);
        leftTalon.config_kI(0, LEFT_PIDF.I, 20);
        leftTalon.config_kD(0, LEFT_PIDF.D, 20);
        leftTalon.config_kF(0, LEFT_PIDF.F, 20);

        leftTalon.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 0, 20);
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 20);
        leftTalon.config_kP(1, ANGLE_PID.P, 20);
        leftTalon.config_kI(1, ANGLE_PID.I, 20);
        leftTalon.config_kD(1, ANGLE_PID.D, 20);

        rightTalon.config_kP(0, RIGHT_PIDF.P, 20);
        rightTalon.config_kI(0, RIGHT_PIDF.I, 20);
        rightTalon.config_kD(0, RIGHT_PIDF.D, 20);
        rightTalon.config_kF(0, RIGHT_PIDF.F, 20);

        rightTalon.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 0, 20);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 20);
        rightTalon.config_kP(1, ANGLE_PID.P, 20);
        rightTalon.config_kI(1, ANGLE_PID.I, 20);
        rightTalon.config_kD(1, ANGLE_PID.D, 20);

        fillTalon(leftTrajectory, leftTalon);
        fillTalon(rightTrajectory, rightTalon);

        // Twice as fast as profile - page 21 of MP manual
        leftTalon.changeMotionControlFramePeriod(10);
        leftTalon.configAllowableClosedloopError(0, AllowableError.POS, 20);
        leftTalon.configAllowableClosedloopError(1, AllowableError.ANGLE, 20);
        rightTalon.changeMotionControlFramePeriod(10);
        rightTalon.configAllowableClosedloopError(0, AllowableError.POS, 20);
        rightTalon.configAllowableClosedloopError(1, AllowableError.ANGLE, 20);
        MP_Notifier.startPeriodic(.01);
        ;
    }

    /*
     * Fill the talons with MP. Page 21 of motion profiling manual
     */
    private static void fillTalon(Trajectory path, WPI_TalonSRX talon) {
        talon.clearMotionProfileTrajectories();

        TrajectoryPoint point = new TrajectoryPoint();

        for (int i = 0; i < path.length(); ++i) {
            Segment s = path.get(i);
            point.position = s.position;
            point.headingDeg = Pathfinder.r2d(s.heading);
            point.timeDur = TrajectoryDuration.valueOf((int) (s.dt * 1000)); // Takes ms
            point.profileSlotSelect0 = 0; // Position (MP Manual pg 8)
            point.profileSlotSelect1 = 1; // Heading (MP Manual pg 8)
            point.zeroPos = false;

            if (i == 0)
                point.zeroPos = true;
            if ((i + 1) == path.length())
                point.isLastPoint = true;

            talon.pushMotionProfileTrajectory(point);
        }
    }

    private boolean MP_Enabled = false;

    public boolean FollowPath() {
        MotionProfileStatus leftStatus = new MotionProfileStatus();
        MotionProfileStatus rightStatus = new MotionProfileStatus();
        leftTalon.getMotionProfileStatus(leftStatus);
        rightTalon.getMotionProfileStatus(rightStatus);

        // 5 points in buffer minimum before we start
        if (!MP_Enabled && leftStatus.btmBufferCnt >= 5 && rightStatus.btmBufferCnt > 5) {
            leftTalon.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
            rightTalon.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Enable.value);
            MP_Enabled = true;
        }

        // Check for finish
        if (leftStatus.activePointValid && leftStatus.isLast && rightStatus.activePointValid && rightStatus.isLast) {
            return true;
        }

        return false;
    }

    public void readSensors() {
        leftEncoder = leftTalon.getSelectedSensorPosition(0);
        rightEncoder = rightTalon.getSelectedSensorPosition(0);

        pigeon.getYawPitchRoll(xyz_dps);
    }

    public void writeDashBoard() {
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("currentYaw", xyz_dps[0]);
    }

    public void zeroSensors() {
        leftTalon.setInverted(true);
        leftTalon.setSelectedSensorPosition(0, 0, 20);
        rightTalon.setSelectedSensorPosition(0, 0, 20);
        pigeon.setYaw(0, 0);
    }

    public void zeroTalons() {
        leftTalon.set(ControlMode.PercentOutput, 0);
        rightTalon.set(ControlMode.PercentOutput, 0);
        leftTalon.clearMotionProfileTrajectories();
        rightTalon.clearMotionProfileTrajectories();
        MP_Enabled = false;
    }

    public void arcadeDrive(double x, double y) {
        x = deadband(x, .05);
        y = deadband(y, .05);
        leftTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, -x);
        rightTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, x);
    }

    public static double deadband(double input, double deadband) {
        if ((deadband > input) && (input > -deadband))
            return 0;
        else
            return input;
    }
}
