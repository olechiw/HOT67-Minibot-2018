package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
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
        public static final int TALON_PIGEON = 2;
        public static final int TALON_RIGHT = 4;

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
        public static final double MAX_VELOCITY = 1.0;
        public static final double MAX_ACCEL = 2;
        public static final double MAX_JERK = 60.0;

        public static final class POS_PIDVA {
                public static final double P = .02;
                public static final double I = 0;
                public static final double D = 0;
                public static final double V = 1; // Velocity feed forward
                public static final double A = 0; // Acceleration gain
        }

        public static final class ANGLE_PID {
                public static final double P = .015;
                public static final double I = 0;
                public static final double D = 0;
        }

        // Path
        // x = forward
        // y = left
        public static final Waypoint[] MP_Points = new Waypoint[] {
                 new Waypoint(0, 0, Pathfinder.d2r(0)),
                        new Waypoint(1, 0, Pathfinder.d2r(0))};

        // Controllers
        private EncoderFollower leftEncoderFollower;
        private EncoderFollower rightEncoderFollower;

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

                leftTalon.setNeutralMode(NeutralMode.Brake);
                rightTalon.setNeutralMode(NeutralMode.Brake);

                setupMotionProfiling();
        }

        // Motion Profile setup
        Trajectory.Config MP_Config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
        Trajectory.Config.SAMPLES_HIGH, 0.020, MAX_VELOCITY, MAX_ACCEL, MAX_JERK);

        Trajectory MP_Path = Pathfinder.generate(MP_Points, MP_Config);
        private void setupMotionProfiling() {

                double MP_Width = .36;
                TankModifier MP_TankModifier = new TankModifier(MP_Path).modify(MP_Width);

                leftEncoderFollower = new EncoderFollower(MP_TankModifier.getLeftTrajectory());
                rightEncoderFollower = new EncoderFollower(MP_TankModifier.getRightTrajectory());

                leftEncoderFollower.configureEncoder(0, 3600, 0.05436);
                rightEncoderFollower.configureEncoder(0, 3600, 0.05436);

                leftEncoderFollower.configurePIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
                rightEncoderFollower.configurePIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
        }

        int i =0;
        public boolean FollowPath() {
                if (i < MP_Path.segments.length)
                {
                        Segment s = MP_Path.segments[i];
                        SmartDashboard.putNumber("targetPosition", s.position);
                        SmartDashboard.putNumber("targetVelocity", s.velocity);
                        SmartDashboard.putNumber("targetAcceleration", s.acceleration);
                        SmartDashboard.putNumber("targetHeading" ,s.heading);
                        ++i;
                }
                double l = -leftEncoderFollower.calculate((int) leftEncoder);
                double r = -rightEncoderFollower.calculate((int) rightEncoder);

                double heading = xyz_dps[0];
                double desired_heading = Pathfinder.r2d(leftEncoderFollower.getHeading());

                SmartDashboard.putNumber("Heading", heading);
                SmartDashboard.putNumber("Desired Heading", desired_heading);

                double angleError = Pathfinder.boundHalfDegrees(desired_heading - heading);
                double turn = ANGLE_PID.P * angleError;

                leftTalon.set(ControlMode.PercentOutput, l + turn);
                rightTalon.set(ControlMode.PercentOutput, r - turn);

                return (leftEncoderFollower.isFinished() && rightEncoderFollower.isFinished());
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
                pigeon.setYaw(0,0);
        }

        public void zeroTalons() {
                leftTalon.set(ControlMode.PercentOutput, 0);
                rightTalon.set(ControlMode.PercentOutput, 0);
                leftEncoderFollower.reset();
                rightEncoderFollower.reset();
        }

        public void arcadeDrive(double x, double y)
        {
                x = deadband(x, .05); y = deadband(y, .05);
                leftTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, -x);
                rightTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, x);
        }

        public double deadband(double input, double deadband)
        {
                if ((deadband > input) && (input > -deadband))
                        return 0;
                else
                        return input;
        }
}
