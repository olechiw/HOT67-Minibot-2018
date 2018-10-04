package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
    /** Hardware */
    TalonSRX _leftMaster = new TalonSRX(13);
    TalonSRX _rightMaster = new TalonSRX(0);
    Joystick _gamepad = new Joystick(0);

    @Override
    public void robotInit() {
        /* Don't use this for now */
    }
    
    @Override
    public void teleopInit(){
        /* Disable motor controllers */
        _rightMaster.set(ControlMode.PercentOutput, 0);
        _leftMaster.set(ControlMode.PercentOutput, 0);
        
        /* Set Neutral mode */
        _leftMaster.setNeutralMode(NeutralMode.Brake);
        _rightMaster.setNeutralMode(NeutralMode.Brake);
        
        /* Configure output direction */
        _leftMaster.setInverted(false);
        _rightMaster.setInverted(true);
        
        System.out.println("This is a basic arcade drive using Arbitrary Feed Forward.");
    }
    
    @Override
    public void teleopPeriodic() {        
        /* Gamepad processing */
        double forward = -1 * _gamepad.getY();
        double turn = _gamepad.getRawAxis(4)/.5;        
        forward = Deadband(forward);
        turn = Deadband(turn);

        /* Basic Arcade Drive using PercentOutput along with Arbitrary FeedForward supplied by turn */
        _leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
        _rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    }

    /** Deadband 5 percent, used on the gamepad */
    double Deadband(double value) {
        /* Upper deadband */
        if (value >= +0.05) 
            return value;
        
        /* Lower deadband */
        if (value <= -0.05)
            return value;
        
        /* Outside deadband */
        return 0;
    }
}