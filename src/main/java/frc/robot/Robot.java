/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BiFunction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    public static final int TALON_LEFT = 13;
    public static final int TALON_RIGHT = 0;

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 4;
    public static final int JOYSTICK_RX = 1;


    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);

    Joystick driver = new Joystick(JOYSTICK_DRIVER);

    DifferentialDrive drive;

    @Override
    public void robotInit() {
        drive = new DifferentialDrive(leftTalon, rightTalon);

        // Inverting left, may have to change
        /*
        leftTalon.setInverted(false);
        rightTalon.setInverted(true);
        */

        leftTalon.selectProfileSlot(0, 0);
        rightTalon.selectProfileSlot(0, 0);

        leftTalon.set(ControlMode.PercentOutput, 0.0);
        rightTalon.set(ControlMode.PercentOutput, 0.0);
    }


    @Override
    public void robotPeriodic() {
        BiFunction<Double, Double, Double> getInputWithDeadband = (input, deadband) -> {
            if ((-deadband < input) && (input < deadband))
                return 0.0;
            else
                return input; // Scale to 0-1
        };

        
        // May have to invert driveturn
        double driveSpeed = getInputWithDeadband.apply(driver.getRawAxis(1), .2);
        double driveTurn = getInputWithDeadband.apply(driver.getRawAxis(4), .2);
        /*
        double driveTurn = driver.getRawAxis(JOYSTICK_LY);
        double driveTurn = driver.getRawAxis(JOYSTICK_RX);
        */

        SmartDashboard.putNumber("driveSpeed", driveSpeed);
        SmartDashboard.putNumber("driveTurn", driveTurn);

        
        drive.arcadeDrive(driveSpeed, driveTurn);
        /*
        leftTalon.set(ControlMode.PercentOutput, driveTurn);
        rightTalon.set(ControlMode.PercentOutput, driveTurn);
        */
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }
}
