/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BiFunction;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

    public static final int TALON_LEFT = 1;
    public static final int TALON_RIGHT = 4;

    public static final int PIGEON = 2;

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 0;
    public static final int JOYSTICK_RX = 4;

    public static final double LEFT_DEADBAND = .2;
    public static final double RIGHT_DEADBAND = .2;

    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);

    PigeonIMU pigeon = new PigeonIMU(2);

    Joystick driver = new Joystick(JOYSTICK_DRIVER);

    DifferentialDrive drive;

    @Override
    public void robotInit() {
        drive = new DifferentialDrive(leftTalon, rightTalon);

        // Inverting left, may have to change
        leftTalon.setInverted(true);
        rightTalon.setInverted(false);
    }


    @Override
    public void robotPeriodic() {
        BiFunction<Double, Double, Double> getInputWithDeadband = (input, deadband) -> {
            if ((-deadband < input) && (input < deadband))
                return 0.0;
            else
                return (input / (1 - deadband)); // Scale to 0-1
        };

        double driveSpeed = getInputWithDeadband.apply(driver.getRawAxis(0), LEFT_DEADBAND);
        double driveTurn = getInputWithDeadband.apply(driver.getRawAxis(4), RIGHT_DEADBAND);

        drive.arcadeDrive(driveSpeed, driveTurn);
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
