// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.clawroller;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.robot.Ports;
import frc.robot.Robot;

public class ClawRollerConstants {

    public static String NAME = "ClawRoller";

    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(100);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(10);

    private static final double GEARING = (1.0 / 1.0);

    public static final AngularVelocity TOLERANCE = CRUISE_VELOCITY.times(0.1);

    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.0);

    // Velocity PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(800.0)
        .withKI(0.0)
        .withKD(5.0)
        .withKS(9.0);

    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 70.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT0CONFIG;

        return config;
    }

    public static FlywheelMechanismReal getReal()
    {
        return new FlywheelMechanismReal(
            new MotorIOTalonFX(NAME, getFXConfig(), Ports.CLAW_ROLLER));
    }

    public static FlywheelMechanismSim getSim()
    {
        return new FlywheelMechanismSim(
            new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.CLAW_ROLLER),
            DCMOTOR, MOI, TOLERANCE);
    }

    public static FlywheelMechanism getReplay()
    {
        return new FlywheelMechanism() {};
    }
}
