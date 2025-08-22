// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.linear.*;
import frc.lib.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.util.MechanismUtil.DistanceAngleConverter;
import frc.robot.Ports;
import frc.robot.Robot;

public class ElevatorConstants {
    public static String NAME = "Elevator";

    public static final Distance TOLERANCE = Inches.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        RotationsPerSecond.of(11);
    public static final AngularAcceleration ACCELERATION =
        RotationsPerSecondPerSecond.of(50);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final Distance DRUM_RADIUS = Inches.of(1.756 / 2);
    private static final Mass CARRIAGE_MASS = Kilograms.of(21.5);
    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(2);

    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    private static final double GEARING = (48.0 / 12.0) * (44.0 / 24.0);
    private static final Distance MIN_DISTANCE = Inches.of(0.0);
    private static final Distance MAX_DISTANCE = Inches.of(31);
    private static final Distance STARTING_DISTANCE = Inches.of(0.0);



    private static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(MIN_DISTANCE, MAX_DISTANCE, STARTING_DISTANCE, CONVERTER);

    // Positional PID
    public static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(580.0)
        .withKI(0.0)
        .withKD(50.0)
        .withKG(5.0)
        .withGravityType(GravityTypeValue.Elevator_Static);

    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 50.0;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 70.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: May need inversion
                                                                        // for sim

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            CONVERTER.toAngle(MAX_DISTANCE).in(Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            CONVERTER.toAngle(MIN_DISTANCE).in(Rotations);


        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING;

        config.Slot0 = SLOT0CONFIG;

        return config;
    }

    public static LinearMechanismReal getReal()
    {
        return new LinearMechanismReal(
            new MotorIOTalonFX(NAME, getFXConfig(), Ports.ELEVATOR_MAIN)); // TODO: configure
                                                                           // follower motor
    }

    public static LinearMechanismSim getSim()
    {
        return new LinearMechanismSim(
            new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.ELEVATOR_MAIN),
            DCMOTOR, CARRIAGE_MASS, CHARACTERISTICS, true);
    }

    public static LinearMechanism getReplay()
    {
        return new LinearMechanism() {};
    }
}
