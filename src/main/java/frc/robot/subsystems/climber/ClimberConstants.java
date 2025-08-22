// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.rotary.*;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Ports;

/** Add your docs here. */
public class ClimberConstants {
    public static String NAME = "Climber";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(.4);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(1);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double GEARING = 135;

    private static final Angle MIN_ANGLE = Degrees.of(0.0);
    private static final Angle MAX_ANGLE = Degrees.of(360.0);
    private static final Angle STARTING_ANGLE = Degrees.of(90.0);
    private static final Distance ARM_LENGTH = Inches.of(14.0);

    private static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    private static final Mass ARM_MASS = Pounds.of(4);
    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    // Positional PID
    private static Slot0Configs SLOT0CONFIG = new Slot0Configs()
        .withKP(40.0)
        .withKD(10.0);

    // Climb PID
    private static Slot1Configs SLOT1CONFIG = new Slot1Configs()
        .withKP(1200.0);

    public static TalonFXConfiguration getFXConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = 1.0;

        config.Feedback.SensorToMechanismRatio = GEARING; // TODO: May need sim gear ratio diff

        config.Slot0 = SLOT0CONFIG;
        config.Slot1 = SLOT1CONFIG;

        return config;
    }

    public static RotaryMechanismReal getReal()
    {
        return new RotaryMechanismReal(
            new MotorIOTalonFX(NAME, getFXConfig(), Ports.CLIMBER));
    }

    public static RotaryMechanismSim getSim()
    {
        return new RotaryMechanismSim(
            new MotorIOTalonFXSim(NAME, getFXConfig(), Ports.CLIMBER),
            DCMOTOR, MOI, true, CONSTANTS);
    }

    public static RotaryMechanism getReplay()
    {
        return new RotaryMechanism() {};
    }
}
