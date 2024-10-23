package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import java.lang.Math;


import frc.robot.Constants;


public class IntakeConstants {
    public static final double targetRPMHigh = 6000; // any number from -6000 to 6000
    public static final double targetRPMLow = 6000;
    public static final double targetRPMEject = 1800;

    // feeder kp of 2 kS of 3.5 current limits of 40 on everyhting



    public static final double rollerRatio = (1 / 1);
    public static final String CANBUS_NAME = "drivetrain";

    public static final int kInnerIntakeMotorID = 14;
    public static final TalonFXConfiguration innerRollerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs() // current limits for voltage based control
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(50)
            .withSupplyTimeThreshold(0.0)

        ).withTorqueCurrent(new TorqueCurrentConfigs() // current limits for torqueFOC based control
            .withPeakForwardTorqueCurrent(40)
            .withPeakReverseTorqueCurrent(-40)

        ).withSlot0(new Slot0Configs()  // PID gains, more slots can be added for differrent current limits / senarios
            .withKP(2.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKS(8.6)
            .withKV(0.0)
            .withKA(0.0)

        ).withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)

        ).withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(0.0)
        );

    public static final int kOuterIntakeMotorID = 33;
    public static final TalonFXConfiguration outerRollerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs() // current limits for voltage based control
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(50)
            .withSupplyTimeThreshold(0.0)

        ).withTorqueCurrent(new TorqueCurrentConfigs() // current limits for torqueFOC based control
            .withPeakForwardTorqueCurrent(40)
            .withPeakReverseTorqueCurrent(-40)

        ).withSlot0(new Slot0Configs()  // PID gains, more slots can be added for differrent current limits / senarios
            .withKP(2.0)  // untuned
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKS(11.2)
            .withKV(0.0013)
            .withKA(0.0)
        );
}