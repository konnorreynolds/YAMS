package yams.telemetry;

import edu.wpi.first.units.measure.*;
import yams.motorcontrollers.SmartMotorController;

import static edu.wpi.first.units.Units.*;

public class SmartMotorControllerTelemetryConfig {
    /**
     * Mechanism lower limit reached.
     */
    public boolean mechanismLowerLimitEnabled = false;
    public boolean mechanismLowerLimit = false;
    /**
     * Mechanism upper limit reached.
     */
    public boolean mechanismUpperLimitEnabled = false;
    public boolean mechanismUpperLimit = false;
    /**
     * Motor temperature cutoff reached.
     */
    public boolean temperatureLimitEnabled = false;
    public boolean temperatureLimit = false;
    /**
     * Velocity PID controller used.
     */
    public boolean velocityControlEnabled = false;
    public boolean velocityControl = false;
    /**
     * Elevator feedforward used.
     */
    public boolean elevatorFeedforwardEnabled = false;
    public boolean elevatorFeedforward = false;
    /**
     * Arm feedforward used.
     */
    public boolean armFeedforwardEnabled = false;
    public boolean armFeedforward = false;
    /**
     * Simple feedforward used.
     */
    public boolean simpleFeedforwardEnabled = false;
    public boolean simpleFeedforward = false;
    /**
     * Motion profiling used.
     */
    public boolean motionProfileEnabled = false;
    public boolean motionProfile = false;
    /**
     * Setpoint position given.
     */
    public boolean setpointPositionEnabled = false;
    public double setpointPosition = 0;
    /**
     * Setpoint velocity given.
     */
    public boolean setpointVelocityEnabled = false;
    public double setpointVelocity = 0;
    /**
     * Feedforward voltage supplied to the {@link SmartMotorController}
     */
    public boolean feedforwardVoltageEnabled = false;
    public double feedforwardVoltage = 0.0;
    /**
     * PID Output voltage supplied to the {@link SmartMotorController}
     */
    public boolean pidOutputVoltageEnabled = false;
    public double pidOutputVoltage = 0.0;
    /**
     * Output voltage to the {@link SmartMotorController}
     */
    public boolean outputVoltageEnabled = false;
    public double outputVoltage = 0.0;
    /**
     * Stator current (motor controller output current) to the Motor.
     */
    public boolean statorCurrentEnabled = false;
    public double statorCurrent = 0.0;
    /**
     * Motor temperature.
     */
    public boolean temperatureEnabled = false;
    public Temperature temperature         = Fahrenheit.of(72);
    /**
     * Mechanism distance.
     */
    public boolean distanceEnabled = false;
    public Distance distance            = Meters.of(0);
    /**
     * Mechanism linear velocity.
     */
    public boolean linearVelocityEnabled = false;
    public LinearVelocity linearVelocity      = MetersPerSecond.of(0);
    /**
     * Mechanism position.
     */
    public boolean mechanismPositionEnabled = false;
    public Angle mechanismPosition;
    /**
     * Mechanism velocity.
     */
    public boolean mechanismVelocityEnabled = false;
    public AngularVelocity mechanismVelocity;
    /**
     * Rotor position.
     */
    public boolean rotorPositionEnabled = false;
    public Angle           rotorPosition;
    /**
     * Rotor velocity.
     */
    public boolean rotorVelocityEnabled = false;
    public AngularVelocity rotorVelocity;


    /**
     * This entire class is a WIP method TODO: WIP Callback, also the entire class needs commented.
     * @param telemetry
     */
    public void updateSpecifiedTelemetry(SmartMotorControllerTelemetry telemetry)
    {
        if (mechanismLowerLimitEnabled) {telemetry.mechanismLowerLimit = mechanismLowerLimit;}
        if (mechanismUpperLimitEnabled) {telemetry.mechanismUpperLimit = mechanismUpperLimit;}
        if (temperatureLimitEnabled) {telemetry.temperatureLimit = temperatureLimit;}
        if (velocityControlEnabled) {telemetry.velocityControl = velocityControl;}
        if (elevatorFeedforwardEnabled) {telemetry.elevatorFeedforward = elevatorFeedforward;}
        if (armFeedforwardEnabled) {telemetry.armFeedforward = armFeedforward;}
        if (simpleFeedforwardEnabled) {telemetry.simpleFeedforward = simpleFeedforward;}
        if (motionProfileEnabled) {telemetry.motionProfile = motionProfile;}
        if (setpointPositionEnabled) {telemetry.setpointPosition = setpointPosition;}
        if (setpointVelocityEnabled) {telemetry.setpointVelocity = setpointVelocity;}
        if (feedforwardVoltageEnabled) {telemetry.feedforwardVoltage = feedforwardVoltage;}
        if (pidOutputVoltageEnabled) {telemetry.pidOutputVoltage = pidOutputVoltage;}
        if (outputVoltageEnabled) {telemetry.outputVoltage = outputVoltage;}
        if (statorCurrentEnabled) {telemetry.statorCurrent = statorCurrent;}
        if (temperatureEnabled) {telemetry.temperature = temperature;}
        if (distanceEnabled) {telemetry.distance = distance;}
        if (linearVelocityEnabled) {telemetry.linearVelocity = linearVelocity;}
        if (mechanismPositionEnabled) {telemetry.mechanismPosition = mechanismPosition;}
        if (mechanismVelocityEnabled) {telemetry.mechanismVelocity = mechanismVelocity;}
        if (rotorPositionEnabled) {telemetry.rotorPosition = rotorPosition;}
        if (rotorVelocityEnabled) {telemetry.rotorVelocity = rotorVelocity;}
    }

    public SmartMotorControllerTelemetryConfig withMechanismLowerLimit()
    {
        mechanismLowerLimitEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withMechanismUpperLimit()
    {
        mechanismUpperLimitEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withTemperatureLimit()
    {
        temperatureLimitEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withVelocityControl()
    {
        velocityControlEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withElevatorFeedforward()
    {
        elevatorFeedforwardEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withArmFeedforward()
    {
        armFeedforwardEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withSimpleFeedforward()
    {
        simpleFeedforwardEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withMotionProfile()
    {
        motionProfileEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withSetpointPosition()
    {
        setpointPositionEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withSetpointVelocity()
    {
        setpointVelocityEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withFeedbackVoltage()
    {
        feedforwardVoltageEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withPidOutputVoltage()
    {
        pidOutputVoltageEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withOutputVoltage()
    {
        outputVoltageEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withStatorCurrent()
    {
        statorCurrentEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withTemperature()
    {
        temperatureEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withDistance()
    {
        distanceEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withLinearVelocity()
    {
        linearVelocityEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withMechanismPosition()
    {
        mechanismPositionEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withMechanismVelocity()
    {
        mechanismVelocityEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withRotorPosition()
    {
        rotorPositionEnabled = true;
        return this;
    }
    public SmartMotorControllerTelemetryConfig withRotorVelocity()
    {
        rotorVelocityEnabled = true;
        return this;
    }
}
