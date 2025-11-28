package yams.mechanisms.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.simulation.MapleModuleSim;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

/**
 * Swerve Module
 */
public class SwerveModule
{

  /**
   * Drive motor controller.
   */
  private final SmartMotorController m_driveMotorController;
  /**
   * Azimuth motor controller.
   */
  private final SmartMotorController m_azimuthMotorController;
  /**
   * Swerve module configuration.
   */
  private final SwerveModuleConfig   m_config;
  /**
   * Mechanism Telemetry
   */
  private final MechanismTelemetry   m_telemetry = new MechanismTelemetry();

  /**
   * Create a SwerveModule.
   *
   * @param config {@link SwerveModuleConfig} for the module.
   */
  public SwerveModule(SwerveModuleConfig config)
  {
    m_config = config;
    m_driveMotorController = config.getDriveMotor();
    m_azimuthMotorController = config.getAzimuthMotor();
    if (m_config.getTelemetryName().isEmpty())
    {
      throw new IllegalArgumentException("SwerveModuleConfig must have a telemetry name!");
    }
    if (m_config.getLocation().isEmpty())
    {
      throw new IllegalArgumentException("SwerveModuleConfig must have a position!");
    }
    if (m_azimuthMotorController.getConfig().getExternalEncoder().isPresent() &&
        !m_azimuthMotorController.getConfig().getUseExternalFeedback())
    {
      throw new SmartMotorControllerConfigurationException("External encoder cannot be used without external feedback",
                                                           "External encoder could not be used",
                                                           "withUseExternalFeedbackEncoder(true)");
    }
    m_telemetry.setupTelemetry("swerve/" + getName() + "/drive", m_driveMotorController);
    m_telemetry.setupTelemetry("swerve/" + getName() + "/azimuth", m_azimuthMotorController);
    seedAzimuthEncoder();
  }

    /**
   * Seed the azimuth encoder with the absolute encoder angle.
   */
  public void seedAzimuthEncoder()
  {
    if (RobotBase.isReal() && (m_azimuthMotorController.getConfig().getExternalEncoder().isEmpty() ||
                               !m_azimuthMotorController.getConfig().getUseExternalFeedback()))
    {
      m_azimuthMotorController.setEncoderPosition(m_config.getAbsoluteEncoderAngle());
    }
  }

  /**
   * Get the name of the module.
   *
   * @return Name of the module.
   */
  public String getName()
  {
    return m_config.getTelemetryName().orElse("SwerveModule");
  }

  /**
   * Get the {@link SwerveModuleConfig} for the module.
   *
   * @return {@link SwerveModuleConfig} for the module.
   */
  public SwerveModuleConfig getConfig()
  {
    return m_config;
  }

  /**
   * Set the {@link SwerveModuleState} of the module.
   *
   * @param state State to set.
   */
  public void setSwerveModuleState(SwerveModuleState state)
  {
    state = m_config.getOptimizedState(state);
    // If in simulation and maple module sim is present, run the module state.
    if (RobotBase.isSimulation()) {
        if (m_config.getMapleModuleSim().isPresent()) {
            m_config.getMapleModuleSim().get().runModuleState(state);
            return; // Return early to avoid updating the motor controllers.
        }
    }
    // Otherwise run the motor controllers.
    m_driveMotorController.setVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
    m_azimuthMotorController.setPosition(state.angle.getMeasure());
  }

  /**
   * Get the {@link SwerveModuleState} of the module.
   *
   * @return {@link SwerveModuleState} of the module.
   */
  public SwerveModuleState getState()
  {
    // If in simulation and maple module sim is present, return the simulated state.
    if (RobotBase.isSimulation()) {
        if (m_config.getMapleModuleSim().isPresent()) {
            return m_config.getMapleModuleSim().get().getMeasuredState();
        }
    }
    // Otherwise return the real state.
    return new SwerveModuleState(
            m_driveMotorController.getMeasurementVelocity(),
            new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Get the {@link SwerveModulePosition} of the module.
   *
   * @return {@link SwerveModulePosition} of the module.
   */
  public SwerveModulePosition getPosition()
  {
    // If in simulation and maple module sim is present, return the simulated position.
    if (RobotBase.isSimulation()) {
        if (m_config.getMapleModuleSim().isPresent()) {
            return m_config.getMapleModuleSim().get().getModulePosition();
        }
    }
    // Otherwise return the real position.
    return new SwerveModulePosition(m_driveMotorController.getMeasurementPosition(),
            new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Update the telemetry of the module.
   */
  public void updateTelemetry()
  {
    m_driveMotorController.updateTelemetry();
    m_azimuthMotorController.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  /**
   * Update the simulation of the module.
   */
  public void simIterate()
  {
    m_driveMotorController.simIterate();
    m_azimuthMotorController.simIterate();
  }
}
