package yams.exceptions;

public class SwerveModuleConfigurationException extends RuntimeException {
    /**
     * SwerveDrive configuration exception.
     *
     * @param message        Message to display.
     * @param result         Result of the configuration.
     * @param remedyFunction Remedy function to use.
     */
    public SwerveModuleConfigurationException(String message, String result, String remedyFunction)
    {
        super(message + "!\n" + result + "\nPlease use SwerveModuleConfig." + remedyFunction + " to fix this error.");
    }
}
