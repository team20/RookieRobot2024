# DangerZone Branch

I, Jonah Morgan, made these changes and think that they make our codebase cleaner. I removed exactly 312 lines of both used and unused code. I think this code *should* behave the same as the code in the GettingStarted branch. Don't quote me on that.

Here is a list of changes:
- I inlined all commands
- I removed singletons
- I removed some of the getSomething methods
- I removed the empty PneumaticsSubsystem
- I removed DriveConstants.kSteerPeriod because it was set to the default value
- I removed DriveConstants.kIz and DriveConstants.kFF because they weren't used anywhere
- I removed DriveConstants.kI and DriveConstants.kD because the programmers told me it is dangerous to set them to anything other than 0
- I added DriveConstants.kSwerveAngleTolerance because it was a magic number
- I changed SwerveConstants.wheelDiameter from 0.1016 to the more accurate Units.inchesToMeters(4);
- I simplified the mathematical expression used to set SwerveConstants.kTicksToMeters
- I added new methods to SwerveModule
- I changed the return type of SwerveModule.getDriveEncoderPosition from double to the more descriptive Measure&lt;Distance&gt;
- I changed the return type of SwerveModule.getSteerAngle from double to the more descriptive Rotation2d
- I remove the second navX reset from the DriveSubsystem constructor
- I moved the SwerveDriveKinematics into the DriveSubsystem
- I made the DriveSubsystem.autoAngleCommand wait for every swerve module to finish
