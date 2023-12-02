import wpilib
import wpilib.drive
import wpimath.controller
import ctre
import navx
import numpy as np
import wpilib.interfaces
import photonvision
from wpilib import SmartDashboard
import math


seconds = 0

c_ArmLiftTotalValue = 4096*.5       # Constant for max value ArmLift encoder should ever go to
c_ArmExtendMaxValue = 100       # Constant for max value ArmExtend encoder should ever go to
c_gyroMountAngle = 45           # Constant for what angle the gyro is mounted at
balanceMode = False

c_ticksPerDegree = 4096 * 3 / 360   # Constant for converting encoder values to degrees
c_measuredHorizontalArmPosition = -2233 # The position measured when the arm is horizontal
c_gravityPower = 1                  # The power to the motor required to hold the arm straight at horizontal position

class Robot(wpilib.TimedRobot):
    
    def __init__(self, period: seconds = 0.02) -> None:
        super().__init__(period)

        # Camera
        wpilib.CameraServer.launch()

        # Create four motors for driving, put them in a list for PID control in autonomouse
        self.MotorFL = ctre.WPI_TalonSRX(5)
        self.MotorFL.setInverted(True)
        self.MotorFR = ctre.WPI_TalonSRX(3)
        self.MotorBL = ctre.WPI_TalonSRX(0)
        self.MotorBL.setInverted(True)
        self.MotorBR = ctre.WPI_TalonSRX(6)
        self.DriveMotors = [self.MotorFL, self.MotorFR, self.MotorBL, self.MotorBR]

        # Create two motors to control the arms
        self.ArmLift = ctre.WPI_TalonSRX(1)
        self.ArmLift.setInverted(True)
        self.ArmExtend = ctre.WPI_TalonSRX(4)

        # Differential Drives to control wheel motors in teleop
        self.frontDrive = wpilib.drive.DifferentialDrive(self.MotorFL, self.MotorFR)
        self.backDrive = wpilib.drive.DifferentialDrive(self.MotorBL, self.MotorBR)

        # Create Encoders and set distance per pulse for Arm extension and Arm Lift
        self.ArmExtendEncoder = wpilib.Encoder(0, 1)
        self.ArmExtendEncoder.setDistancePerPulse(180/4)    # 180 distance per 4 pulses

        # Joysticks
        self.DriveStick = wpilib.Joystick(1)
        self.HelperStick = wpilib.XboxController(4)

        # PID controllers for arm lift and arm extension
        self.PIDArmLift = wpimath.controller.PIDController(.0025, 0, 0)
        self.PIDArmLift.setTolerance(5)
        self.PIDArmExtend = wpimath.controller.PIDController(.0625, 0, 0)
        self.PIDArmExtend.setTolerance(2)

        # PID controller for auto targetting
        self.autoTargetPID = wpimath.controller.PIDController(1, 0, 1)

        # Double solenoid for arm grabbing
        self.ArmGrab = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)

        # PID controller for balancing the robot
        self.PIDBalance = wpimath.controller.PIDController(1, 0, 1)
        self.PIDBalance.setTolerance(1)

        # navx gyro, mounted at an angle on actual robot
        self.gyro = navx.AHRS(wpilib._wpilib.SPI.Port(0))

        # timer for timing, and getting delta time
        self.timer = wpilib.Timer()

        # variable to keep track of arm
        self.ArmOpened = False #kForward to close, kReverse to open

        # constants
        self.liftTargetAngle = 0        # Target angle for Arm lift
        self.extendTargetAngle = 0      # target angle for Arm extend
        self.previousTime = 0           # variable for use in getting delta time

        # photonvision
        self.photonCamera = photonvision.PhotonCamera("rock")

        self.autonomousePhaseNum = 1    # Variable to track autonomouse phase


    def DeltaTime(self):
        # function to get delta time
        return self.timer.get() - self.previousTime
    

    def disabledInit(self) -> None:
        self.HelperStick.setRumble(wpilib.interfaces.GenericHID.RumbleType.kLeftRumble, 0)

    
    def getArmLiftEncoderValue(self):
        # function to get clamped values of arm lift encoder (for when the arm lift encoders return values to high or too low)
        return float(np.clip(self.ArmLift.getSelectedSensorPosition(), 0, c_ArmLiftTotalValue))
    
    
    def getArmExtendEncoderValue(self):
        # same as previous, but for arm extend
        return float(np.clip(self.ArmExtend.get(), 0, c_ArmExtendMaxValue))
    

    def autonomousInit(self) -> None:
        # reset all used variables
        self.timer.reset()
        self.timer.start()
        self.ArmLift.setSelectedSensorPosition(0)
        self.ArmExtendEncoder.reset()
        self.autonomousePhaseNum = 1
        self.ArmOpened = False
        self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)


    def autonomousPeriodic(self) -> None:
        if balanceMode:
            # phase 1, lift the arm
            if self.autonomousePhaseNum == 1:
                self.ArmLift.set(self.PIDArmLift.calculate(self.getArmLiftEncoderValue, 470))
                if self.PIDArmLift.atSetpoint(self.getArmLiftEncoderValue, 470) == True:
                    self.autonomousePhaseNum == 2
            # phase 2, extend the arm
            if self.autonomousePhaseNum == 2:
                self.ArmExtend.set(self.PIDArmExtend.calculate(self.getArmExtendEncoderValue, 100))
                if self.PIDArmLift.atSetpoint(self.getArmExtendEncoderValue, 100):
                    self.timer.reset()
                    self.timer.start()
                    self.autonomousePhaseNum == 3
            # phase 3, open arm + close arm
            if self.autonomousePhaseNum == 3:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kReverse)
                if self.timer.get() > 2:
                    self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)
                    self.autonomousePhaseNum = 4
            # phase 4, de-extend the arm
            if self.autonomousePhaseNum == 4:
                self.ArmExtend.set(self.PIDArmExtend.calculate(self.getArmExtendEncoderValue, 0))
                if self.PIDArmLift.atSetpoint(self.getArmExtendEncoderValue, 0):
                    self.autonomousePhaseNum == 5
            # stage 5, lower the arm
            if self.autonomousePhaseNum == 5:
                self.ArmLift.set(self.PIDArmLift.calculate(self.getArmLiftEncoderValue, 0))
                if self.PIDArmLift.atSetpoint(self.getArmLiftEncoderValue, 0) == True:
                    self.timer.reset()
                    self.timer.start()
                    self.autonomousePhaseNum == 6
            # stage 6, move backwards untill out of comunity zone, or hit the seesaw
            if self.autonomousePhaseNum == 6:
                for obj in self.DriveMotors:
                    obj.set(-.75)
                if self.gyro.getPitch < c_gyroMountAngle - 5:
                    self.autonomousePhaseNum = 7
                if self.timer.get() > 8:
                    for obj in self.DriveMotors:
                        obj.set(0)
                        self.autonomousePhaseNum = 100
            # if hit the seesaw, set speed to 1/4 untill robot reaches the top of the seesaw
            if self.autonomousePhaseNum == 7:
                for obj in self.DriveMotors:
                    obj.set(-.25)
                if self.gyro.getPitch > c_gyroMountAngle - 10:
                    self.autonomousePhaseNum == 8
            # at top of seesaw, control with PID to auto balance
            if self.autonomousePhaseNum == 8:
                for obj in self.DriveMotors:
                    obj.set(self.PIDBalance.calculate(self.gyro.getPitch, c_gyroMountAngle))
        else:
            data, hasResult = self.photonResultGet()
            if self.autonomousePhaseNum == 1:
                if hasResult == True:
                    self.frontDrive.arcadeDrive(.5, self.autoTargetPID.calculate(data.getYaw(), 0))
                    self.backDrive.arcadeDrive(.5, self.autoTargetPID.calculate(data.getYaw(), 0))
                    if data.getYaw() == 0 and data.getPitch() < -5:
                        self.autonomousePhaseNum = 2
            if self.autonomousePhaseNum == 2:
                self.frontDrive.arcadeDrive(0, 0)
                self.backDrive.arcadeDrive(0, 0)
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kReverse)


    def photonResultGet(self):
        photonData = self.photonCamera.getLatestResult()
        photonResult = photonData.getBestTarget()
        return photonResult, photonData.hasTargets()



    def teleopInit(self) -> None:
        # reset used variables
        self.previousTime = 0
        self.timer.reset()
        self.timer.start()
        self.liftTargetAngle = 0
        self.ArmLift.setSelectedSensorPosition(0)
        self.ArmExtendEncoder.reset()
        self.ArmOpened = False
        self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)
        self.ArmExtendEncoder.reset()
        self.HelperStick.setRumble(wpilib.interfaces.GenericHID.RumbleType.kLeftRumble, .5)


    def teleopPeriodic(self) -> None:
        # get x and z angles, and add together
        t_zangle = self.DriveStick.getRawAxis(2)
        t_xangle = self.DriveStick.getRawAxis(0)
        t_finalangle = t_zangle/2 + t_xangle/2
        
        # get y and w values, multiply together for forward/reverse/speed control, and pass the previous values into arcade drive
        self.frontDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)
        self.backDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)

        # change arm lift target angle based on joystick input + clamp the value
        self.liftTargetAngle -= (-self.HelperStick.getRawAxis(1) * c_ArmLiftTotalValue * self.DeltaTime())
        self.liftTargetAngle = np.clip(self.liftTargetAngle, -c_ArmLiftTotalValue, 0)
        SmartDashboard.putNumber("TargetAngle", self.liftTargetAngle)

        currentpos = self.ArmLift.getSelectedSensorPosition()
        degrees = (currentpos - c_measuredHorizontalArmPosition) / c_ticksPerDegree
        radians = math.radians(degrees)
        cosineScale = math.cos(radians)

        SmartDashboard.putNumber("CurrentAngle", self.ArmLift.getSelectedSensorPosition())
        outputValue = np.clip(self.PIDArmLift.calculate(self.ArmLift.getSelectedSensorPosition(), self.liftTargetAngle), -1, 1)
        SmartDashboard.putNumber("OutputValue", outputValue)
        SmartDashboard.putNumber("CosineScale", cosineScale)
        self.ArmLift.set(ctre.ControlMode.PercentOutput, -outputValue + .63*cosineScale)

        SmartDashboard.putNumber("ArmOutputTesting", self.ArmLift.getMotorOutputVoltage())

        # self.ArmLift.set(ctre.ControlMode.MotionMagic, self.liftTargetAngle, ctre.DemandType.Neutral, 0)   # c_gravityPower*cosineScale

        # if x button pressed, reverse direction of solenoid
        if self.HelperStick.getXButtonPressed():
            if self.ArmOpened:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)
                self.ArmOpened = False
            else:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.ArmOpened = True

        # 0 - -11.25
        SmartDashboard.putNumber("Encoder Value", self.ArmExtendEncoder.get())
        RightJoystick = self.HelperStick.getRawAxis(5)
        if RightJoystick > .05 or RightJoystick < -.05:
            RightJoystick = RightJoystick
        else:
            RightJoystick = 0
        if self.ArmExtendEncoder.get() <= 0 or self.ArmExtendEncoder.get() >= -11.25:
            if RightJoystick != 0:
                self.ArmExtend.set(ctre.ControlMode.PercentOutput, (RightJoystick/abs(RightJoystick))/2)
            else:
                self.ArmExtend.set(ctre.ControlMode.PercentOutput, 0)
        else:
            self.ArmExtend.set(ctre.ControlMode.PercentOutput, 0)

        # set previousTime variable to the current time, so that delta time can be gotten on the next loop
        self.previousTime = self.timer.get()

    def testInit(self) -> None:
        self.ArmExtendEncoder.reset()

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)