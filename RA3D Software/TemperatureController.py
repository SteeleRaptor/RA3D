
import smbus
import math
import RPi.GPIO as GPIO

class TemperatureController:
    #region init
    def __init__(self, root):
        self.root = root

        # I2C Channel to be used
        self.i2cChannel = 1
        # device address (For ADDR connected to GND)
        self.adcAddr = 0x48
        # Register addresses
        self.regConversionAddr = 0x00
        self.regConfigAddr = 0x01
        self.regLoThreshAddr = 0x02
        self.regHiThreshAddr = 0x03
        # Create the bus
        self.bus = smbus.SMBus(self.i2cChannel)

        # GPIO Pin Declarations
        self.hotendPin = 32
        self.bedPin = 33
        # GPIO Pin Initializations
        GPIO.setup(self.hotendPin, GPIO.OUT)
        GPIO.setup(self.bedPin, GPIO.OUT)        

        # Configuration variables
		#    MSB           LSB
		# x.xxx.xxx.x xxx.x.x.x.xx | Config Register division
        # Refer to section 8.1.3 of the datasheet for all of the options
        self.OS        = 0b1   # 0b1   = Start single conversion
        self.MUX       = 0b100 # 0b100 = AIN_P = AIN0 and AIN_N = GND
        self.PGA       = 0b001 # 0b001 = FSR = +/- 4.096V
        self.MODE      = 0b0   # 0b0   = Continuous-conversion mode
        self.DR        = 0b100 # 0b100 = 128 Sa/s
        self.COMP_MODE = 0b0   # 0b0   = Traditional comparator
        self.COMP_POL  = 0b0   # 0b0   = Active low
        self.COMP_LAT  = 0b0   # 0b0   = Nonlatching comparator
        self.COMP_QUE  = 0b11  # 0b11  = Disable comparator

        # Constants for temperature calculation
        self.hotendR = 9930 # Hotend voltage divider resistor value
        self.bedR    = 99200 # Bed voltage divider resistor value
        self.voltageDividerVcc = 3.3
        self.thermistorBeta = 3950 # Beta value for the thermistors (Should be the same for both)
        self.thermistorRo = 100000 # Resistance of thermistors at room temp (Should be the same for both)
        self.thermistorTo = 298.15  # Room temperature value for Ro value in Kelvin (25C)

        self.fullScaleVoltage = 4.096 # This is tied to self.PGA, if one changes, so must the other
        
        # Measurement values
        self.lastADCReading = None
        self.hotendTempCelsius = None
        self.bedTempCelsius = None
        self.measurementState = 0 # 0 = Measure hotend, 1 = Measure bed

        # Control Variables
        self.hotendTempCtrlEnabled = False # If temperature control is enabled for the hotend
        self.bedTempCtrlEnabled = False    # If temperature control is enabled for the bed
        self.hotendHeaterOn = False        # If the hotend heater is turned on or not
        self.bedHeaterOn = False           # If the bed heater is turned on or not
        self.hotendTargetTemp = 0          # Target temperature for the hotend to heat up to
        self.bedTargetTemp = 0             # Target temperature for the bed to heat up to
        self.hotendTempHardLimit = 300     # Hard limit to not allow crossing over for the hotend
        self.bedTempHardLimit = 100        # Hard limit to not allow crossing over for the bed
        self.hotendTempAdjustment = 0      # Adjustment amount to counter the interference from heater current
        self.bedTempAdjustment = 3.25      # Adjustment amount to counter the interference from heater current
        self.targetTolerance = 30          # Tolerance for target temperature in degrees Celsius. 

        #Used so that room temperature is not considered an acceptable temperature for extrusion,
        #which could cause damage to the extruder if it attempted to extrude without the hotend heating up first
        self.minExtrusionTemp = 170        # Minimum hotend temperature for extrusion of any plastic to occur
        
        # Set up the config register according to default values stated earlier
        self.setConfigReg()
    #endregion

    #region ADC Control
    def setConfigReg(self):
        # Format the data properly according to data sheet (Section 8.1.3)
        data = [0, 0]
        data[0] = (self.OS << 7) + (self.MUX << 4) + (self.PGA << 1) + self.MODE
        data[1] = (self.DR << 5) + (self.COMP_MODE << 4) + (self.COMP_POL << 3) + (self.COMP_LAT << 2) + self.COMP_QUE
        # Write to the config register
        self.bus.write_i2c_block_data(self.adcAddr, self.regConfigAddr, data)

    def selectADCChannel(self, channel, autoUpdate=True):
        # Change MUX config variable depending on user specified channel
        if channel == 0:
            self.MUX = 0b100 # A0
        elif channel == 1:
            self.MUX = 0b101 # A1
        elif channel == 2:
            self.MUX = 0b110 # A2
        elif channel == 3:
            self.MUX = 0b111 # A3
        else:
            self.MUX = 0b100 # Default to A0 if value is invalid
        # Update the config reg automatically unless user specifies otherwise
        if autoUpdate:
            self.setConfigReg()

    def readADC(self):
        # Read the most recent ADC conversion
        data = self.bus.read_i2c_block_data(self.adcAddr, self.regConversionAddr, 2)
        # Format the reading properly
        rawAdc = (data[0] << 8) + data[1]
        # Make the value negative if needed
        if rawAdc > 32768:
            rawAdc -= 65536
        # Save the last ADC reading
        self.lastADCReading = rawAdc
        # Return the reading
        return self.lastADCReading
    #endregion
    
    #region Temperature Calculations
    def calculateHotendTemp(self, rawAdc):
        # Calculate the measured voltage
        Vmeas = rawAdc / pow(2, 15) * self.fullScaleVoltage
        # Convert the voltage into a resistance
        thermR = Vmeas / (self.voltageDividerVcc - Vmeas) * self.hotendR
        # Only perform calculations if thermR is positive. Should only be negative if the thermistor isn't working at all
        if thermR > 0:
            # Calculate the temperature from the resistance
            hotendTempKelvin = 1 / ((1 / self.thermistorTo) + (1 / self.thermistorBeta) * math.log(thermR / self.thermistorRo))
            # Convert to Celsius and save
            self.hotendTempCelsius = hotendTempKelvin - 273.15
            # Handle the interference when the heater is on
            if (self.hotendHeaterOn):
                self.hotendTempCelsius = self.hotendTempCelsius + self.hotendTempAdjustment
        else:
            # TODO: Possibly disable heaters if this happens
            self.root.statusPrint("Error: hotend thermR is negative")
        # Return value
        return self.hotendTempCelsius

    def calculateBedTemp(self, rawAdc):
        # Calculate the measured voltage
        Vmeas = rawAdc / pow(2, 15) * self.fullScaleVoltage
        # Convert the voltage into a resistance
        thermR = Vmeas / (self.voltageDividerVcc - Vmeas) * self.bedR
        # Only perform calculations if thermR is positive. Should only be negative if the thermistor isn't working at all
        if thermR > 0:
            # Calculate the temperature from the resistance
            bedTempKelvin = 1 / ((1 / self.thermistorTo) + (1 / self.thermistorBeta) * math.log(thermR / self.thermistorRo))
            # Convert to Celsius and save
            self.bedTempCelsius = bedTempKelvin - 273.15
            # Handle the interference when the heater is on
            if (self.bedHeaterOn):
                self.bedTempCelsius = self.bedTempCelsius + self.bedTempAdjustment
        else:
            # TODO: Possibly disable heaters if this happens
            self.root.statusPrint("Error: bed thermR is negative")
        # Return value
        return self.bedTempCelsius
    #endregion

    #region Update
    def updateTemp(self):

        # Alternates per call which thermistor to measure to not overwhelm the ADC
        if (self.measurementState == 0):
                # Handle reading and calculating hotend temperature
                adcVal = self.readADC()
                self.calculateHotendTemp(adcVal)
                self.root.hotendActual.config(text=f"{round(self.hotendTempCelsius, 2)}°C")
                # Change the ADC channel to A1 (Bed)
                self.selectADCChannel(1)
                self.measurementState = 1 # Toggle measurement state
        else:
                # Handle reading and calculating bed temperature
                adcVal = self.readADC()
                self.calculateBedTemp(adcVal)
                self.root.bedActual.config(text=f"{round(self.bedTempCelsius, 2)}°C")
                # Change the ADC channel to A0 (Hotend)
                self.selectADCChannel(0)
                self.measurementState = 0 # Toggle measurement state
        
        # Temperature control
        if (self.hotendTempCtrlEnabled):
            # Basic on/off implementation
            if (self.hotendTempCelsius > self.hotendTempHardLimit):
                # If the hotend measurement is above the hard limit, force heater off and show error
                self.root.warningPrint("Hotend exceeded hard limit")
                self.disableHotendControl() # Disable hotend control for safety
                self.hotendHeaterOn = False
                GPIO.output(self.hotendPin, GPIO.LOW)
            elif (self.hotendTempCelsius < self.hotendTargetTemp):
                # Turn the hotend heater on
                self.hotendHeaterOn = True
                GPIO.output(self.hotendPin, GPIO.HIGH)
            else:
                # Turn the hotend heater off
                self.hotendHeaterOn = False
                GPIO.output(self.hotendPin, GPIO.LOW)
        else:
            # Drive the hotend signal low in case it is on when control flag disabled
            GPIO.output(self.hotendPin, GPIO.LOW)

        if (self.bedTempCtrlEnabled):
            # Basic on/off implementation
            if (self.bedTempCelsius > self.bedTempHardLimit):
                # If the bed temp is above the hard limit, force heater off and show error
                self.root.warningPrint("Bed exceeded hard limit")
                self.disableBedControl() # Disable bed control for safety
                self.bedHeaterOn = False
                GPIO.output(self.bedPin, GPIO.LOW)
            if (self.bedTempCelsius < self.bedTargetTemp):
                # Turn the bed heater on
                self.bedHeaterOn = True
                GPIO.output(self.bedPin, GPIO.HIGH)
            else:
                # Turn the bed heater off
                self.bedHeaterOn = False
                GPIO.output(self.bedPin, GPIO.LOW)
        else:
            # Drive the bed signal low in case it is on when control flag disabled
            GPIO.output(self.bedPin, GPIO.LOW)

    #endregion

    #region Temperature Control
    #toggle control for UI
    def toggleControl(self, heater):
        if (heater == "hotend"):
            if (self.hotendTempCtrlEnabled):
                self.disableHotendControl()
            else:
                self.enableHotendControl()
                #TODO: Temporarily just set target to whatever is in target box
                self.setHotendTargetTemp(float(self.root.hotendTarget.get()))
        elif (heater == "bed"):
            if (self.bedTempCtrlEnabled):
                self.disableBedControl()
                
            else:
                self.enableBedControl()
                #TODO: Temporarily just set target to whatever is in target box
                self.setBedTargetTemp(float(self.root.bedTarget.get()))

    # Used for setting a target temperature for the hotend
    def setHotendTargetTemp(self, targetTemp):
        if (targetTemp < 0):
            self.root.terminalPrint("Provided target temperature is less than zero (hotend)")
            return
        if (targetTemp > self.hotendTempHardLimit):
            self.root.warningPrint(f"Hotend temperature attempted to be set higher than hard limit (targetTemp={targetTemp})")
            return
        self.hotendTargetTemp = targetTemp

    # Used for setting a target temperature for the bed
    def setBedTargetTemp(self, targetTemp):
        if (targetTemp < 0):
            self.root.terminalPrint("Provided target temperature is less than zero (bed)")
        if (targetTemp > self.bedTempHardLimit):
            self.root.warningPrint(f"Bed temperature attempted to be set higher than hard limit (targetTemp={targetTemp})")
        self.bedTargetTemp = targetTemp

    # Enables temperature control for the hotend
    def enableHotendControl(self):
        self.hotendTempCtrlEnabled = True
        self.root.hotendCtrlButton.config(relief="ridge")
        self.root.terminalPrint("Hotend heater control enabled")

    # Disables temperature control for the hotend
    def disableHotendControl(self):
        self.hotendTempCtrlEnabled = False
        self.root.hotendCtrlButton.config(relief="raised")
        
        self.root.terminalPrint("Hotend heater control disabled")

    # Enables temperature control for the bed
    def enableBedControl(self):
        self.bedTempCtrlEnabled = True
        self.root.bedCtrlButton.config(relief="ridge")
        self.root.terminalPrint("Bed heater control enabled")

    # Disables temperature control for the bed
    def disableBedControl(self):
        self.bedTempCtrlEnabled = False
        self.root.bedCtrlButton.config(relief="raised")
        self.root.terminalPrint("Bed heater control disabled")
    
    #endregion

    #region Printer Commands
    # These functions are used for the print controller to check if target temperatures have been reached for the M109 and M190 gcode commands
    def HotendTargetReached(self):
        if self.hotendTempCelsius > self.minExtrusionTemp:
            return self.hotendTempCelsius >= self.hotendTargetTemp-self.targetTolerance
        else:
            return False
    def BedTargetReached(self):
        return self.bedTempCelsius >= self.bedTargetTemp-self.targetTolerance
    
    #endregion