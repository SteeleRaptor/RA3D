from tkinter import *
from tkinter import filedialog
import os, re, copy, threading, time, math
import numpy as np
from ArmController import Position, Origin, MoveCommand,MoveParameters

class PrintController:
    #region init
    def __init__(self, root,armController):
        self.root = root
        self.armController = armController
        #------Important variables-------
        #MoveParameters(speed,acceleration,deceleratio,ramp,loopmode,speedtype)
        self.speed = 50 #mm/s
        self.acceleration = 5 #percent of move that is accelerating
        self.decceleration = 5 #percent of move that is decelerating
        self.ramp = 20 #minimum is 10
        #0=closed loop
        self.printOpenLoopControl = 0 #Thus closed cloop
        self.defaultPrintParameters = MoveParameters(self.speed,self.acceleration,self.decceleration,self.ramp,self.printOpenLoopControl,"m") #50mm/s print speed
        self.timeoutExtra = 60 #Extra timeout added to every timeout estimation
        self.ignoreFlags = True #ignores flags and unrecognized gcode lines and boundary check
        self.checkBoundaryTrue = True #enables/disable boundary checks

        self.plateHeight = 280 #Change bed height relative to pen
        self.dropHeight = 0.5 #mm, drop all layers by amount, z will not go negative
        #boundarys for corner calibration/setting recommended origin
        #These boundaries are for better printing
        self.YEdge = [-100,100] #These values were adjusted carefully, should not change
        self.XEdge = [300,500]

        self.hardCodePrinterSpeed = False #Will obey the print parameters and not the gcode feed rate
        self.axis5 = False #NOTE should be set to false because axis 5 implementation is incomplete
        
        #Boundaries to stop printing
        #These boundaries are to prevent dangeous movement
        self.maxBoundaryX = [270,530]
        self.maxBoundaryY = [-130,130]
        self.maxBoundaryZ = [200,900]
        self.bufferBoundary = 10 # Warning pops up if this boundary is entered from the max boundaries
        self.bedCalibrateHeight = 50 #Height moved up for calibration


        self.maxHotEndTempSetting = 300 #Max hotend temp that can be set from gcode
        self.maxBedTempSetting = 120 #Max bed temp that can be set from gcode
        
        self.backlashAngleOffset = 0 #5.55 #offset to account for backlash
        #Could add another safeguard within the temperatur controller
        #------End important variables-------
        self.defaultAngle = 90 - self.backlashAngleOffset
        #recomended Origin for move set at middle of calibration corners
        self.recommendedOriginPosition = Position((self.XEdge[0]+self.XEdge[1])/2,0,self.plateHeight,0,self.defaultAngle,0,None)
        #extract position to origin
        self.recommendedOrigin = self.recommendedOriginPosition.toOrigin()
        
        #Assume origin is at recommended origin
        self.origin = self.recommendedOrigin

        # Calibration corners
        FLCorner = Position(self.XEdge[1],self.YEdge[0],self.plateHeight,0,self.defaultAngle,0,None)
        FRCorner = Position(self.XEdge[1],self.YEdge[1],self.plateHeight,0,self.defaultAngle,0,None)
        BLCorner = Position(self.XEdge[0],self.YEdge[0],self.plateHeight,0,self.defaultAngle,0,None)
        BRCorner = Position(self.XEdge[0],self.YEdge[1],self.plateHeight,0,self.defaultAngle,0,None)

        #corners are absolute relative to the physical structure
        #but z values of calibration will update to be the same as origin if the origin is changed in the UI
        #however calibration corners z remain unchanged after initialization
        self.calibrationCorners = [FRCorner,BRCorner,BLCorner,FLCorner]
        
        #---------- Setup variables---------------
        # Variables that are just simply initialized

        # Parameters used for saving the last used coordinate information
        self.lastPos = Position(None,None,None,None,None,None,self.origin)
        self.printPos = Position(None,None,None,None,None,None,self.origin)
        
        self.lastF = 0.0
        self.lastE = 0.0
        self.feedRate = 0
        self.extrudeRate = 0
        self.currentInstruction = 0
        
        self.justMovedHome = False
        self.relativeExtrusion = True
        self.relativePositioning = False #default is abosolute positioning

        self.selectedFilepath = None
        self.gcodeLines = []
        self.teensyLines = []
        self.fileOpen = False
        self.printing = False
        self.printPaused = False
        self.cornerSweeping = False
        self.bedCalibrationInProgress = False
        self.bedCalStep = 0

        #Flag variable for errors
        self.flag = None

    
        
    #endregion init
    #region ================== Main Functions =====================

    # This is the main function that will loop when printing a file
    def printLoop(self):
        # NOTE for proper flag handling the program should repeat the last command depending on the flag, this is not been implemented
        self.checkFlag() #check flag to see if to continue printing
        
        #If the flag stopped the print, exit
        if self.printing == False or self.printPaused == True:
            self.root.printThreadStarted = False
            return
        
        message = "comment"
        #G code lines will skip past comments
        while message == "comment":
            #check if at end of file
            if self.currentInstruction > len(self.gcodeLines) - 1:
                self.root.statusPrint("End of program reached")
                self.endPrint() #Do any necessary processes to end the print
                message = ""
                break
            
            # Pull current line
            lineToConvert = self.gcodeLines[self.currentInstruction] 
            
            #Update progress bar
            self.currentInstruction += 1 # Increment currentInstruction
            self.root.progressBar["value"] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update progress bar to match
            self.root.printProgressBarHome['value'] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update home progress bar to match
            self.root.progressHomeLabel.config(text=f"{(self.currentInstruction / len(self.gcodeLines)) * 100}%")

            #Read gcode line and convert, handle rare messages inside
            message = self.interpretGcode(lineToConvert) # Convert line and updates printPos
        
        #Display gcode lines for debugging and if a lineToConvert exists
        if lineToConvert and self.root.PrintDebugMode:
            self.root.terminalPrint(f"Line: {lineToConvert}")# Print the line we're converting
        
        #region -----------Message Processing--------
        #Common commands are handled here, rare commands handled inside interpretGcode
        if message == "":
            pass
        elif message[:5] == "Error": # If the point is blank, don't try to send a command
            self.root.warningPrint("Error printing "+ message)
            if not self.ignoreFlags:
                self.pausePrint()
        # TODO: Add ability to home one axis
        elif message[:4] == "Home":
            self.root.armController.moveHome()
            #TODO check to see if home printer only happens at beginning of gcode for 5 axis printing
            #When we combine 2 gcodes the 2nd one cannot move origin
            self.justMovedHome = True #For better wrist condition after moving from home position
        elif message == "Success":
            #TODO when turn hazard encountered,
            # printer moves home than to position with normal wrist condition
            #Execute move command
            moveParameters = copy.deepcopy(self.defaultPrintParameters)
            if not self.hardCodePrinterSpeed:
                if self.feedRate != None and self.feedRate != 0:
                    moveParameters.speedType = "m"
                    #Conver to mm/s from mm/min
                    moveParameters.speed = self.feedRate / 60
            #estimate timeout
            timeout = self.root.armController.estimateMoveTime(self.lastPos,self.printPos,moveParameters.speed)
            timeout += self.timeoutExtra #extra time for communication

            #if valid position and no flag
            if self.printPos.x is not None and self.flag is None:
                if self.root.PrintDebugMode:
                    self.root.terminalPrint(f"Point: {self.printPos.GetAbsolute()}") # Print the returned point list
                
                #So that when coming from a home command it moves with optimal wrist condition
                if self.justMovedHome:
                    moveParameters.wrist = "N"
                    self.justMovedHome = False
                    self.root.armController.sendMJ(self.printPos, moveParameters=moveParameters, timeout=timeout)
                #if there is not extrusion, movement doesn't have to be a straight line
                elif self.extrudeRate == 0:
                    moveParameters.wrist = "N"
                    #self.root.armController.moveHome()#NOTE this line is for drawing tests only
                    self.root.armController.sendMJ(self.printPos, moveParameters=moveParameters, timeout=timeout)
                else:
                    #Check if hotend is at target temperature before extruding, if not set flag to stop print and display warning
                    if not self.root.temperatureController.HotendTargetReached():
                        self.flag = "Hotend not at target temperature"
                    else:
                        # Send the command to the arm, will wait for a response
                        self.root.armController.sendML(self.printPos, moveParameters=moveParameters, extrudeRate=self.extrudeRate,timeout=timeout, RelativeExtrude = self.relativeExtrusion)
        
        elif message == "Extrusion Only":
            self.root.terminalPrint(f"Extruding {self.extrudeRate} without moving")
            #Must be at hotened before extruding
            if not self.root.temperatureController.HotendTargetReached():
                self.flag = "Hotend not at target temperature"
            if self.feedRate != None and self.feedRate != 0:
                moveParameters = copy.deepcopy(self.defaultPrintParameters)
                moveParameters.speedType = "m"
                #Conver to mm/s from mm/min
                moveParameters.speed = self.feedRate / 60
            else:
                #Use default extrusion parameters rather default print parameters
                moveParameters = None
            if self.flag is None:
                self.root.armController.extrude(self.extrudeRate,moveParameters=moveParameters,timeoutMultiplier=2, RelativeExtrude = self.relativeExtrusion)
        else:
            self.root.warningPrint("Unexpected message from interpretGcode")
        #endregion message handling
        #must be last thing to do, copy last position
        self.lastPos = copy.deepcopy(self.printPos)
        #Signifies thread has ended to start next thread
        self.root.printThreadStarted = False #Do NOT return anywhere else in this function

    # Converts a GCode instruction to the instruction to send over serial
    # Manages rare gcode commands and updates the print position based on the gcode command
    def interpretGcode(self, lineToConvert):
        #Assume extruderate will not be set
        self.extrudeRate = 0

        #NOTE some of these gcode commands are a work in progress
        if lineToConvert[0] == ';': # Line is comment
            return "comment" # Don't convert
        elif lineToConvert[0:3] == "G21":
            return ""
        elif lineToConvert == "\n": # Line is newline
            return "" # Don't convert
        #Relative Extrusion
        elif lineToConvert[:3] == "M83":
            self.root.terminalPrint("Using relative extrusion")
            self.relativeExtrusion = True
            return ""
        #Absolute Extrusion
        elif lineToConvert[:3] == "M82":
            self.root.terminalPrint("Using absolute extrusion")
            self.relativeExtrusion = False
            return ""
        #Temperature control command
        #Set Hot End Temperature
        elif lineToConvert[:4] == "M104":
            self.root.terminalPrint("Setting hotend temperature...")
            sMatch = re.search(r"[sS](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            s = float(sMatch.group(1)) if sMatch else None
            if s is not None and s >= 0 and s <= self.maxHotEndTempSetting:
                self.root.temperatureController.setHotendTargetTemp(s)
                self.root.temperatureController.enableHotendControl()
            return ""
        
        #Set Bed Temperature
        elif lineToConvert[:4] == "M140":
            self.root.terminalPrint("Setting bed temperature...")
            sMatch = re.search(r"[sS](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            s = float(sMatch.group(1)) if sMatch else None
            if s is not None and s >= 0 and s <= self.maxBedTempSetting:
                self.root.temperatureController.setBedTargetTemp(s)
                self.root.temperatureController.enableBedControl()
            return ""
        
        #Wait for Hot End Temperature
        elif lineToConvert[:4] == "M109":
            self.root.terminalPrint("Waiting for hotend to reach target temperature...")
            while not self.root.temperatureController.HotendTargetReached():
                time.sleep(1) # Wait for 1 second before checking again
            self.root.terminalPrint("Hotend reached target temperature")
            return ""
        #Wait for Bed Temperature
        elif lineToConvert[:4] == "M190":
            self.root.terminalPrint("Waiting for bed to reach target temperature...")
            while not self.root.temperatureController.BedTargetReached():
                time.sleep(1) # Wait for 1 second before checking again
            self.root.terminalPrint("Bed reached target temperature")
            return ""
        elif lineToConvert[:3] == "G28": # Home the printer
            return "Home" + lineToConvert[3:]
        elif lineToConvert[:3] == "G90": # Absolute positioning
            self.relativePositioning = False
            self.root.terminalPrint("Using absolute positioning")
            return ""
        #relative positioning
        elif lineToConvert[:3] == "G91":
            self.relativePositioning = True
            self.root.terminalPrint("Using relative positioning")
            return ""
        #Zero extruder axis
        elif lineToConvert == "G92 E0\n":
            self.root.armController.zeroJ7()
        #wait
        elif lineToConvert[:2] == "G4":
            span = float(lineToConvert[3:])
            time.sleep(span/1000)
        elif lineToConvert[0:2] == "G0" or lineToConvert[0:2] == "G1": # Move (treating G0 & G1 as mostly equal)
        
            #Search for values
            xMatch = re.search(r"[xX](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            yMatch = re.search(r"[yY](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            zMatch = re.search(r"[zZ](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            fMatch = re.search(r"[fF](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
            eMatch = re.search(r"[eE](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)

            #For 5 axis printing
            if self.axis5:
                uMatch = re.search(r"[uU](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
                vMatch = re.search(r"[vV](-?(?:\d+\.?\d*|\.\d+))", lineToConvert)
                u = float(uMatch.group(1)) if uMatch else None
                v = float(vMatch.group(1)) if vMatch else None
            else:
                u = None
                v = None
            
            #Coordinates here are RELATIVE to the origin
            #Absolute positiong
            
            x = float(xMatch.group(1)) if xMatch else None
            y = float(yMatch.group(1)) if yMatch else None
            z = float(zMatch.group(1)) if zMatch else None
            #Relative positioning
            if self.relativePositioning:
                if x is not None:
                    x += self.lastPos.GetRelativeX()
                if y is not None:
                    y += self.lastPos.GetRelativeY()
                if z is not None:
                    z += self.lastPos.GetRelativeZ()


            #print("z read:", z)
            f = float(fMatch.group(1)) if fMatch else None
            e = float(eMatch.group(1)) if eMatch else None

           
            #NOTE 5 axis g code is not fully implemented
            if self.axis5:
                #For now both u and v are need for rotation
                if u == None or v == None:
                    Rx = self.lastPos.Rx
                    Ry = self.lastPos.Ry
                    Rz = self.lastPos.Rz
                else:
                    u +=-90 #may add instead of subtract
                    #TODO I think u is elevation and v azimuth, need to check
                    Rz,Ry,Rx = self.aer_to_euler_zyx(v,u,0) # 3 options for transformation

            NoMove = False #For some commands they only specify E


            if x == None and y==None and z==None:
                NoMove = True
                self.printPos=copy.deepcopy(self.lastPos)
            else:
                # If GCode instruction didn't contain a parameter, pull from last saved value
                # If instruction DID contain a parameter, offset the value to put it in the build volume    
                if x == None:
                    x = self.lastPos.GetRelativeX()
                if y == None:
                    y = self.lastPos.GetRelativeY()
                if z == None:
                    z = self.lastPos.GetRelativeZ()
                #if using 5 axis g code
                if self.axis5:
                    self.printPos.SetRelative(x,y,z,Rx,Ry,Rz)
                else:
                    self.printPos.SetRelative(x,y,z,0,self.defaultAngle,0)
            if z is not None: 
                #Adjust for change
                z = z - self.dropHeight
                z = max(z,0) #don't allow negative z values

           
            #Do not extrude if not told to
            #Handle If E or F or both are missing
            if not NoMove:
                if e == None:
                    e = 0
                else:
                    self.lastE = e
            elif e == None and f==None:
                #extrusion has to be specified for extrusion only
                return "Error no movement specified"
            elif e == None:
                self.feedRate = f
                return ""
            elif f == None:
                self.extrudeRate = e
                return "Extrusion only"

            atBoundary = False
            #If values are past max boundaries set to be at boundaries
            if self.checkBoundaryTrue:
                if self.printPos.y > self.YEdge[1]:
                    atBoundary = True
                if self.printPos.y < self.YEdge[0]:
                    atBoundary = True
                if self.printPos.x > self.XEdge[1]:
                    atBoundary = True
                if self.printPos.x < self.XEdge[0]:
                    atBoundary = True
                if atBoundary:
                    self.root.terminalPrint("Warning moving/printing at boundaries, print will continue")
            
            #G0 moves printer at max speed
            if lineToConvert[0:2] == "G0":
                self.feedRate = 5000#mm/min
            else:
                if f is not None:
                    self.feedRate = f

            #Set extrude rate of printer
            self.extrudeRate = e

            #One last check if point is in boundary else pause print
            if self.checkBoundary(self.printPos):
                return "Success"
            else:
                #NOTE this disabled with ignore flags
                if not self.checkBoundaryTrue:
                    self.pausePrint()
                self.root.warningPrint("Moving of bounds")
                return "Error moving out of bounds"
        else:
            if not self.ignoreFlags:
                return "Error unrecognized gcode line"
    
        #newLine = f"MLX{x}Y{y}Z{z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{self.root.armController.speed}Ac{self.root.armController.acceleration}Dc{self.root.armController.deceleration}Rm{self.root.armController.ramp}Rnd0WFLm000000Q0\n"
        #return newLine
        return ""
    
    #Function used when end of print is reached or when print is cancelled to end any related processes
    def endPrint(self, moveHome = True):
        self.root.temperatureController.disableHotendControl()
        self.root.temperatureController.disableBedControl()
        self.root.LEDOn = False # Turn off LED to signify print is cancelled
        self.currentInstruction=0
        self.printing = False
        self.printPaused = False #If print was paused, it is no longer paused if it is cancelled or ended
        #Move Home when complete
        self.root.armController.moveHome()

    #endregion main functions

    #region ================| GUI Buttons |===================
    #These functions will relate to the buttons in the GUI
    #TODO add busy check for any user implementation and add more flags if needed
    def selectFile(self,userSelect = True, filepath = None):
        # TODO: Needs some form of garbage collection as Python holds onto the memory allocated when opening a file
        # TODO: One possibility is utilizing "yield" command (or other methods) to only read one line at a time
        
        # This list contains valid file types
        filetypes = [
            ("GCode Files", "*.gcode"),
            ("All Files", "*.*")
        ]
        # Have user select a file
        if userSelect:
            self.selectedFilepath = filedialog.askopenfilename(filetypes=filetypes)
        else:
            self.selectedFilepath = filepath
        
        # Check if user actually selected a file
        if (self.selectedFilepath == ""):
            self.root.statusPrint("No file selected")
            self.root.selectedFileLabel.config(text="No file selected")
            self.root.selectedFileHomeLabel.config(text="Please select a file")
            self.root.textBox.config(state="normal")
            self.root.textBox.delete("1.0", END) # Clear text box
            self.root.textBox.config(state="disabled")
            self.gcodeLines = [""]
            self.fileOpen = False
            # Disable the buttons
            self.root.startPrintButton.config(state="disabled")
            self.root.stepPrintButton.config(state="disabled")
            self.root.pausePrintButton.config(state="disabled")
            self.root.cancelPrintButton.config(state="disabled")
            # Home tab buttons
            self.root.startPrintHomeButton.config(state="disabled")
            self.root.pausePrintHomeButton.config(state="disabled")
            self.root.stopPrintHomeButton.config(state="disabled")
            return
        # Change selectedFileLabel to have filename
        self.root.statusPrint(f"Selected \"{os.path.basename(self.selectedFilepath)}\"")
        self.root.selectedFileLabel.config(text=os.path.basename(self.selectedFilepath))
        self.root.selectedFileHomeLabel.config(text=os.path.basename(self.selectedFilepath))
        self.fileOpen = True
        self.currentInstruction = 0 # Reset the currentInstruction counter

        # Read all lines of the file into gcodeLines
        selectedFile = open(self.selectedFilepath, "r")
        self.gcodeLines = selectedFile.readlines()
        selectedFile.close()

        # Change the text box text to be the lines of the file
        self.root.textBox.config(state="normal") # Need to enable to modify
        self.root.textBox.delete("1.0", END) # Clear text box
        for i in range(0, len(self.gcodeLines) - 1):
            self.root.textBox.insert(END, self.gcodeLines[i])
        self.root.textBox.config(state="disabled") # Disable again to avoid user changes
        # Enable the buttons
        self.root.startPrintButton.config(state="normal")
        self.root.stepPrintButton.config(state="normal")
        self.root.pausePrintButton.config(state="normal")
        self.root.cancelPrintButton.config(state="normal")
        # Home tab buttons
        self.root.startPrintHomeButton.config(state="normal")
        self.root.pausePrintHomeButton.config(state="normal")
        self.root.stopPrintHomeButton.config(state="normal")

    def autoSelectFile(self, filepath):
        try:
            self.selectFile(userSelect=False, filepath=filepath)
            self.root.terminalPrint(f"Automatically selected file: {filepath}")
        except:
            self.root.terminalPrint(f"Failed to automatically select file: {filepath}")
    

    def startPrint(self):
        
        self.syncOrigin()#Get origin from arm controller

        #Check for problems that would prevent print from starting
        if not self.origin.checkOriginSet():
            self.root.statusPrint("Origin not set, print cancelled")
            return
        if self.root.armController.checkIfAllBusy(message="start print"):
            return
        #Warning prints
        if self.ignoreFlags:
            self.root.warningPrint("Ignore flags is enabled\nFlags will be ignored. Disable in print controller")
        if not self.checkBoundaryTrue:
            self.root.warningPrint("Boundary check disabled, arm may move dangerously")
        if self.root.coolendMode:
            self.root.warningPrint("Printing in coolend mode. Extruder should not be attached to the hotend else damage will occur.")
        #Print starts here if no problems
        self.root.LEDOn = True # Solid LED to signify print in progress, will be turned off when print is paused or finished

        #Resume print if paused
        if self.printPaused == True and self.printing == True:
            self.printPaused = False
            return #all other initialization ignored when resuming print
        
        self.currentInstruction = 0 # Reset the currentInstruction counter to start of file
        
        #Zero extruder axis used for measuring total filament used
        self.root.armController.zeroJ7()

        #Reset flag
        self.flag = None
        self.root.serialController.clearQueue() #clear queue so it is not backed up

        
        #print(self.origin.z, "test2")
        self.printPos.origin = self.origin
        self.printPos = self.origin.toPosition(angle=self.defaultAngle) #Reset print position to origin
        #Last position starts at origin
        self.lastPos = Position(self.origin.x,self.origin.y,self.origin.z,0,self.defaultAngle,0,self.origin)
        #print(self.lastPos.z)

        #Reset feedrate and extruderate
        self.lastF = 0.0
        self.lastE = 0.0
        self.printing = True
        self.root.statusPrint("Starting print...")
        self.root.printStatusHomeLabel.config(text="PRINTING...")

    def stepPrint(self):
        if self.checkIfPrinterBusy():
            self.root.terminalPrint("Cannot step print, printer busy")
        if self.root.armController.checkIfBusy():
            self.root.terminalPrint("Cannot step print, arm busy")
        #will perform one print loop only
        self.printLoop()
    
    def pausePrint(self):
        self.root.LEDOn = False # Turn off LED to signify print is paused
        self.root.terminalPrint("Pausing Print")
        self.root.printStatusHomeLabel.config(text="PAUSED...")
        self.printPaused = True

    def cancelPrint(self,moveHome = True):
        self.endPrint(moveHome=moveHome) #Do any necessary processes to end the print
        self.root.statusPrint("Print cancelled")
        self.root.printStatusHomeLabel.config(text="IDLING...")
        pass

    # Bed Calibration and sweeps ==========================

    def startPrintBedCalibration(self):
        self.flag = None #set flag to none only on start
        #If anything is busy cannot start arm
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Printer is busy, cannot start bed calibration")
            return
        self.root.serialController.clearQueue() #clear queue so it is not backed up
        self.bedCalibrationInProgress = True #flag to signal calibration
        self.bedCalStep = 1 #Start at 1st corner

        self.syncOrigin() #Sync origin with arm controller
        #Update the corner z so that match a new origin
        #The x and y stay fixed so origin can be offset from center.
        self.plateHeight = self.origin.z
        for corner in self.calibrationCorners:
            corner.z = self.origin.z
        
        self.nextBedCalibration()

    #user controlled next button
    def nextBedCalibration(self):
        #Do not check all busy because bed calibration is one of the flags
        if self.root.armController.checkIfBusy():
            self.root.statusPrint("Arm is busy cannot go to next step")
            return
        if self.bedCalibrationInProgress==False:
            self.root.statusPrint("Bed calibration not started")
        self.checkFlag()
        if self.bedCalibrationInProgress == True:
            bedCalibrationThread = threading.Thread(target=self.bedCalibrationStep, name= "Bed Calibration Thread")
            bedCalibrationThread.start()
        
    def startCornerSweep(self):
        self.flag = None #reset flag
        
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start corner sweep")
            return
        else:
            self.cornerSweeping = True
        self.root.serialController.clearQueue() #clear queue so it is not backed up
        cornerSweepThread = threading.Thread(target=self.cornerSweep, name="Corner Sweep Thread")
        cornerSweepThread.start()

    #corner sweep and move up one level at a time
    def startFullCornerSweep(self):
        self.flag = None
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start corner sweep")
            return
        else:
            self.cornerSweeping = True
        self.root.serialController.clearQueue() #clear queue so it is not backed up
        cornerSweepThread = threading.Thread(target=self.fullCornerSweep, name="Full Corner Sweep Thread")
        cornerSweepThread.start()

    #will cancel any related printing setup functions
    def cancelAny(self):
        if self.printing:
            self.cancelPrint()
        threading.Thread(target=self.endSweepOrCal, name="End Sweep or Calibration Thread").start() #On thread because move command is on there
        #just in case someone thinks this will cancel the print
        

    
    #endregion gui

    #region ===============| Other Functions |=================
    #All flag management should be done here besides raising a flag
    def checkFlag(self):
        #Assuming that pausing print is desired if there is a flag
        flag = self.flag
        if flag is not None and not self.ignoreFlags:
            if self.printing:
                self.pausePrint()
                self.root.warningPrint(f"Print paused due to error: {self.flag}")
                self.flag = None
            if self.bedCalibrationInProgress or self.cornerSweeping:
                self.endSweepOrCal()
                self.root.warningPrint(f"Bed calibration/corner sweeping paused due to error: {self.flag}")
                self.flag = None
            #Clear queue because queue tends to get clogged when encountering flags
            self.root.serialController.clearQueue()
            return flag
        elif self.flag is not None:
            #display warning print instead of pausing when ignore flag is on
            self.root.warningPrint("Flag: "+self.flag)
            self.flag = None
            #Clear queue because queue tends to get clogged when encountering flags
            self.root.serialController.clearQueue()
            return flag
        return None

    #Check if the printer is busy with printing, calibration or sweeping
    def checkIfPrinterBusy(self,message = None,display=True):
        if self.cornerSweeping or self.bedCalibrationInProgress or (self.printing and not self.printPaused):
            #if there is a message to display
            if message and display:
                self.root.terminalPrint("Cannot" + message + ", Printer busy")
            elif display:
                self.root.terminalPrint("Printer busy")
            return True
        
        return False

    def syncOrigin(self):
        self.origin = self.root.armController.origin
    #change recommened origin based on plate height
    def changeRecommendedOrigin(self, PlateHeight):
        self.recommendedOriginPosition.z = PlateHeight
        #extract position to origin
        self.recommendedOrigin = self.recommendedOriginPosition.toOrigin()
        self.root.terminalPrint(f"Recommended origin set to {self.recommendedOrigin.getOrigin()}")

    #determines whether a position is within the boundaries of the printer
    def checkBoundary(self, pos):
        # check whether position is within buffer boundary, if so print warning
        if pos.x < self.maxBoundaryX[0] or pos.x > self.maxBoundaryX[1]:
            self.root.terminalPrint("X boundary exceeded")
            print(pos.x < self.maxBoundaryX[0] + self.bufferBoundary or pos.x > self.maxBoundaryX[1] - self.bufferBoundary)
            self.root.terminalPrint(f"X at {pos.x} when exceeded")
        if pos.y < self.maxBoundaryY[0] + self.bufferBoundary or pos.y > self.maxBoundaryY[1] - self.bufferBoundary:
            self.root.terminalPrint("Y boundary exceeded")
            self.root.terminalPrint(f"Y at {pos.y} when exceeded")
        if pos.z < self.maxBoundaryZ[0] or pos.z > self.maxBoundaryZ[1] :
            self.root.terminalPrint("Z boundary exceeded")
            self.root.terminalPrint(f"Z at {pos.z} when exceeded")
        # Check if position is out of bounds, if so return false to prevent move
        if pos.x < self.maxBoundaryX[0] or pos.x > self.maxBoundaryX[1] or pos.y < self.maxBoundaryY[0] or pos.y > self.maxBoundaryY[1] or pos.z < self.maxBoundaryZ[0] or pos.z > self.maxBoundaryZ[1]:
            return False
        return True
    
    #endregion
    
    #region Calibration

    #Move to one corner and slowly descend on it
    def bedCalibrationStep(self):
        #Move home on first step
        if self.bedCalStep == 1:
            self.root.armController.moveHome()
            self.checkFlag()

        #End calibration
        if self.bedCalStep >= 5: #Is at 6 after 4 is completed so done
            self.endSweepOrCal()
            return
        #set label
        currentCornerPos = self.calibrationCorners[self.bedCalStep-1]
        self.root.cornerLabel.config(text=f"Current Corner: {self.bedCalStep}")
        self.root.cornerLabelHome.config(text=f"Current Corner: {self.bedCalStep}")

        #For better positioning move home than origin so J4 starts at 0 rather than 180
        #self.root.armController.moveHome()
        #self.root.armController.moveOrigin()

        #Move above
        posStep = copy.deepcopy(currentCornerPos)
        posStep.z += self.bedCalibrateHeight
        moveParameters = copy.deepcopy(self.defaultPrintParameters)
        moveParameters.wrist = "N"
        self.root.armController.sendMJ(posStep,moveParameters=moveParameters,timeout=self.timeoutExtra)
        self.checkFlag()
        #Move halfway
        self.root.armController.sendMJ(posStep,moveParameters=self.defaultPrintParameters,timeout=self.timeoutExtra)
        posStep.z -= self.bedCalibrateHeight/2

        #Move to corner to touch plate
        self.root.armController.sendMJ(currentCornerPos,moveParameters=self.defaultPrintParameters,timeout=self.timeoutExtra)
        self.checkFlag()
        self.root.statusPrint(f"Corner {self.bedCalStep} calibration complete")
        self.bedCalStep += 1
        

        

    # Used to sweep the corners without lifting to ensure kinematics are level to bed
    def cornerSweep(self, height=None, full=False):
        self.checkFlag 
        #Height is assumed to be the height of the calibration corners
        if height is None:
            height = self.plateHeight
        
        moveOrder = [2,3,4,1,3,2,4] #1 will always be first
        moveOrder= [x-1 for x in moveOrder] #adjust to 0 base index

        pos = copy.deepcopy(self.calibrationCorners[0])
        pos.z = height
        self.root.cornerLabel.config(text=f"Current Corner: {1}")
        moveParameters = copy.deepcopy(self.defaultPrintParameters)
        moveParameters.wrist = "N"
        #Move to first corner non linearly
        self.root.armController.sendMJ(pos,moveParameters=moveParameters, timeout=self.timeoutExtra)

        for i in moveOrder:
            time.sleep(1)
            pos = copy.deepcopy(self.calibrationCorners[i])
            pos.z = height
            self.root.cornerLabel.config(text=f"Current Corner: {i+1}")
            self.root.cornerLabelHome.config(text=f"Current Corner: {i+1}")
            if self.cornerSweeping:
                self.root.armController.sendML(pos,moveParameters=self.defaultPrintParameters, timeout=self.timeoutExtra)
            #dont continue if no longer sweeping
            else:
                return
            self.checkFlag()
            
        #End calibration
        #if not doing a fullSweep()
        if not full:
            self.endSweepOrCal()
    
    #End any corner sweep or bed calibration
    def endSweepOrCal(self, move=True):
        
        self.cornerSweeping = False
        self.bedCalibrationInProgress=False
        self.bedCalStep == 0
        self.root.cornerLabel.config(text=f"Current Corner: N/A")
        self.root.cornerLabelHome.config(text=f"Current Corner: N/A")
        #Wait until move finishes to send move home command
        
        if move:
            while self.root.armController.checkIfBusy():
                pass
            self.root.armController.moveHome()
        self.root.terminalPrint("Ended sweep or cal")
    
    #use to pause everything going on
    def pauseAll(self):
        if self.cornerSweeping or self.bedCalibrationInProgress:
            self.endSweepOrCal(move=False) #end sweep without moving
        if self.printing:
            self.pausePrint()
        

    #sweep multiple layers 20mm at a time
    def fullCornerSweep(self):
        self.root.armController.moveHome()
        self.root.armController.moveOrigin()
        heightDelta = 200
        heightStep = 20
        currentHeight = self.plateHeight
        endHeight = currentHeight + heightDelta
        while currentHeight < endHeight and self.cornerSweeping:
            self.cornerSweep(height=currentHeight,full=True)
            currentHeight += heightStep
            self.checkFlag()
