from tkinter import *
from tkinter import filedialog
import os, re, copy, threading, time, math
import numpy as np
from ArmController import Position, Origin, MoveCommand,MoveParameters

class PrintController:
    #region init
    def __init__(self, root):
        self.root = root

        #------Important variables-------
        #MoveParameters(speed,acceleration,deceleratio,ramp,loopmode,speedtype)
        self.defaultPrintParameters = MoveParameters(50,5,5,15,0,"m") #20mm/s print speed
        
        self.ignoreflags = False #ignores flags and unrecognized gcode lines and boundary check
        self.checkBoundaryTrue = True #enables/disable boundary checks

        self.plateHeight = 314 #Change bed height relitive to pen
        #boundarys for corner calibration/setting recommended origin
        YEdge = [-80,80]
        XEdge = [250,400]

        self.axis5 = False #NOTE should be set to false because axis 5 implementation is incomplete
        #TODO These might be changed to reflect xedge, yedge
        #Boundaries to stop printing
        #Placeholder boundaries, should be adjusted
        self.maxBoundaryX = [200,500]
        self.maxBoundaryY = [-130,130]
        self.maxBoundaryZ = [290,800]
        self.bufferBoundary = 10 # Warning pops up if this boundary is entered from the max boundaries
        self.bedCalibrateHeight = 50 #Height moved up for calibration

        #------End important variables-------

        #recomended Origin for move set at middle of calibration corners
        self.recommendedOriginPosition = Position((XEdge[0]+XEdge[1])/2,0,self.plateHeight,0,90,0,None)
        #extract position to origin
        self.recommendedOrigin = self.recommendedOriginPosition.toOrigin()
        
        #Assume origin is at recommended origin
        self.origin = self.recommendedOrigin

        # Calibration corners
        FLCorner = Position(XEdge[1],YEdge[0],self.plateHeight,0,90,0,None)
        FRCorner = Position(XEdge[1],YEdge[1],self.plateHeight,0,90,0,None)
        BLCorner = Position(XEdge[0],YEdge[0],self.plateHeight,0,90,0,None)
        BRCorner = Position(XEdge[0],YEdge[1],self.plateHeight,0,90,0,None)

        #corners are absolute relative to the physical structure
        #but z values will update to be the same as origin if the origin is changed in the UI
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

        self.selectedFilepath = None
        self.gcodeLines = []
        self.teensyLines = []
        self.fileOpen = False
        self.printing = False
        self.printPaused = False
        self.cornerSweeping = False
        self.bedCalibration = False
        self.bedCalStep = 0

        #Flag variable for errors
        self.flag = None
    
        
    #endregion init
    #region ================== Main Functions =====================

    # This is the main function that will loop when printing a file
    def printLoop(self):

        self.checkFlag() #check flag to see if to continue printing
        
        #If the flag stopped the print, exit
        if self.printing == False:
            self.root.printThreadStarted = False
            return
        
        message = "comment"
        #G code lines will skip past comments
        while message == "comment":
            #check if at end of file
            if self.currentInstruction > len(self.gcodeLines) - 1:
                self.root.statusPrint("End of program reached")
                self.currentInstruction = 0
                self.printing = False
                #Move Home when complete
                self.root.armController.moveHome()
                message = ""
                break
            
            # Pull current line
            lineToConvert = self.gcodeLines[self.currentInstruction] 
            
            #Update progress bar
            self.currentInstruction += 1 # Increment currentInstruction
            self.root.progressBar["value"] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update progress bar to match
            
            #Read gcode line and convert, handle rare messages inside
            message = self.gcodeToTeensy(lineToConvert) # Convert line and updates printPos
            
        self.root.terminalPrint(f"Line: {lineToConvert}")# Print the line we're converting
        
        #region -----------Message Processing--------
        #Common commands are handled here, rare commands handled inside gcodeToTeensy
        if message == "":
            pass
        elif message[:5] == "Error": # If the point is blank, don't try to send a command
            self.root.warningPrint("Pausing print due to gcode"+message)
            self.pausePrint()
        # TODO: Add waiting for temperature to heat up from M104/M109 commands
        # TODO: Add ability to home one axis
        elif message[:4] == "Home":
            self.root.armController.moveHome()
            #TODO check to see if home printer only happens at beginning of gcode
            #When we combine 2 gcodes the 2nd one cannot move origin
            #self.root.armController.moveOrigin() #Use normal wrist condition
            self.justMovedHome = True
            #move
        elif message == "Success":
            #TODO add method that on each layer or when turn hazard encountered,
            # printer moves home than to position with normal wrist condition
            #Execute move command
            moveParameters = copy.deepcopy(self.defaultPrintParameters)
            if self.feedRate != None and self.feedRate != 0:
                #moveParameters.speedType = "m"
                #Conver to mm/s from mm/min
                moveParameters.speed = self.feedRate / 60
            #estimate timeout
            timeout = self.root.armController.estimateMoveTime(self.lastPos,self.printPos,moveParameters.speed)
            timeout += 10 #extra time for communication

            #if valid position and no flag
            if self.printPos.x is not None and self.flag is None:
                self.root.terminalPrint(f"Point: {self.printPos.GetAbsolute()}") # Print the returned point list
                
                #So that when coming from a home command it moves with optimal wrist condition
                if self.justMovedHome:
                    moveParameters.wrist = "N"
                    self.justMovedHome = False
                    self.root.armController.sendMJ(self.printPos, moveParameters=moveParameters, timeout=timeout)
                else:
                    # Send the command to the arm, will wait for a response
                    self.root.armController.sendML(self.printPos, moveParameters=moveParameters, extrudeRate=self.extrudeRate,timeout=timeout, RelativeExtrude = self.relativeExtrusion)
            
            #must be last thing to do, copy last position
            self.lastPos = copy.deepcopy(self.printPos)
        else:
            print("Unexpected message from gcodeToTeensy")
        #endregion message handling
        #Signifies thread has ended to start next thread
        self.root.printThreadStarted = False #Do NOT return anywhere else in this function

    # Converts a GCode instruction to the instruction to send over serial
    def gcodeToTeensy(self, lineToConvert):
        #Assume feedrate and extruderate will not be set
        self.feedRate = 0
        self.extrudeRate = 0

        #NOTE some of these gcode commands are a work in progress
        if lineToConvert[0] == ';': # Line is comment
            return "comment" # Don't convert
        elif lineToConvert[0:3] == "G21":
            return ""
        elif lineToConvert == "\n": # Line is newline
            return "" # Don't convert
        elif lineToConvert == "M83":
            self.root.terminalPrint("Using relative Extrusion")
            self.relativeExtrusion = True
            return ""
        elif lineToConvert == "M82":
            return "Error absolute extrusion not supported"
        #Temperature control command
        elif lineToConvert[:4] == "M104":
            return ""
        elif lineToConvert == "M109":
            return ""
        # Actual instructions to convert
        elif lineToConvert[0:3] == "G28": # Home the printer
            return "Home" + lineToConvert[3:]
        elif lineToConvert[0:3] == "G90": # Absolute positioning
            self.root.terminalPrint("Using absolute positioning")
            # TODO: This needs handling or removal
            return ""
        elif lineToConvert[0:3] == "G91":
            return "Error relative positioning not supported"
        elif lineToConvert == "G92 E0\n":
            self.root.armController.zeroJ7()
        #wait
        elif lineToConvert[:2] == "G4":
            span = float(lineToConvert[3:])
            time.sleep(span/1000)
        elif lineToConvert[0:2] == "G0" or lineToConvert[0:2] == "G1": # Move (treating G0 & G1 as mostly equal)
        
            #Search for values
            xMatch = re.search(r"[xX](-?\d+\.?\d*)", lineToConvert)
            yMatch = re.search(r"[yY](-?\d+\.?\d*)", lineToConvert)
            zMatch = re.search(r"[zZ](-?\d+\.?\d*)", lineToConvert)
            fMatch = re.search(r"[fF](-?\d+\.?\d*)", lineToConvert)
            eMatch = re.search(r"[eE](-?\d+\.?\d*)", lineToConvert)

            #For 5 axis printing
            if self.axis5:
                uMatch = re.search(r"[uU](-?\d+\.?\d*)", lineToConvert)
                vMatch = re.search(r"[vV](-?\d+\.?\d*)", lineToConvert)
                u = float(uMatch.group(1)) if uMatch else None
                v = float(vMatch.group(1)) if vMatch else None
            else:
                u = None
                v = None
            
            #Coordinates here are RELATIVE
            x = float(xMatch.group(1)) if xMatch else None
            y = float(yMatch.group(1)) if yMatch else None
            z = float(zMatch.group(1)) if zMatch else None
            #print("z read:", z)
            f = float(fMatch.group(1)) if fMatch else None
            e = float(eMatch.group(1)) if eMatch else None

            # If GCode instruction didn't contain a parameter, pull from last saved value
            # If instruction DID contain a parameter, offset the value to put it in the build volume
            if x == None:
                x = self.lastPos.GetRelativeX()
            if y == None:
                y = self.lastPos.GetRelativeY()
            if z == None:
                z = self.lastPos.GetRelativeZ()
           
            #get last feedrate if missing feedrate
            if f == None:
                f = self.lastF
            else:
                self.lastF = f
            
            #Do not extrude if not told to
            if e == None:
                e = 0
            else:
                self.lastE = e
            
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
            
            #TODO Add rotation transition in teensy if necessary
            #if using 5 axis g code
            if self.axis5:
                self.printPos.SetRelative(x,y,z,Rx,Ry,Rz)
            else:
                self.printPos.SetRelative(x,y,z,0,90,0)

            atBoundary = False
            #If values are part boundaries set to be at boundaries
            if self.checkBoundaryTrue:
                if self.printPos.y > self.maxBoundaryY[1]:
                    self.printPos.y = self.maxBoundaryY[1]-1
                    atBoundary = True
                if self.printPos.y < self.maxBoundaryY[0]:
                    self.printPos.y = self.maxBoundaryY[0]+1
                    atBoundary = True
                if self.printPos.x > self.maxBoundaryX[1]:
                    self.printPos.x = self.maxBoundaryX[1]-1
                    atBoundary = True
                if self.printPos.x < self.maxBoundaryX[0]:
                    self.printPos.x = self.maxBoundaryX[0]+1
                    atBoundary = True
                if atBoundary:
                    self.root.terminalPrint("Warning moving/printing at boundaries, print will continue")
            
            #G0 moves printer at max speed
            if lineToConvert[0:2] == "G0":
                self.feedRate = 5000#mm/min
            else:
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
            pass
            if not self.ignoreflags:
                return "Error unrecognized gcode line"
    
        #newLine = f"MLX{x}Y{y}Z{z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{self.root.armController.speed}Ac{self.root.armController.acceleration}Dc{self.root.armController.deceleration}Rm{self.root.armController.ramp}Rnd0WFLm000000Q0\n"
        #return newLine
        return ""

    #This may be redundant now if loop skips comments
    def findStartBlock(self):
        count = 0
        for line in self.gcodeLines:
            if line.strip() == "; EXECUTABLE_BLOCK_START":
                self.currentInstruction = count
                return
            count +=1
        print("No start line found")

    #endregion main functions

    #region ================| GUI Buttons |===================
    #These functions will relate to the buttons in the GUI
    #TODO add busy check for any user implementation and add more flags if needed
    def selectFile(self):
        # TODO: Needs some form of garbage collection as Python holds onto the memory allocated when opening a file
        # TODO: One possibility is utilizing "yield" command (or other methods) to only read one line at a time
        
        # This list contains valid file types
        filetypes = [
            ("GCode Files", "*.gcode"),
            ("All Files", "*.*")
        ]
        # Have user select a file
        self.selectedFilepath = filedialog.askopenfilename(filetypes=filetypes)
        # Check if user actually selected a file
        if (self.selectedFilepath == ""):
            self.root.statusPrint("No file selected")
            self.root.selectedFileLabel.config(text="No file selected")
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
            return
        # Change selectedFileLabel to have filename
        self.root.statusPrint(f"Selected \"{os.path.basename(self.selectedFilepath)}\"")
        self.root.selectedFileLabel.config(text=os.path.basename(self.selectedFilepath))
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

    def startPrint(self):
        self.syncOrigin()#Get origin from arm controller
        if not self.origin.checkOriginSet():
            self.root.statusPrint("Origin not set, print cancelled")
            return
        if self.root.armController.checkIfAllBusy(message="start print"):
            return
        if self.ignoreflags:
            self.root.warningPrint("Ignore flags is enabled\nflags will be ignored disable in print controller")
        if not self.checkBoundaryTrue:
            self.root.warningPrint("Boundary check disabled, arm may move dangerously")
        #Resume print if paused
        if self.printPaused == True and self.printing == True:
            self.printPaused = False
            return
        
        #Zero extruder axis used for measuring total filament used
        self.root.armController.zeroJ7()

        #Reset flag
        self.flag = None

        # When starting print, reset the "last*" parameters
        print(self.origin.z, "test2")
        self.printPos.origin = self.origin
        self.printPos = self.origin.toPosition()
        #Last position starts at origin
        self.lastPos = Position(self.origin.x,self.origin.y,self.origin.z,0,90,0,self.origin)
        print(self.lastPos.z)
        self.lastF = 0.0
        self.lastE = 0.0

        if not self.printPaused:
            self.findStartBlock()#Find where the gcode insturctions actually start
        self.printing = True

    def stepPrint(self):
        if self.checkIfPrinterBusy():
            self.root.terminalPrint("Cannot step print, printer busy")
        if self.root.armController.checkIfBusy():
            self.root.terminalPrint("Cannot step print, arm busy")
        #will perform one print loop only
        self.printLoop()
    
    def checkFlag(self):
        #Assuming that pausing print is desired if there is a flag
        if self.flag is not None and not self.ignoreflags:
            self.pausePrint()
            self.root.warningPrint(f"Print paused due to error: {self.flag}")
            return self.flag
        return None

    #Check if the printer is busy with printing, calibration or sweeping
    def checkIfPrinterBusy(self,message = None):
        if self.printing or self.cornerSweeping or self.bedCalibration:
            #if there is a message to display
            if message:
                self.root.terminalPrint("Cannot" + message + ", Printer busy")
            else:
                self.root.terminalPrint("Printer busy")
            return True
        
        return False

    def pausePrint(self):
        self.root.terminalPrint("Pausing Print")
        self.printPaused = True

    def cancelPrint(self):
        self.currentInstruction=0
        self.printing = False
        self.printPaused = False
        self.root.statusPrint("Print cancelled")
        pass

    # Bed Calibration and sweeps ==========================

    def startPrintBedCalibration(self):
        self.flag = None #set flag to none only on start

        #If anything is busy cannot start arm
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Printer is busy, cannot start bed calibration")
            return
        
        self.bedCalibration = True #flag to signal calibration
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
        if self.bedCalibration==False:
            self.root.statusPrint("Bed calibration not started")
        #Pause calibration
        if self.flag is not None:
            self.root.statusPrint("paused calibration step because flag: "+self.flag)
            self.root.warningPrint("Flag: "+self.flag)
            self.flag = None
        if self.bedCalibration == True:
            bedCalibrationThread = threading.Thread(target=self.bedCalibrationStep)
            bedCalibrationThread.start()
        

    def startCornerSweep(self):

        self.flag = None #reset flag
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start corner sweep")
            return
        else:
            self.cornerSweeping = True
        cornerSweepThread = threading.Thread(target=self.cornerSweep)
        cornerSweepThread.start()

    #corner sweep and move up one level at a time
    def startFullCornerSweep(self):
        self.flag = None
        
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start corner sweep")
            return
        else:
            self.cornerSweeping = True
        cornerSweepThread = threading.Thread(target=self.fullCornerSweep)
        cornerSweepThread.start()

    #will cancel any related printing setup functions
    def cancelAny(self):
        threading.Thread(target=self.endSweepOrCal)
        #just in case someone thinks this will cancel the print
        if self.printing:
            self.cancelPrint()
    #endregion gui

    #region ===============| Other Functions |=================


    def syncOrigin(self):
        self.origin = self.root.armController.origin

    #determines whether a position is within the boundaries of the printer
    #Get this functioning properly
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
        #End calibration
        if self.bedCalStep >= 5: #Is at 6 after 4 is completed so done
            self.endSweepOrCal()
            return
        #set label
        currentCornerPos = self.calibrationCorners[self.bedCalStep-1]
        self.root.cornerLabel.config(text=f"Current Corner: {self.bedCalStep}")

        #For better positioning move home than origin so J4 starts at 0 rather than 180
        #self.root.armController.moveHome()
        #self.root.armController.moveOrigin()

        #Move above
        posStep = copy.deepcopy(currentCornerPos)
        posStep.z += self.bedCalibrateHeight
        moveParameters = copy.deepcopy(self.defaultPrintParameters)
        moveParameters.wrist = "N"
        self.root.armController.sendMJ(posStep,moveParameters=moveParameters)
        
        #Move halfway
        self.root.armController.sendMJ(posStep,moveParameters=self.defaultPrintParameters)
        posStep.z -= self.bedCalibrateHeight/2

        #Move to corner to touch plate
        self.root.armController.sendMJ(currentCornerPos,moveParameters=self.defaultPrintParameters)
        self.root.statusPrint(f"Corner {self.bedCalStep} calibration complete")
        self.bedCalStep += 1

        

    # Used to sweep the corners without lifting to ensure kinematics are level to bed
    def cornerSweep(self, height=None, full=False):
       
        if self.flag is not None:
            self.root.terminalPrint("Corner sweep paused because of flag: "+ self.flag)
            self.root.warningPrint("Encountered flag: "+self.flag)
            #self.endSweepOrCal()
            #return
        
        #Height is assumed to be the height of the calibration corners
        if height is None:
            height = self.calibrationCorners[0].z
        moveOrder = [1,2,3,4,1,3,2,4]
        moveOrder= [x-1 for x in moveOrder] #adjust to 0 base index

        for i in moveOrder:
            time.sleep(1)
            pos = copy.deepcopy(self.calibrationCorners[i])
            pos.z = height
            self.root.cornerLabel.config(text=f"Current Corner: {i+1}")
            if self.cornerSweeping:
                self.root.armController.sendML(pos,moveParameters=self.defaultPrintParameters, timeout=20)
            #dont continue if no longer sweeping
            else:
                return
            if self.flag is not None:
                self.root.terminalPrint("Corner sweep paused because of flag: "+ self.flag)
                self.root.warningPrint("Encountered flag: "+self.flag)
                self.flag = None
                #self.endSweepOrCal()
                #return
            
        #End calibration
        #if not doing a fullSweep()
        if not full:
            self.endSweepOrCal()
    
    #End any corner sweep or bed calibration
    def endSweepOrCal(self):
        
        self.cornerSweeping = False
        self.bedCalibration=False
        self.bedCalStep == 0
        self.root.cornerLabel.config(text=f"Current Corner: N/A")
        #Wait until move finishes to send move home command
        while self.root.armController.checkIfBusy():
            pass
        self.root.armController.moveHome()

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
            if self.flag is not None:
                self.root.terminalPrint("Corner sweep cancelled because of flag: "+ self.flag)
                self.root.warningPrint("Encountered flag: "+self.flag)
                self.flag = None
                #self.endSweepOrCal()
