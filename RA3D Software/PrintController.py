from tkinter import *
from tkinter import filedialog
import os, re, copy, threading, time, math
from ArmController import Position, Origin, MoveCommand,MoveParameters

class PrintController:
    #region init
    def __init__(self, root):
        self.root = root
        #Placeholder boundaries, should be adjusted
        self.bufferBoundary = 10 # Warning pops up if this boundary is entered
        self.selectedFilepath = None
        self.gcodeLines = []
        self.teensyLines = []
        self.fileOpen = False
        self.printing = False
        self.printPaused = False
        self.cornerSweeping = False
        self.bedCalibration = False
        self.bedCalStep = 0
        self.plateHeight = 300
        YEdge = 80
        XEdge = [250,400]#for corner calibration
        self.bedCalibrateHeight = 50 #Height moved up for calibration
        FLCorner = Position(XEdge[1],-YEdge,self.plateHeight,0,90,0,None)
        FRCorner = Position(XEdge[1],YEdge,self.plateHeight,0,90,0,None)
        BLCorner = Position(XEdge[0],-YEdge,self.plateHeight,0,90,0,None)
        BRCorner = Position(XEdge[0],YEdge,self.plateHeight,0,90,0,None)
        #recomended Origin for move
        self.recommendedOriginPosition = Position((XEdge[0]+XEdge[1])/2,0,self.plateHeight,0,90,0,None)
        #extract position to origin
        self.recommendedOrigin = Origin(self.recommendedOrigin.x,self.recommendedOrigin.y,self.recommendedOrigin.z)
        
        #Assume origin is at recommended origin
        self.origin = self.recommendedOrigin

        #TODO These might be changed to reflect xedge, yedge
        self.maxBoundaryX = [200,500]
        self.maxBoundaryY = [-130,130]
        self.maxBoundaryZ = [290,800]
        
        #Flag variable for errors
        self.flag = None
        
        #self.origin = Origin(None,None,None)
        #corners are absolute, may change to relative to origin
        self.calibrationCorners = [FRCorner,BRCorner,BLCorner,FLCorner]
        # Parameters for printing coordinates
        #self.xBounds = [300, 500] # X Min & X Max
        #self.yBounds = [-100, 100] # Y Min & Y Max
        #self.zBounds = [100, 300] # Z Min & Z Max
        # Parameters used for saving the last used coordinate information
        self.lastPos = Position(None,None,None,None,None,None,self.origin)
        self.printPos = Position(None,None,None,None,None,None,self.origin)
        self.defaultPrintParameters = MoveParameters(50,5,5,15,0,"m") #20mm/s print speed
        self.lastF = 0.0
        self.lastE = 0.0
        self.feedRate = 0
        self.extrudeRate = 0
        self.currentInstruction = 0
        
    #endregion init
    #region ================== Main Functions =====================

    # This is the main function that will loop when printing a file
    def printLoop(self):

        self.checkFlag() #check flag to see if to continue printing
        
        if self.printing == False:
            self.root.printThreadStarted = False
            return
        
        message = "comment"
        #G code lines will skip past comments
        while message == "comment":
            if self.currentInstruction > len(self.gcodeLines) - 1:
                self.root.statusPrint("End of program reached")
                self.currentInstruction = 0
                self.printing = False
                #Move Home when complete
                self.root.armController.moveHome()
                self.root.printThreadStarted = False
                return
            
            lineToConvert = self.gcodeLines[self.currentInstruction] # Pull current line
            
            self.currentInstruction += 1 # Increment currentInstruction
            self.root.progressBar["value"] = (self.currentInstruction / len(self.gcodeLines)) * 100 # Update progress bar to match
            message = self.gcodeToTeensy(lineToConvert) # Convert line and updates printPos
            
        self.root.terminalPrint(f"Line: {lineToConvert}")# Print the line we're converting
        
        # TODO turn this into a switchcase structure
        #Message Processing
        if message == "" or message == "Error": # If the point is blank, don't try to send a command
            self.root.printThreadStarted = False
            return
        # TODO: Add waiting for temperature to heat up from M104/M109 commands
        if message == "Wait":
            time.sleep(3)
            self.root.printThreadStarted = False
            return
        if message == "Home":
            self.root.armController.moveHome()
            self.root.armController.moveOrigin()
            self.root.printThreadStarted = False
            return
        
        #Execute move command
        moveParameters = copy.deepcopy(self.defaultPrintParameters)
        if self.feedRate != None and self.feedRate != 0:
            #moveParameters.speedType = "m"
            #Conver to mm/s from mm/min
            moveParameters.speed = self.feedRate / 60
        #estimate timeout
        timeout = self.root.armController.estimateMoveTime(self.lastPos,self.printPos,moveParameters.speed)
        timeout += 10 #extra time for communication
        
        if self.printPos.x is not None:
            self.root.terminalPrint(f"Point: {self.printPos.GetAbsolute()}") # Print the returned point list
            # Send the command to the arm, will wait for a response
            self.root.armController.sendML(pos=self.printPos, moveParameters=moveParameters, extrudeRate=self.extrudeRate,timeout=timeout)
        self.root.printThreadStarted = False

        #must be last thing to do, copy last position
        self.lastPos = copy.deepcopy(self.printPos)


    # Converts a GCode instruction to the instruction to send over serial
    def gcodeToTeensy(self, lineToConvert):
        #Assume feedrate and extruderate will not be set
        self.feedRate = 0
        self.extrudeRate = 0

        if lineToConvert[0] == ';': # Line is comment
            return "comment" # Don't convert
        elif lineToConvert[0:3] == "G21":
            return ""
        elif lineToConvert == "\n": # Line is newline
            return "" # Don't convert
        elif lineToConvert == "M104":
            return ""
        elif lineToConvert == "M109":
            return ""
        # Actual instructions to convert
        elif lineToConvert[0:3] == "G28": # Home the printer
            return "Home"
        elif lineToConvert[0:3] == "G90": # Absolute positioning
            # TODO: This needs handling or removal
            return ""
        elif lineToConvert[0:2] == "G0" or lineToConvert[0:2] == "G1": # Move (treating G0 & G1 as equal)

            xMatch = re.search(r"X(-?\d+\.?\d*)", lineToConvert)
            yMatch = re.search(r"Y(-?\d+\.?\d*)", lineToConvert)
            zMatch = re.search(r"Z(-?\d+\.?\d*)", lineToConvert)
            fMatch = re.search(r"F(-?\d+\.?\d*)", lineToConvert)
            eMatch = re.search(r"E(-?\d+\.?\d*)", lineToConvert)

            #Coordinates here are RELATIVE
            x = float(xMatch.group(1)) if xMatch else None
            y = float(yMatch.group(1)) if yMatch else None
            z = float(zMatch.group(1)) if zMatch else None
            #print("z read:", z)
            f = float(fMatch.group(1)) if fMatch else None
            e = float(eMatch.group(1)) if eMatch else None
            # TODO: Temporary rotation information

            # If GCode instruction didn't contain a parameter, pull from last saved value
            # If instruction DID contain a parameter, offset the value to put it in the build volume
            if x == None:
                x = self.lastPos.GetRelative()[0]
            if y == None:
                y = self.lastPos.GetRelative()[1]
            if z == None:
                z = self.lastPos.GetRelative()[2]
            if f == None:
                f = 0.0
            else:
                self.lastF = f
            if e == None:
                e = self.lastE
            else:
                self.lastE = e
            atBoundary = False
            self.printPos.SetRelative(x,y,z,0,90,0)
            #If values are within boundaries
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
            
            
            self.feedRate = f
            self.extrudeRate = e

            
            #Check if point is in boundary else pause print
            if self.checkBoundary(self.printPos):
                return "Success"
            else:
                #self.pausePrint()
                self.root.warningPrint("Moving of bounds")
                return "Error"
    
        # TODO: Additional processing for F to control speed or something
        # Note that F is in units per minute (per LinuxCNC specifications)
        # https://linuxcnc.org/docs/html/gcode/machining-center.html#sub:feed-rate
        # TODO: Note that it is pointless to convert to strings because to send the instruction to the arm (through current methods) we give the coordinates
        # TODO: Might make a custom datatype for storing position data that can be used
        #newLine = f"MLX{x}Y{y}Z{z}Rz{Rz}Ry{Ry}Rx{Rx}J70.00J80.00J90.00Sp{self.root.armController.speed}Ac{self.root.armController.acceleration}Dc{self.root.armController.deceleration}Rm{self.root.armController.ramp}Rnd0WFLm000000Q0\n"
        #return newLine
        return ["",0,0]
    

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
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start print")
            return
        if self.root.armController.armCalibrated is False:
            self.root.warningPrint("Cannot start print arm is not calibrated")
            return
        #Resume print if paused
        if self.printPaused == True and self.printing == True:
            self.printPaused = False
            return
        
        #Reset flag
        self.flag = None
        # When starting print, reset the "last*" parameters
        print(self.origin.z, "test2")
        self.printPos.origin = self.origin
        self.printPos = self.origin.toPosition()
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
        if self.flag is not None:
            self.pausePrint()
            self.root.warningPrint(f"Print paused due to error: {self.flag}")
            return self.flag
        return None

    
    #TODO Improve this function and make it work well with armcontroller check if busy
    def checkIfPrinterBusy(self):
        if self.printing or self.cornerSweeping or self.bedCalibration:
            self.root.terminalPrint("Printer busy")
            return True
        return False

    def pausePrint(self):
        self.root.terminalPrint("Pausing Print")
        self.printing = False
        self.printPaused = True

    def cancelPrint(self):
        self.currentInstruction=0
        self.printing = False
        self.printPaused = False
        self.root.statusPrint("Print cancelled")
        pass

    # Calibration and sweeps ==========================

    def startPrintBedCalibration(self):
        self.flag = None #set flag to none only on start
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Printer is busy, cannot start bed calibration")
            return
        self.bedCalibration = True
        self.bedCalStep = 1
        self.syncOrigin()
        self.root.armController.moveHome()
        self.nextBedCalibration()

    #user controlled next button
    def nextBedCalibration(self):
        #Do not check all busy because bed calibration is one of the flags
        if self.root.armController.checkIfBusy():
            self.root.statusPrint("Arm is busy cannot go to next step")
            return
        if self.bedCalibration==False:
            self.root.statusPrint("Bed calibration not started")
        if self.bedCalibration == True and self.flag == None:
            bedCalibrationThread = threading.Thread(target=self.bedCalibrationStep)
            bedCalibrationThread.start()

    def startCornerSweep(self):
        self.flag = None
       
        if self.root.armController.checkIfAllBusy():
            self.root.statusPrint("Arm is busy, cannot start corner sweep")
            return
        else:
            self.cornerSweeping = True
        cornerSweepThread = threading.Thread(target=self.cornerSweep, args=self.plateHeight)
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
        self.endSweepOrCal()
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
        #set label
        currentCornerPos = self.calibrationCorners[self.bedCalStep-1]
        self.root.cornerLabel.config(text=f"Current Corner: {self.bedCalStep}")

        #For better positioning move home than origin so J4 starts at 0 rather than 180
        self.root.armController.moveHome()
        self.root.armController.moveOrigin()

        #Move above
        posStep = copy.deepcopy(currentCornerPos)
        posStep.z += self.bedCalibrateHeight
        self.root.armController.sendML(posStep,moveParameters=self.defaultPrintParameters)
        
        #Move halfway
        self.root.armController.sendML(posStep,moveParameters=self.defaultPrintParameters)
        posStep.z -= self.bedCalibrateHeight/2

        #Move to corner to touch plate
        self.root.armController.sendML(currentCornerPos,moveParameters=self.defaultPrintParameters)
        self.root.statusPrint(f"Corner {self.bedCalStep} calibration complete")
        self.bedCalStep += 1

        #End calibration
        if self.bedCalStep == 5: #Is at 5 after 4 is completed so done
            self.endSweepOrCal()


    # Used to sweep the corners without lifting to ensure kinematics are level to bed
    def cornerSweep(self, height, full=False):
       
        if self.flag is not None:
            self.root.terminalPrint("Corner sweep cancelled because of flag: "+ self.flag)
            self.endSweepOrCal()
            return
        moveOrder = [1,2,3,4,1,3,2,4]
        moveOrder= [x-1 for x in moveOrder] #adjust to 0 base index
        for i in moveOrder:
            time.sleep(1)
            pos = copy.deepcopy(self.calibrationCorners[i])
            pos.z = height
            self.root.cornerLabel.config(text=f"Current Corner: {i+1}")
            if self.cornerSweeping:
                self.root.armController.sendML(pos,moveParameters=self.defaultPrintParameters, timeout=10)
            #dont continue if no longer sweeping
            else:
                return
            if self.flag is not None:
                self.root.terminalPrint("Corner sweep cancelled because of flag: "+ self.flag)
                self.endSweepOrCal
                return
            #Move To position
            #End calibration
        #if not doing a fullSweep()
        if not full:
            self.endSweepOrCal()
        
    
    def endSweepOrCal(self):
        self.cornerSweeping = False
        self.bedCalibration=False
        self.bedCalStep == 0
        self.root.cornerLabel.config(text=f"Current Corner: N/A")
        self.root.armController.moveHome()

    def fullCornerSweep(self):
        self.root.armController.moveHome()
        self.root.armController.moveOrigin()
        heightDelta = 200
        currentHeight = self.plateHeight
        endHeight = currentHeight + heightDelta
        while currentHeight < endHeight and self.cornerSweeping:
            self.cornerSweep(currentHeight,full=True)
            currentHeight += 20
            if self.flag is not None:
                self.root.terminalPrint("Corner sweep cancelled because of flag: "+ self.flag)
                self.endSweepOrCal()
    
    #endregion Calibration
        
    # TODO check if busy for the printcontroller
