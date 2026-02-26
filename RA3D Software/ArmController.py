from asyncio import wait
import numpy as np
import re, time, threading, math, copy
from SerialController import SerialController
from Kinematics import Kinematics

class ArmController:
    #region init
    def __init__(self, root, serialController):
        # Save references to the main window and the serial controller
        self.root = root
        self.serialController = serialController
        self.kinematics = Kinematics
        #Default loop mode Closed
        # Arm calibration variables
        self.armCalibrated = False # Flag to signify if the arm has been calibrated
        self.calibrationInProgress = False # Flag to signify if calibration is currently in progress
        # Contains current calibration step
        #  0 - N/A
        #  1 - Send CalStage1
        #  2 - Wait for CalStage1 resposne & process when ready
        #  3 - Send CalStage2
        #  4 - Wait for CalStage2 response & process when ready
        self.calibrationState = 0
        # These variables structure when to calibrate each joint. 1 signifying that joint will be calibrated in that stage
        self.calJStage1 = [1, 1, 1, 0, 0, 0] # J1, J2, & J3 calibration in Stage 1
        self.calJStage2 = [0, 0, 0, 1, 1, 1] # J4, J5, & J6 calibration in Stage 2

        # Speed parameters used for movement commands
        self.defaultMoveParameters = MoveParameters(80,10,10,30,0,'p')
        #J Limits:
        self.J1Limits = [-170,170]
        self.J2Limits = [-42,90]
        self.J3Limits = [-89,52]; 
        self.J4Limits = [-165,165]
        self.J5Limits = [-86,105]
        self.J6Limits = [-155,155]
        # Stores current variable information
        self.curJ1 = None
        self.curJ2 = None
        self.curJ3 = None
        self.curJ4 = None
        self.curJ5 = None
        self.curJ6 = None
        #Orign variables
        self.origin = Origin(None,None,None)
        self.curPos = Position(None,None,None,None,None,None,self.origin)
        # Stores calibration offset values
        self.J1CalOffset = 0
        self.J2CalOffset = 0
        self.J3CalOffset = 0
        self.J4CalOffset = 0
        self.J5CalOffset = 0
        self.J6CalOffset = 0

        # Other variables
        self.awaitingMoveResponse = False # Flag for if the ArmController is awaiting a serial response after sending a move command
        self.testingLimitSwitches = False # Flag for if the limit switch test is being performed
        self.testingEncoders = False      # Flag for if the encoder test is being performed
        self.awaitingTestResponse = False # Flag for if we are awaiting a response after sending a test command such as for the limit switches or encoders
        self.finishTest = False           # Flag for signifying to the program that the user wants to stop a test
        self.awaitingPosResponse = False  # Flag for requesting current position
    
    #endregion init

    #region ========|Calibration|==========
    def startArmCalibration(self):
        # Check if calibration is already in progress and exit if so
        #TODO a warning print is printed for all buttons when board is not connected
        if not self.serialController.boardConnected:
            self.root.warningPrint("Board not connected")
            return
        if self.calibrationInProgress is True:
            self.root.statusPrint("Calibration already in progress")
            return
        # Check if arm is busy with something else
        if self.checkIfBusy() is True:
            self.root.statusPrint("Arm is busy with something else at the moment")
            return
        self.root.statusPrint("Beginning arm calibration")
        # Set flag for calibration in progress
        self.calibrationInProgress = True
        # Move to next state of calibration
        self.calibrationState = 1
        # Call the calibration update function
        self.calibrateArmUpdate()

    def calibrateArmUpdate(self,response=None):
        # Exit the function if calibration is not in progress and exit if so
        if self.calibrationInProgress is False:
            return
        # Check current state and perform associated tasks
        if self.calibrationState == 1: # Send CalStage1
            self.calibrateJoints(calJ1=self.calJStage1[0],
                                 calJ2=self.calJStage1[1],
                                 calJ3=self.calJStage1[2],
                                 calJ4=self.calJStage1[3],
                                 calJ5=self.calJStage1[4],
                                 calJ6=self.calJStage1[5])
            self.calibrationState = 2
        elif self.calibrationState == 2: # Await CalStage1 Response & process when ready
            # Check if the serial controller has a response ready
            if response is not None:
                # Save the response
                # Check if the calibration was successful
                # Inform user of Stage 1 success
                self.root.statusPrint("Stage 1 Calibration Successful")
                # Process position response and dispaly
                self.processPosition(response)
                # Move to next state
                self.calibrationState = 3
                # Print out the response received
                self.root.terminalPrint(response)
        elif self.calibrationState == 3: # Send CalStage2
            self.calibrateJoints(calJ1=self.calJStage2[0],
                                 calJ2=self.calJStage2[1],
                                 calJ3=self.calJStage2[2],
                                 calJ4=self.calJStage2[3],
                                 calJ5=self.calJStage2[4],
                                 calJ6=self.calJStage2[5])
            self.calibrationState = 4
        elif self.calibrationState == 4: # Await CalStage2 Response & process when ready
            # Check if the serial controller has a response ready
            if response is not None:
                # Check if the calibration was successful
                # Inform user of Stage 1 success
                self.root.statusPrint("Stage 2 Calibration Successful")
                # Process position response and dispaly
                self.processPosition(response)

                # Calibration complete
                self.calibrationState = 0
                # Calibration no longer in progress
                self.calibrationInProgress = False
                # Set arm calibration flag to True
                self.armCalibrated = True
                # Print out the response received
                self.root.terminalPrint(response)
                self.root.timeoutStartedCal = False
                #On every calibration reset to recomended
                self.setOrigin(self.printController.recommendedOrigin)

    #for debugging
    def overrideCalibration(self):
        self.armCalibrated = True

    def calibrateJoints(self, calJ1=False, calJ2=False, calJ3=False, calJ4=False, calJ5=False, calJ6=False):
        self.getCalOffsets() # Update calibration offsets from entry fields
        command = f"LLA{calJ1}B{calJ2}C{calJ3}D{calJ4}E{calJ5}F{calJ6}G0H0I0J{self.J1CalOffset}K{self.J2CalOffset}L{self.J3CalOffset}M{self.J4CalOffset}N{self.J5CalOffset}O{self.J6CalOffset}P0Q0\n"
        self.root.terminalPrint("Command to send: ")
        self.root.terminalPrint(command[0:-2])
        if self.serialController.boardConnected is False:
            self.root.statusPrint("Command not sent due to no board connected")
            return "E"
        # Tell the serial controller to send the serial
        self.serialController.sendSerial(command)

    def postCalibrateJoints(self, calJ1=False, calJ2=False, calJ3=False, calJ4=False, calJ5=False, calJ6=False):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Cannot post calibrate arm is busy")

        self.getPostCalOffsets() # Update calibration offsets from entry fields
        command = f"LEA{calJ1}B{calJ2}C{calJ3}D{calJ4}E{calJ5}F{calJ6}G0H0I0J{self.J1CalOffset}K{self.J2CalOffset}L{self.J3CalOffset}M{self.J4CalOffset}N{self.J5CalOffset}O{self.J6CalOffset}P0Q0\n"
        self.root.terminalPrint("Command to send: ")
        self.root.terminalPrint(command[0:-2])
        if self.serialController.boardConnected is False:
            self.root.statusPrint("Command not sent due to no board connected")
            return "E"
        # Tell the serial controller to send the serial
        self.serialController.sendSerial(command)
        self.awaitingMoveResponse = True
        # Set values back to zero to avoid unintentionally stacking offsets
        self.root.J1OffsetEntryP.delete(0, 'end')
        self.root.J2OffsetEntryP.delete(0, 'end')
        self.root.J3OffsetEntryP.delete(0, 'end')
        self.root.J4OffsetEntryP.delete(0, 'end')
        self.root.J5OffsetEntryP.delete(0, 'end')
        self.root.J6OffsetEntryP.delete(0, 'end')
        self.root.J1OffsetEntryP.insert(0,"0")
        self.root.J2OffsetEntryP.insert(0,"0")
        self.root.J3OffsetEntryP.insert(0,"0")
        self.root.J4OffsetEntryP.insert(0,"0")
        self.root.J5OffsetEntryP.insert(0,"0")
        self.root.J6OffsetEntryP.insert(0,"0")

    def getPostCalOffsets(self):
        self.J1CalOffset = float(self.root.J1OffsetEntryP.get())
        self.J2CalOffset = float(self.root.J2OffsetEntryP.get())
        self.J3CalOffset = float(self.root.J3OffsetEntryP.get())
        self.J4CalOffset = float(self.root.J4OffsetEntryP.get())
        self.J5CalOffset = float(self.root.J5OffsetEntryP.get())
        self.J6CalOffset = float(self.root.J6OffsetEntryP.get())
        self.root.terminalPrint(f"Post calibration offsets J1: {self.J1CalOffset}, J2: {self.J2CalOffset}, J3: {self.J3CalOffset}, J4: {self.J4CalOffset}, J5: {self.J5CalOffset}, J6: {self.J6CalOffset}")
    
    def getCalOffsets(self):
        # Grab values from the entry fields, convert to integers, and save
        self.J1CalOffset = float(self.root.J1OffsetEntry.get())
        self.J2CalOffset = float(self.root.J2OffsetEntry.get())
        self.J3CalOffset = float(self.root.J3OffsetEntry.get())
        self.J4CalOffset = float(self.root.J4OffsetEntry.get())
        self.J5CalOffset = float(self.root.J5OffsetEntry.get())
        self.J6CalOffset = float(self.root.J6OffsetEntry.get())
        self.root.terminalPrint(f"Calibration offsets J1: {self.J1CalOffset}, J2: {self.J2CalOffset}, J3: {self.J3CalOffset}, J4: {self.J4CalOffset}, J5: {self.J5CalOffset}, J6: {self.J6CalOffset}")
    #endregion Calibration

    #region ========|Position|=============

    def processPosition(self, response):
        # Collect all the indexes for finding values
        # General formatting of a response: A[val]B[val]C[val]D[val]E[val]F[val]G[val]H[val]I[val]J[val]K[val]L[val]M[val]N[val]O[val]P[val]Q[val]R[val]
        J1Idx = response.find('A') # A value is angle of J1
        J2Idx = response.find('B') # B value is angle of J2
        J3Idx = response.find('C') # C value is angle of J3
        J4Idx = response.find('D') # D value is angle of J4
        J5Idx = response.find('E') # E value is angle of J5
        J6Idx = response.find('F') # F value is angle of J6
        XPosIdx = response.find('G') # G value is X position
        YPosIdx = response.find('H') # H value is Y position
        ZPosIdx = response.find('I') # I value is Z position
        RzIdx = response.find('J') # J value is Rz rotation
        RyIdx = response.find('K') # K value is Ry rotation
        RxIdx = response.find('L') # L value is Rx rotation
        SpeedViolationIdx = response.find('M') # M value is if there is a speed violation
        DebugIdx = response.find('N') # N value is ???
        FlagIdx = response.find('O') # O value is ???
        J7Idx = response.find('P') # P value is angle of J7
        J8Idx = response.find('Q') # Q value is angle of J8
        J9Idx = response.find('R') # R value is angle of J9
        # Extract the actual values from the response
        # Joint angles
        self.curJ1 = float(response[J1Idx+1:J2Idx].strip())
        self.curJ2 = float(response[J2Idx+1:J3Idx].strip())
        self.curJ3 = float(response[J3Idx+1:J4Idx].strip())
        self.curJ4 = float(response[J4Idx+1:J5Idx].strip())
        self.curJ5 = float(response[J5Idx+1:J6Idx].strip())
        self.curJ6 = float(response[J6Idx+1:XPosIdx].strip())
        
        # XYZ Positions
        self.curPos.x = float(response[XPosIdx+1:YPosIdx].strip())
        self.curPos.y = float(response[YPosIdx+1:ZPosIdx].strip())
        self.curPos.z = float(response[ZPosIdx+1:RzIdx].strip())

        # RXYZ Angles
        self.curPos.Rx = float(response[RxIdx+1:SpeedViolationIdx].strip())
        self.curPos.Ry = float(response[RyIdx+1:RxIdx].strip())
        self.curPos.Rz = float(response[RzIdx+1:RyIdx].strip())

        # Display values on UI
        # XYZ
        self.root.xCurCoord.config(text=self.curPos.x)
        self.root.yCurCoord.config(text=self.curPos.y)
        self.root.zCurCoord.config(text=self.curPos.z)
        self.root.RxCurCoord.config(text=self.curPos.Rx)
        self.root.RyCurCoord.config(text=self.curPos.Ry)
        self.root.RzCurCoord.config(text=self.curPos.Rz)
        jointColors=self.getJointColors(self.curJ1,self.curJ2,self.curJ3,self.curJ4,self.curJ5,self.curJ6)
        # Joint
        self.root.J1CurCoord.config(text=self.curJ1, fg=jointColors[0])
        self.root.J2CurCoord.config(text=self.curJ2, fg=jointColors[1])
        self.root.J3CurCoord.config(text=self.curJ3, fg=jointColors[2])
        self.root.J4CurCoord.config(text=self.curJ4, fg=jointColors[3])
        self.root.J5CurCoord.config(text=self.curJ5, fg=jointColors[4])
        self.root.J6CurCoord.config(text=self.curJ6, fg=jointColors[5])
        self.updateDeltaFromOrigin()

    def requestPositionManual(self):
        # Check if a board is connected
        # Check if the arm is busy with anything else
        if self.nominalCheck():
            self.root.statusPrint("Failed to request position update.")
            return
        self.root.statusPrint("Requesting position update...")
        self.serialController.sendSerial("RP\n") # Send instruction
        self.awaitingPosResponse = True # Set the flag
        positionThread = threading.Thread(target=self.requestPositionAndWait)
        positionThread.start()
                

    def requestPositionAndWait(self):
        # Return if we aren't awaiting a position response
        if self.awaitingPosResponse is False:
            return
        
        response = self.root.serialController.waitForResponse("POS",3)
        # Check if the serial controller has a response ready
        self.root.terminalPrint("response is " + response)
        # Inform user and process the position response
        if response is not None:
            self.root.statusPrint("Position request fulfilled")
            self.processPosition(response)
            # Reset the awaiting position respone flag
            self.awaitingPosResponse = False
            self.root.timeoutStartedPos = False

    #endregion process position

    #region GUI Functions
    #TODO add busy check for any user buttons and add more flags if needed
    def startPostCalibration(self, calJ1=False, calJ2=False, calJ3=False, calJ4=False, calJ5=False, calJ6=False):
        # Check if calibration is already in progress and exit if so
        if self.calibrationInProgress is True:
            self.root.statusPrint("Calibration already in progress")
            return
        # Check if arm is busy with something else
        if self.checkIfBusy() is True:
            self.root.statusPrint("Arm is busy with something else at the moment")
            return
        self.root.statusPrint("Beginning post calibration")
        self.postCalibrateJoints(calJ1=calJ1, calJ2=calJ2, calJ3=calJ3, calJ4=calJ4, calJ5=calJ5, calJ6=calJ6)

    #calibration for one joint at a time
    def startSpecificCalibration(self, calJ1=False, calJ2=False, calJ3=False, calJ4=False, calJ5=False, calJ6=False):
        # Check if calibration is already in progress and exit if so
        if self.calibrationInProgress is True:
            self.root.statusPrint("Calibration already in progress")
            return
        # Check if arm is busy with something else
        if self.checkIfBusy() is True:
            self.root.statusPrint("Arm is busy with something else at the moment")
            return
        self.root.statusPrint("Beginning arm calibration")
        self.calibrateJoints(calJ1=calJ1, calJ2=calJ2, calJ3=calJ3, calJ4=calJ4, calJ5=calJ5, calJ6=calJ6)

    #getxyz
    #TODO add request posistion before setting
    def populateMJ(self):
        if self.checkIfBusy() is True:
            self.root.statusPrint("Failed to request position update. Arm is busy.")
            return
        self.requestPositionAndWait()
        self.root.xCoordEntry.delete(0, 'end')
        self.root.yCoordEntry.delete(0, 'end')
        self.root.zCoordEntry.delete(0, 'end')
        self.root.RxCoordEntry.delete(0, 'end')
        self.root.RyCoordEntry.delete(0, 'end')
        self.root.RzCoordEntry.delete(0, 'end')
        self.root.xCoordEntry.insert(0,str(self.curPos.x))
        self.root.yCoordEntry.insert(0,str(self.curPos.y))
        self.root.zCoordEntry.insert(0,str(self.curPos.z))
        self.root.RxCoordEntry.insert(0,str(self.curPos.Rx))
        self.root.RyCoordEntry.insert(0,str(self.curPos.Ry))
        self.root.RzCoordEntry.insert(0,str(self.curPos.Rz))
    
    def populateJoints(self):
        if self.checkIfBusy() is True:
            self.root.statusPrint("Failed to request position update. Arm is busy.")
            return
        self.requestPositionAndWait()
        self.root.J1CoordEntry.delete(0, 'end')
        self.root.J2CoordEntry.delete(0, 'end')
        self.root.J3CoordEntry.delete(0, 'end')
        self.root.J4CoordEntry.delete(0, 'end')
        self.root.J5CoordEntry.delete(0, 'end')
        self.root.J6CoordEntry.delete(0, 'end')
        self.root.J1CoordEntry.insert(0,str(self.curJ1))
        self.root.J2CoordEntry.insert(0,str(self.curJ2))
        self.root.J3CoordEntry.insert(0,str(self.curJ3))
        self.root.J4CoordEntry.insert(0,str(self.curJ4))
        self.root.J5CoordEntry.insert(0,str(self.curJ5))
        self.root.J6CoordEntry.insert(0,str(self.curJ6))

    def prepRJCommand(self):
        if self.checkIfAllBusy():
            self.root.statusPrint("Cannot send RJ command. Arm is busy.")
            return
        # Read the values from each entry box
        J1 = self.root.J1CoordEntry.get()
        J2 = self.root.J2CoordEntry.get()
        J3 = self.root.J3CoordEntry.get()
        J4 = self.root.J4CoordEntry.get()
        J5 = self.root.J5CoordEntry.get()
        J6 = self.root.J6CoordEntry.get()
        # Check if any values are blank
        allValuesNumeric = True
        pattern = r"^-?(\d+(?:\.\d+)?)"  # Regular expression for a valid float

        if not re.match(pattern, J1):
            self.root.terminalPrint("J1 is not a number")
            allValuesNumeric = False
        if not re.match(pattern, J2):
            self.root.terminalPrint("J2 is not a number")
            allValuesNumeric = False
        if not re.match(pattern, J3):
            self.root.terminalPrint("J3 is not a number")
            allValuesNumeric = False
        if not re.match(pattern, J4):
            self.root.terminalPrint("J4 is not a number")
            allValuesNumeric = False
        if not re.match(pattern, J5):
            self.root.terminalPrint("J5 is not a number")
            allValuesNumeric = False
        if not re.match(pattern, J6):
            self.root.terminalPrint("J6 is not a number")
            allValuesNumeric = False
        
        if allValuesNumeric:
            self.root.terminalPrint("All values numeric, sending RJ command")
            #TODO start sendRJ as a thread so that 
            RJThread = threading.Thread(target=self.sendRJ, args=[J1, J2, J3, J4, J5, J6, self.defaultMoveParameters])
            RJThread.start()
        else:
            self.root.terminalPrint("RJ command not sent due to a value not being a number")

    def prepMLCommand(self):
        #TODO implement this where it is needed
        if self.checkIfAllBusy():
            self.root.statusPrint("Cannot send ML command. Arm is busy.")
            return
        # Read the values from each entry box
        x  = self.root.xCoordEntry.get()
        y  = self.root.yCoordEntry.get()
        z  = self.root.zCoordEntry.get()
        Rx = self.root.RxCoordEntry.get()
        Ry = self.root.RyCoordEntry.get()
        Rz = self.root.RzCoordEntry.get()
        # Check if any values are blank
        pattern = r"^-?(\d+(?:\.\d+)?)"  # Regular expression for a valid float
        allValuesNumeric = True
        if not re.match(pattern, x):
            self.root.terminalPrint("X is not a number")
            allValuesNumeric = False
        if not re.match(pattern, y):
            self.root.terminalPrint("Y is not a number")
            allValuesNumeric = False
        if not re.match(pattern, z):
            self.root.terminalPrint("Z is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Rx):
            self.root.terminalPrint("Rx is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Ry):
            self.root.terminalPrint("Ry is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Rz):
            self.root.terminalPrint("Rz is not a number")
            allValuesNumeric = False
        
        if allValuesNumeric:
            #self.root.terminalPrint("All values numeric, sending ML command")
            commandPos = Position(x,y,z,Rx,Ry,Rz,None)
            #Thread so that command doesn't interupt UI
            MLThread = threading.Thread(target=self.sendML, args=[commandPos, self.defaultMoveParameters])
            MLThread.start()
        else:
            self.root.statusPrint("ML command not sent due to a value not being a number")

    def prepMJCommand(self):
        if self.checkIfAllBusy():
            self.root.statusPrint("Cannot send RJ command. Arm is busy.")
            return
        # Read the values from each entry box
        x  = self.root.xCoordEntry.get()
        y  = self.root.yCoordEntry.get()
        z  = self.root.zCoordEntry.get()
        Rx = self.root.RxCoordEntry.get()
        Ry = self.root.RyCoordEntry.get()
        Rz = self.root.RzCoordEntry.get()
        # Check if any values are blank
        pattern = r"^-?(\d+(?:\.\d+)?)"  # Regular expression for a valid float
        allValuesNumeric = True
        if not re.match(pattern, x):
            self.root.terminalPrint("X is not a number")
            allValuesNumeric = False
        if not re.match(pattern, y):
            self.root.terminalPrint("Y is not a number")
            allValuesNumeric = False
        if not re.match(pattern, z):
            self.root.terminalPrint("Z is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Rx):
            self.root.terminalPrint("Rx is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Ry):
            self.root.terminalPrint("Ry is not a number")
            allValuesNumeric = False
        if not re.match(pattern, Rz):
            self.root.terminalPrint("Rz is not a number")
            allValuesNumeric = False
        
        if allValuesNumeric:
            self.root.terminalPrint("All values numeric, sending ML command")
            #Thread so that command doesn't interupt UI
            commandPos = Position(float(x),float(y),float(z),float(Rx),float(Ry),float(Rz),None)
            MJThread = threading.Thread(target=self.sendMJ, args=[commandPos, self.defaultMoveParameters])
            MJThread.start()
        else:
            self.root.statusPrint("ML command not sent due to a value not being a number")

    def getJointColors(self,J1,J2,J3,J4,J5,J6):
        if J1 == None:
            return ["#000000"]*6
        Js = [float(x) for x in [J1,J2,J3,J4,J5,J6]]
        Js = np.matrix(Js).T
        Limits = np.matrix([self.J1Limits,self.J2Limits,self.J3Limits,self.J4Limits,self.J5Limits,self.J6Limits])
        
        LimitsNeg = Limits[:,0]
        LimitsPos = Limits[:,1]
        #Find Angles Closest to Limits rather than furthest from center
        NegativeDeltas = LimitsNeg-Js
        PositiveDeltas = LimitsPos-Js
        Scores = np.zeros([6,1])
        for i in range(len(Js)):
            if abs(PositiveDeltas[i]) < abs(NegativeDeltas[i]):
                if PositiveDeltas[i]<LimitsPos[i]:
                    Scores[i] = (1-abs(PositiveDeltas[i])/LimitsPos[i])*255
                else:
                    Scores[i] = 0
            else:
                if NegativeDeltas[i]>LimitsNeg[i]:
                    Scores[i] = (1-abs(NegativeDeltas[i])/abs(LimitsNeg[i]))*255
                else:
                    Scores[i] = 0
        greenArray = 255-Scores
        green = [int(x[0])for x in greenArray.astype(int)]
        red = [int(x[0])for x in Scores.astype(int)]
        blue = 0
        jointColors = [" "]*6
        for i in range(len(green)):
            rgb = (red[i],green[i],blue)
            jointColors[i] = '#%02x%02x%02x' % rgb
        return jointColors
    
    
    def prepMoveHome(self):
        if self.checkIfAllBusy():
            self.root.statusPrint("Cannot Home. Arm is busy.")
            return
        if self.serialController.boardConnected is False or self.armCalibrated is False:
            self.root.statusPrint("Failed to move home")
            return
        self.moveHome()
        
    def prepMoveSafe(self):
        if self.checkIfAllBusy():
            self.root.statusPrint("Cannot Home. Arm is busy.")
            return
        if self.serialController.boardConnected is False or self.armCalibrated is False:
            self.root.statusPrint("Failed to move to safe positiion, Board not connected")
            return
        self.moveSafe()

    def setOpenLoop(self):
        self.defaultMoveParameters.setLoopMode(1)
        self.root.loopStatus.config(text="Open Loop")

    def setClosedLoop(self):
        self.defaultMoveParameters.setLoopMode(0)
        self.root.loopStatus.config(text="Closed Loop")
    
    #endregion GUI
    
    #region Move Commands
    def sendMJ(self,commandPos, moveParameters,timeout=10):
        if self.awaitingMoveResponse:
            self.root.statusPrint("Cannot send ML command as currently awaiting response from a previous move command")
            return
        #self.root.terminalPrint("Sending MJ command...")
        # Taken from AR4.py, line XXXX
        # command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
        # Create the command
        command = MoveCommand("MJ",commandPos, moveParameters)
        #self.root.terminalPrint("Command to send:")
        #self.root.terminalPrint(str(command)[0:-2])
        # Check if board is not connected or arm is not calibrated
        if self.serialController.boardConnected is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.warningPrint("Command not sent due to no board connected")
            return
        elif self.armCalibrated is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.warningPrint("Command not sent due to arm not calibrated")
            return
        # Send the serial command
        self.awaitingMoveResponse = True # Set the awaiting move response flag
        self.serialController.sendSerial(str(command))

        #Timeout and feedback handling
        response = self.root.serialController.waitForResponse("POS",timeout)
        
        if response is not None:
            self.processPosition(response)
            self.root.terminalPrint(response)
            #self.root.statusPrint("Move command executed successfully")
        self.awaitingMoveResponse = False

    #Move linear, timeout default is 10
    def sendML(self, pos, moveParameters, extrudeRate=None, timeout=10):
        if self.awaitingMoveResponse:
            self.root.statusPrint("Cannot send ML command as currently awaiting response from a previous move command")
            return
        #self.root.terminalPrint("Sending ML command...")
        # Taken from AR4.py, line XXXX
        # command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
        # Create the command
        command = MoveCommand("ML",pos, moveParameters, J7=extrudeRate)
        #self.root.terminalPrint(str(command)[0:-2])
        # Check if board is not connected or arm is not calibrated
        if self.serialController.boardConnected is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.statusPrint("Command not sent due to no board connected")
            return
        elif self.armCalibrated is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.statusPrint("Command not sent due to arm not calibrated")
            return
        # Send the serial command
        self.awaitingMoveResponse = True # Set the awaiting move response flag 
        self.serialController.sendSerial(str(command))

        #Timeout and feedback handling
        response = self.root.serialController.waitForResponse("POS",timeout)
        
        if response is not None:
            self.processPosition(response)
            #self.root.terminalPrint(response)
            #self.root.statusPrint("Move command executed successfully")
        else:
            #stop print if timed out
            self.root.printController.flag = "timeout after: " + str(timeout) + "s"
        self.awaitingMoveResponse = False

    def sendRJ(self, J1, J2, J3, J4, J5, J6, moveParameters, timeout=10):
        #self.root.terminalPrint("Sending RJ command...")
        # Create the command
        # RJA0B0C0D0E0F0J70J80J90Sp25Ac10Dc10Rm80WNLm000000
        command = MoveCommand("RJ",[J1,J2,J3,J4,J5,J6], moveParameters)
        # Check if a bord is connected or if the arm is not calibrated
        self.nominalCheck()
        if self.serialController.boardConnected is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.statusPrint("Command not sent due to no board connected")
            return
        elif self.armCalibrated is False:
            # Inform user in terminal then quit function to avoid sending instruction
            self.root.statusPrint("Command not sent due to arm not calibrated")
            return
        #print("Command is ", str(command))
        # Send the serial command
        self.serialController.sendSerial(str(command))
        self.awaitingMoveResponse = True # Set the awaiting move response flag 

        #Timeout and feedback handling
        response = self.root.serialController.waitForResponse("POS",timeout)
        
        if response is not None:
            self.processPosition(response)
            self.root.terminalPrint(response)
            self.root.statusPrint("Move command executed successfully")
        self.awaitingMoveResponse = False

    def moveCircle(self):
        pass
    def moveArc(self):
        pass
    #Estimate movetime so timeout can be reasonable, only for ML command
    def estimateMoveTime(self,lastPos,curPos,speed):
        delX = curPos.x-lastPos.x
        delY = curPos.y-lastPos.y
        delZ = curPos.z-lastPos.z
        distance = math.sqrt(delX**2+delY**2+delZ**2)
        estimateTime = distance/speed
        print("Time:", estimateTime, " distance:", distance, " speed:", speed)
        return estimateTime
    
    #custom move linear command that calculates inverse kinematics before sending and uses RJ
    #This function may not be much different but InverseKinematics is needed for printing
    def moveLinearCustom(self, X, Y, Z, Rx, Ry, Rz):
        
        outgoingJointAngles = self.kinematics.solveInverseKinematics([X, Y, Z, Rx, Ry, Rz])
        # TODO Configure a way to use driveMotorsL instead driveMotorsJ
        self.sendRJ(outgoingJointAngles[0], outgoingJointAngles[1], outgoingJointAngles[2], outgoingJointAngles[3], outgoingJointAngles[4], outgoingJointAngles[5])
    
    # Moves the robot to a safe position to be turned off
    def moveSafe(self):
        self.sendRJ(0, -40, 60, 0, 45, 0, self.defaultMoveParameters)
    
    # Moves to the neutral position, all joints at zero degrees
    def moveHome(self):
        self.sendRJ(0,0,0,0,0,0, self.defaultMoveParameters)
    
    
    #endregion move commands
    
    #region ======Testing============
    def toggleLimitTest(self):
        if self.serialController.boardConnected is False:
            self.root.statusPrint("Failed to start limit switch test. No board is connected")
        # Check if we are already testing limit switches
        elif self.testingLimitSwitches:
            self.finishTest = True # Set the flag to finish the test
            self.root.limitTestButton.configure(relief="raised") # Make the button look un-toggled
        # If not, check if we are busy with anything else (self.testingLimitSwitches must be False if here)
        elif self.checkIfAllBusy() is True:
            self.root.statusPrint("Failed to start limit switch test. Arm is busy.")
        # If we reach this, the arm is not busy with anything and we can start the test
        else:
            self.testingLimitSwitches = True # set the flag
            self.root.limitTestButton.configure(relief="ridge") # Make the button look toggled
            self.root.statusPrint("Starting limit switch test")
            limitSwitchThread = threading.Thread(target = self.limitTestUpdate)
            limitSwitchThread.start()

    def limitTestUpdate(self):
        while not self.finishTest:
            self.serialController.sendSerial("TL\n") # Send instruction
            response = self.waitForResponse("TL",2)
            # Check if serial controller has a response ready
            if response is not None:
                # If so, read it in
                self.root.terminalPrint("testing limit switch "+response)
                # Limit switch test will never return an error so we can always directly process
                self.root.J1LimState.config(text=response[response.find('J1')+5:response.find("   J2")].strip())
                self.root.J2LimState.config(text=response[response.find('J2')+5:response.find("   J3")].strip())
                self.root.J3LimState.config(text=response[response.find('J3')+5:response.find("   J4")].strip())
                self.root.J4LimState.config(text=response[response.find('J4')+5:response.find("   J5")].strip())
                self.root.J5LimState.config(text=response[response.find('J5')+5:response.find("   J6")].strip())
                self.root.J6LimState.config(text=response[response.find('J6')+5:].strip())
                self.awaitingTestResponse = False # Reset the flag
                # Check if the user wants to stop the test
                # This is done in here to prevent the program eternally waiting on a response
        self.testingLimitSwitches = False # Reset the test flag
        self.finishTest = False # Reset the finish testing flag
        self.root.statusPrint("Stopping limit switch test")

    def toggleEncoderTest(self):
        if self.serialController.boardConnected is False:
            self.root.statusPrint("Failed to start encoder test. No board is connected")
        # Check if we are already testing encoders
        elif self.testingEncoders:
            self.finishTest = True # Set the flag to finish the test
            self.root.encoderTestButton.configure(relief="raised") # Make the button look un-toggled
        # If not, check if we are busy with anything else (self.testingEncoders must be False if here)
        elif self.checkIfAllBusy():
            self.root.statusPrint("Failed to start encoder test. Arm is busy.")
        # If we reach this, the arm is not busy with anything and we can start the test
        else:
            self.testingEncoders = True # set the flag
            self.root.encoderTestButton.configure(relief="ridge") # Make the button look toggled
            self.root.statusPrint("Starting encoder test")
            # Send a "Set Encoder" instruction to set all values to 1000
            # I don't understand the reasoning behind this but this is what the AR4's encoder test code does so it is being included
            #self.serialController.sendSerial("SE\n") # Commented out for now for true encoder values
            # The "Set Encoder" instruction returns a "Done" except it is done with a print instead of println
            # which makes it so the serial can only be read by a "read" instead of "readline".
            # Therefore, we forcibly tell the serialController that it isn't waiting for a response
            self.serialController.waitingForResponse = False

    def encoderTestUpdate(self):
        if self.awaitingTestResponse is False:
            self.serialController.sendSerial("RE\n") # Send instruction
            self.awaitingTestResponse = True # Set the flag
            return
        # Check if serial controller has a response ready
        if self.serialController.responseReady:
            # If so, read it in
            response = self.serialController.getLastResponse()
            self.root.terminalPrint(response)
            # The "SE" instruction returns 'Done' so we need to watch out for it
            if response == "Done":
                self.awaitingTestResponse = False
                return
            # Encoder test will never return an error so we can always directly process
            self.root.J1EncState.config(text=response[response.find('J1')+5:response.find("   J2")].strip())
            self.root.J2EncState.config(text=response[response.find('J2')+5:response.find("   J3")].strip())
            self.root.J3EncState.config(text=response[response.find('J3')+5:response.find("   J4")].strip())
            self.root.J4EncState.config(text=response[response.find('J4')+5:response.find("   J5")].strip())
            self.root.J5EncState.config(text=response[response.find('J5')+5:response.find("   J6")].strip())
            self.root.J6EncState.config(text=response[response.find('J6')+5:].strip())
            self.awaitingTestResponse = False # Reset the flag
            # Check if the user wants to stop the test
            # This is done in here to prevent the program eternally waiting on a response
            if self.finishTest is True:
                self.testingEncoders = False # Reset the test flag
                self.finishTest = False # Reset the finish testing flag
                self.root.statusPrint("Stopping encoder test")
    #endregion testing
    
    #region Jogs
    def startToolJog(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    def stopToolJog(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    def startCartesianJog(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    def stopCartesianJog(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    def startJointJog(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    def offsetTool(self):
        if self.checkIfAllBusy():
            self.root.terminalPrint("Arm busy")
        pass
    #endregion Jogs

    #region Other Functions
  
    # Checks if any of the flags relating to the arm performing a task are True and if so, return True
    def checkIfBusy(self, message = None):
        value = self.calibrationInProgress or self.awaitingMoveResponse or self.testingLimitSwitches or self.testingEncoders or self.awaitingTestResponse or self.awaitingPosResponse
        if value:
            if message:
                self.root.terminalPrint("Cannot" + message + " Arm busy")
            else:
                self.root.terminalPrint("Arm busy")
        return value
    
    # meant to add calibration check without printing check
    # so move commands can execute while printing
    def nominalCheck(self):
        value = self.checkIfBusy()
        if self.root.serialController.boardConnected is False:
            self.root.warningPrint("No board connected")
            value = True
        if self.armCalibrated is False:
            self.root.warningPrint("Arm not calibrated cannot proceed")
            value = True
        return value
    
    #Do not use this command on calibrate and it is assumed that these check do not need to be done to start calibration
    def checkIfAllBusy(self):
        value = False
        if self.nominalCheck():
            value = True
        #check printer for printing processes
        if self.root.printController.checkIfPrinterBusy():
            value = True
        return value
    
    #This should reset everything in check if busy
    def reset(self):
        self.awaitingMoveResponse = False
        #self.serialController.waitingForResponse = False
        self.calibrationInProgress = False
        self.awaitingPosResponse = False
        self.testingLimitSwitches = False
        self.testingEncoders = False
        self.awaitingTestResponse = False
    
    #endregion other functions
    #region ========|Origin|==================
    #TODO Update this function to work with the origin class
    #TODO improve request position to wait
    def setOrigin(self,origin=None):
        if self.serialController.boardConnected is False:
            self.root.statusPrint("Failed to set origin. No board is connected")
            return
        
        self.root.terminalPrint("Setting current position as origin...")
        #origin can be set by an origin or at current position
        if origin is None:
            if self.checkIfBusy() is True:
                self.root.statusPrint("Failed to set origin. Arm is busy.")
                return
            self.requestPositionAndWait #requests and waits
            self.origin.setOrigin(self.curPos)
            self.root.xCurCoordOrigin.config(text=self.curPos.x)
            self.root.yCurCoordOrigin.config(text=self.curPos.y)
            self.root.zCurCoordOrigin.config(text=self.curPos.z)
        else:
            self.origin=copy.deepcopy(origin)
            self.root.xCurCoordOrigin.config(text=self.curPos.x)
            self.root.yCurCoordOrigin.config(text=self.curPos.y)
            self.root.zCurCoordOrigin.config(text=self.curPos.z)
        
    
    def moveOrigin(self):
        if self.origin.checkOriginSet():
            self.sendMJ(Position(self.origin.x,self.origin.y,self.origin.z,0,90,0, None), self.defaultMoveParameters)


    def updateDeltaFromOrigin(self):
        if not self.origin.checkOriginSet():
            self.root.statusPrint("Origin not set")
        
        [deltaX,deltaY,deltaZ] = self.curPos.GetRelative()[:3]
        self.root.xDeltaOrigin.config(text=deltaX)
        self.root.yDeltaOrigin.config(text=deltaY)
        self.root.zDeltaOrigin.config(text=deltaZ)

    def moveRecommendedOrigin(self):
        self.sendMJ(self.root.printController.recommendedOriginPosition, self.defaultMoveParameters)

    #endregion origin
    #region ========|Test Movements WIP|======
    def lineTest(self):
        uBound = [90,90]
        vBound = [0,90]
        wBound = [0,90]
        x=[-30,-20,-10,0,10,20,30,40]
        y=[0,0,0,0,0,0,0,0]
        z=[0,0,0,0,0,0,0,0]
        xyz = self.relativeToAbsolute(x,y,z)
        for i in range(len(x)):
            outgoingJointAngles = self.kinematics.solveInverseKinematicsFreeUVW(xyz[:][i], uBound, vBound, wBound)
            self.sendRJ(outgoingJointAngles[0], outgoingJointAngles[1], outgoingJointAngles[2], outgoingJointAngles[3], outgoingJointAngles[4], outgoingJointAngles[5])
            wait(2000) #wait 2 seconds between moves
   
    def SquareTest(self):
        #Linear sequencing needs to be implemented for this to work properl
        pass
    
    def dynamicgcodeTest(self, gcodeList):
        pass
    #endregion

    #region ========|Timeouts|===============            
    #if no calibrate response
    def calibrateTimeout(self):
        timeout = 35
        timeInc = 0.1
        timeElapsed = 0
        #while timeout is not reached and still waiting for position response, keep waiting
        while timeElapsed < timeout and (self.calibrationState == 2 or self.calibrationState == 1):
            time.sleep(timeInc)
            timeElapsed += timeInc
        #if no longer calibrating, exit function
        if self.calibrationState == 0 or self.calibrationInProgress == False:
            return
        #if response for stage 1 hasn't come back yet
        if self.calibrationState == 1 or self.calibrationState == 2:
            # If not, inform user of Stage 1 Failure
            self.root.statusPrint(f"Stage 1 Calibration FAILED")
            self.root.terminalPrint("Calibration timed out")
            # Exit calibration
            self.calibrationState = 0
            self.calibrationInProgress = False
            # Force arm calibration flag to False
            self.armCalibrated = False
            return
        timeout = 35
        timeInc = 0.1
        timeElapsed = 0
        #while timeout is not reached and still waiting for position response, keep waiting
        while timeElapsed < timeout and (self.calibrationState == 3 or self.calibrationState == 4):
            time.sleep(timeInc)
            timeElapsed += timeInc
        #special case if stage 2 hasn't been sent yet
        if self.calibrationState == 3:
            time.sleep(2)
        #if response for stage 2 hasn't come back yet
        if self.calibrationState == 4:
            # If not, inform user of Stage 2 Failure
            self.root.statusPrint("Stage 2 Calibration FAILED")
            self.root.terminalPrint("Calibration timed out")
            # Exit calibration
            self.calibrationState = 0
            self.calibrationInProgress = False
            # Force arm calibration flag to False
            self.armCalibrated = False
        self.root.timeoutStartedCal = False
    def processPositionTimeout(self):
        timeout = 5
        timeInc = 0.1
        timeElapsed = 0
        #while timeout is not reached and still waiting for position response, keep waiting
        while timeElapsed < timeout and self.awaitingPosResponse:
            time.sleep(timeInc)
            timeElapsed += timeInc
        if self.awaitingPosResponse is True:
            self.root.terminalPrint("Position response timed out")
            self.root.statusPrint("Position response timed out")
            self.awaitingPosResponse = False
            #
            # self.root.serialController.stopWaitingForReponse()
        self.root.timeoutStartedPos = False

    def moveTimeout(self):
        timeout = 15
        timeInc = 0.1
        timeElapsed = 0
        #while timeout is not reached and still waiting for position response, keep waiting
        while timeElapsed < timeout and self.awaitingMoveResponse:
            time.sleep(timeInc)
            timeElapsed += timeInc
        #If awiating move response is still true, then we know we timed out so inform the user
        if self.awaitingMoveResponse is True:
            self.root.statusPrint("Move response timed out")
            self.awaitingMoveResponse = False
            self.root.timeoutStartedMove = False
            self.root.printController.flag = "Move Timeout"
            #self.root.serialController.stopWaitingForReponse()
            return
        self.root.timeoutStartedMove = False

    #Could be used for encoder test but could be removed
    def testTimeout(self):
        response = self.serialController.waitForResponse("TL",2)
        self.armController.limitTestUpdate(response)
    #endregion Timeouts

#region--- Other Classes ---
class Position:
    def __init__(self, x, y, z, Rx, Ry, Rz, originObj):
        if x is not None and y is not None and z is not None:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.Rx = float(Rx)
            self.Ry = float(Ry)
            self.Rz = float(Rz)
        self.origin = originObj
    def GetAbsolute(self):
        return [self.x, self.y, self.z, self.Rx, self.Ry, self.Rz]
    
    def GetRelative(self):
        if self.origin is not None and self.origin.originSet == True:
            relX = self.x - self.origin.x
            relY = self.y - self.origin.y
            #print("test for z and origin of z", self.z,self.origin.z)
            relZ = self.z - self.origin.z
            return [relX, relY, relZ, self.Rx, self.Ry, self.Rz]
        else:
            return [None*6]
    def SetRelative(self,relX,relY,relZ,Rx,Ry,Rz):
        if self.origin.originSet and self.origin is not None:
            self.x = float(relX) + self.origin.x
            self.y = float(relY) + self.origin.y
            self.z = float(relZ) + self.origin.z
            self.Rx = Rx
            self.Ry = Ry
            self.Rz = Rz

    def SetPosition(self, x, y, z, Rx, Ry, Rz):
        self.x = x
        self.y = y
        self.z = z
        self.Rx = Rx
        self.Ry = Ry
        self.Rz = Rz

class Joints:
    def __init__(self):
        pass
class Origin:
    def __init__(self, x, y, z):
        if x is not None and y is not None and z is not None:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.originSet = True
        else:
            self.originSet = False
    def getOrigin(self):
        return [self.x, self.y, self.z]
    def setOrigin(self, position):
        self.x = float(position.x)
        self.y = float(position.y)
        self.z = float(position.z)
        self.originSet = True
    def checkOriginSet(self):
        return self.originSet
    #convert origin to a position object
    def toPosition(self):
        origin = copy.deepcopy(self)
        pos = Position(self.x,self.y,self.z,0,90,0,origin) #standard R position
        return pos
    
class MoveParameters:
    def __init__ (self,speed,acceleration,deceleration,ramp,loopMode,speedType):
        self.speed = speed
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.ramp = ramp
        self.loopMode = loopMode
        self.speedType = speedType
    def setLoopMode(self,loopMode):
        self.loopMode = loopMode
    def getLoopMode(self):
        return self.loopMode
#move command information, does not send a move command
class MoveCommand:
    def __init__(self,type,jointsOrPosition, moveParameters, J7=0):
        self.jointsOrPosition = jointsOrPosition
        if isinstance(jointsOrPosition, Position):
            self.A = jointsOrPosition.x
            self.B = jointsOrPosition.y
            self.C = jointsOrPosition.z
            self.F = jointsOrPosition.Rx
            self.E = jointsOrPosition.Ry
            self.D = jointsOrPosition.Rz
        else:
            self.A = jointsOrPosition[0]
            self.B = jointsOrPosition[1]
            self.C = jointsOrPosition[2]
            self.D = jointsOrPosition[3]
            self.E = jointsOrPosition[4]
            self.F = jointsOrPosition[5]
        if J7 != None:
            self.J7 = J7
        else:
            self.J7 = 0
        self.moveParameters = moveParameters
        self.type = type
        #debugging print("type is: ",self.type, type)
    def __str__(self):
        command = "Error"
        if self.type=="ML":
            command = f"MLX{self.A}Y{self.B}Z{self.C}Rz{self.D}Ry{self.E}Rx{self.F}J7{self.J7}J80.00J90.00S{self.moveParameters.speedType}{self.moveParameters.speed}Ac{self.moveParameters.acceleration}Dc{self.moveParameters.deceleration}Rm{self.moveParameters.ramp}Rnd0WFLm{self.moveParameters.loopMode*6}Q0\n"
        elif self.type=="MJ":
            command = f"MJX{self.A}Y{self.B}Z{self.C}Rz{self.D}Ry{self.E}Rx{self.F}J7{self.J7}J80.00J90.00S{self.moveParameters.speedType}{self.moveParameters.speed}Ac{self.moveParameters.acceleration}Dc{self.moveParameters.deceleration}Rm{self.moveParameters.ramp}Rnd0WFLm{self.moveParameters.loopMode*6}Q0\n"
        elif self.type=="RJ":
            command = f"RJA{self.A}B{self.B}C{self.C}D{self.D}E{self.E}F{self.F}J7{self.J7}J80.00J90.00S{self.moveParameters.speedType}{self.moveParameters.speed}Ac{self.moveParameters.acceleration}Dc{self.moveParameters.deceleration}Rm{self.moveParameters.ramp}WNLm{self.moveParameters.loopMode*6}\n"
        return command
#endregion other classes