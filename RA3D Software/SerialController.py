import serial
import serial.tools.list_ports
from serial import SerialException
import time
import threading
import queue

# TODO: Interesting bug that occurs but when connecting, disconnecting, then reconnecting, the entire serial buffer gets messed up resulting in everything being read in as junk. Not sure how it is happening nor how to fix.

class SerialController:

    def __init__(self, root):
        self.root = root

        # General Connection Variables
        self.port = ""
        self.baud = 9600
        self.board = None
        self.boardConnected = False
        self.running = False#To keep thread running only while connceted
        # Threading Variables
        self.serialThread = None
        self.responseQueue = queue.Queue()
        self.responseReady = False # Used as a flag to check if a response is ready to be read
        self.lastResponse = None # Stores whatever the last response was from serial until requested

    # Handles the "Connect/Disconnect" button being pressed to connect or disconnect the port
    def serialConnect(self):
        # Check if the serial controller is already connected to a board
        if self.boardConnected == False:
            # If not, connect to the port currently selected in the port dropdown
            self.root.statusPrint("Attempting port connection")
            self.connectPort(self.root.portSelection.get())
            self.root.statusPrint(f"Connection Status: {self.boardConnected}")

            # Perform a check to see if board was actually connected to
            if self.boardConnected:
                self.root.connectButton.config(text="Disconnect") # Change button text
                self.root.portDropdown.config(state="disabled") # Disable port dropdown
                self.root.refreshCOMButton.config(state="disabled") # Disable refresh button
                self.root.portStatusLabel.config(text="Status: Connected") # Change port status text
        else:
            # If so, disconnect from the port
            self.root.statusPrint("Disconnecting from port")
            self.disconnectPort()
            self.root.statusPrint(f"Connection Status: {self.boardConnected}")
            self.root.connectButton.config(text="Connect") # Change button text
            self.root.portDropdown.config(state="readonly") # Enable port dropdown
            self.root.refreshCOMButton.config(state="normal") # Enable refresh button
            self.root.portStatusLabel.config(text="Status: Disconnected") # Change port status text

    def connectPort(self, port):
        self.port = port
        try:
            ### TODO: Edge case not caught by this try is if connecting to port, 
            # unplug & replug board on same port, disconnect, 
            # then connecting again doesn't throw error even though it 
            # doesn't actually connect to the port and essentially bricks 
            # the program from connecting to another port until restarted

            tempBoard = serial.Serial(self.port, self.baud)
            self.root.statusPrint(f"Port {port} opened successfully")
            self.board = tempBoard
            self.board.reset_output_buffer()
            self.board.reset_input_buffer()

            time.sleep(2)  # allow Arduino auto-reset
            self.boardConnected = True
            self.responseQueue = queue.Queue()

            self.serialThread = threading.Thread(target=self.serialReader, daemon=True)
            #self.sortThread = threading.Thread(target=self.sortResponseThread, daemon=True)
            self.running = True #Keeps threads running, must be true before starting serialthread
            self.serialThread.start()
            #self.sortThread.start()
            self.processResponses() #start sorting resposnes
            
            
        except SerialException:
            self.root.statusPrint(f"Failed to open port: {port}")

    def disconnectPort(self):
        if self.boardConnected == True:
            self.running = False
            time.sleep(0.1)  # let threads exit cleanly

            self.sendSerial("CL")
            try:
                self.board.reset_input_buffer()
                self.board.reset_output_buffer()
                self.board.close()
            except:
                pass
            self.boardConnected = False
            

    # Repeatedly checks the serial port for new responses
    def serialReader(self):
        while True:#self.running:
            # Check if a board is connected
            #print("serial thread running")
            if self.boardConnected:
                try:
                   # If there are any bytes of data waiting...
                    if self.board.in_waiting > 0:
                        # Read them in and store them in the response queue
                        # errors ignore prevents decode crashes from partial UTF-8 bytes.D
                        data = self.board.readline().decode('utf-8', errors='ignore').strip()
                        #if data
                        if data:
                            self.responseQueue.put(data)
                except:
                    pass
            else:
                time.sleep(0.05)
            #self.checkResponseQueue()
            #Debugging
            if not self.responseQueue.empty():
                #print("Response Queue:",list(self.responseQueue.queue))
                pass

    def cleanQueue(self, item):
        size = int(self.responseQueue.qsize())
        for i in range(size):
            if self.responseQueue.queue[i] == item:
                self.responseQueue.queue.remove(i)
        print("response queue cleaned")

    #Advances the response queue every .01 seconds    
    def processResponses(self):
        while not self.responseQueue.empty() and not self.waiting:
            response = self.responseQueue.get()
            self.sortResponse(response)
        #exit this thread if serial is not connected
        if not self.running:
            return
        #print("processing responses")
        self.root.after(100, self.processResponses)

    #Function process response because correct response is not guaranteed for a command
    #reponse must be passed back to some functions because the response is out of the queue
    def sortResponse(self, sortResponse):
        #shorthand for ease
        PC = self.root.printController
        AC = self.root.armController
        R = self.root
        flag = None
        # TODO: Add in a message/action for every response and edit teensy to send more information
        # TODO I will change this so that only unexpected responses or errors are processed here
        
        #self.root.terminalPrint(f"Received Response: {sortResponse}")
        self.root.terminalPrint("Sorting response...")
        if sortResponse[:5]== "Estop":
            PC.printing = False
            flag = "Estop"
            R.warningPrint("Estop pushed, stopping print")
        if sortResponse[:2]== "ER":
            R.statusPrint(f"Kinematic Error: {sortResponse[2:]}")
            flag = "Kinematic Error"
            AC.awaitingMoveResponse = False
            #Have to clean queue because the arm sends a thousand ERs for some reason
            self.cleanQueue("ER")
        elif sortResponse[:2] == "EL":
            R.statusPrint(f"Error Axis Fault, Out of Reach: {sortResponse[2:]}")
            flag = "Axis Fault"
            AC.awaitingMoveResponse = False
        elif sortResponse[:2] == "TL":
            if AC.testingLimitSwitches:
                #Limit switch test
                R.terminalPrint(f"Received TL Response: {sortResponse}")
                AC.limitTestUpdate(response=sortResponse)
            else:
                R.terminalPrint(f"Received Unexpected Response: {sortResponse}")
        elif sortResponse[:2] == "RE":
            #read encoders
            pass
        elif sortResponse[:4] == "Done":
            #Home position finished
            #Command set output on/off finished
            #send position to arm
            pass
        elif sortResponse[:6] == "WTDone":
            R.statusPrint("Wait command finished")
        elif sortResponse[:4] == "echo":
            R.statusPrint(f"Echo: {sortResponse[4:]}")
        elif sortResponse == "\n":
            pass
        elif sortResponse == "Turn Hazard Move Stopped":
            self.root.statusPrint(f"Encountered Hazard Move Stopped: {sortResponse[2:]}")
            self.root.warningPrint(f"Turn Hazard Encountered. Stopping Print")
            #flag = "Turn Hazard"
            AC.awaitingMoveResponse = False
        elif sortResponse[:3] == "POS" and AC.calibrationInProgress:
            self.root.statusPrint("Position received from arm during calibration")
            AC.calibrateArmUpdate(response=sortResponse)
        elif sortResponse[:3] == "POS" and AC.awaitingMoveResponse:
            self.root.statusPrint("Position received from arm after move command")
            #AC.moveUpdate(response=sortResponse)
        elif sortResponse[:3] == "POS" and AC.awaitingPosResponse:
            self.root.statusPrint("Position received from arm after position request")
            AC.requestPositionUpdate(response=sortResponse)
        elif sortResponse[:3] == "POS":
            AC.processPosition(sortResponse)
        else:
            R.statusPrint(f"Received Unrecognized Response: {sortResponse}")
        if flag is not None:
            PC.flag = flag
    
    
    def getNextResponse(self):
        try:
            return self.responseQueue.get_nowait()
        except queue.Empty:
            return None
    
    #There are 2 ways responses are porcessed, they are processes at an interval by processResponse or waited for
    #Then the response is sorted and
    #wait for a specific response but timeout if not recieved, standard timeout is 2 seconds
    def waitForResponse(self, prefix, timeout=2):
        start = time.time()
        self.waiting = True #so response is not taken from processReponses
        while time.time() - start < timeout:
            try:
                response = self.responseQueue.get(timeout=0.1)
                if response.startswith(prefix):
                    #self.root.terminalPrint("response processed after waiting")
                    self.waiting = False
                    return response
            except queue.Empty:
                pass
        self.waiting = False
        return None
    
    # Sends a string over serial to the connected port
    def sendSerial(self, command):
        # Check if a board is connected
        if self.boardConnected == False:
            self.root.statusPrint("Failed to send command: No board connected")
            return
        #self.board.reset_input_buffer()
        # If we aren't awaiting for a serial response, then send the command
        #self.root.statusPrint(f"Sending Command: {command}")
        self.board.write(command.encode())
        #self.root.terminalPrint("Command sent")
        # Reset the input buffer
        #self.board.reset_input_buffer()
    # stop waiting for response if something timedout
  
    def refreshCOMPorts(self):
        self.root.portList = self.getCOMPorts() # Create options list from found COM ports
        self.root.portSelection.set("Select Port")
        self.root.portDropdown["values"] = self.root.portList
        self.root.portDropdown["textvariable"] = self.root.portSelection

    # Returns a list of all COM ports with a device connected
    def getCOMPorts(self):
        ports = list(serial.tools.list_ports.comports())
        returnList = []
        if not ports:
            self.root.statusPrint("No serial ports w/ connected devices found")
        else:
            self.root.statusPrint(f"Found {len(ports)} connected device(s):")
            for port in ports:
                self.root.terminalPrint(f"Name: {port.device}\nDescription: {port.description}\nID: {port.hwid}\n\n")
                returnList.append(port.device)
        return returnList
