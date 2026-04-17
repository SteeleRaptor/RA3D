from tkinter import *
from tkinter import messagebox
import tkinter.ttk as ttk
import threading
import time
from datetime import datetime
import RPi.GPIO as GPIO
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

from SerialController import SerialController
from ArmController import ArmController, MoveParameters
from PrintController import PrintController
from TemperatureController import TemperatureController

class TkWindow(Tk):

    # region init
    def __init__(self):
        Tk.__init__(self)
        #All Settings
        #Maps each display setting to a variable
        #Simply add to this dictionary to add a setting
        # "Display Name":("VariableName","Object")
        self.settingsDict = {
            "Default Print speed (mm/s)": ("speed","PC"), #print controller
            "Acceleration": ("acceleration","PC"),
            "Decceleration": ("decceleration","PC"),
            "Ramp": ("ramp","PC"),
            "Default Plate Height" : ("plateHeight","PC"),
            "Ignore Flags" : ("ignoreFlags","PC"),
            "deg/mm": ("extruder_deg_per_mm","AC"),
            "Sync Print Parameters": ("syncWithPrintParameters","AC"), #arm controller
            "Debug Mode": ("DebugMode","self"),
            "Print Debug Mode": ("PrintDebugMode", "self"),
            "Current Gcode Feedrate" : ("feedRate", "PC"),
            "Printing Timeout Extra": ("timeoutExtra","PC"),
            "Hard code printer speed to default": ("hardCodePrinterSpeed","PC"),
            "Solid LED": ("LEDOn","self"),
            "Heated Filament Multiplier": ("heatedFilamentMultiplier","AC"),
            "Coolend Mode": ("coolendMode","self"),
            "Drop Height": ("dropHeight","PC"),
            "Default Angle": ("defaultAngle","PC")
        }
        self.DebugMode = True #Will display important debug prints but not all of them
        self.PrintDebugMode = True #Will display gcode lines and print coordinates
        #Debug modes off will NOT hide errors

        self.coolendMode = False #use for testing the arm and extrusion without requiring the hotend to be at temperature

        #Most cases for the blinking the LED should be shown by displaying a warning print
        self.BlinkLED = False #Blink the LED when there is a problem
        self.LEDOn = False #LED stays On, signifies in progress, overridden by blinkLED
        self.ledThreadRunning = True # Used to stop the LED thread
        self.unstoppableThreads = ["Serial Read Thread", "Serial Sort Thread","LED Thread","MainThread", "Thread-1"] #Threads that will not be stopped on reset
        self.LEDPin = 36
        # Initialize the GPIO
        GPIO.setmode(GPIO.BOARD)
        # Initialize the LED
        GPIO.setup(self.LEDPin, GPIO.OUT)

        # Colors
        # Taken from https://brand.uccs.edu/visual-guidelines/color on April 11th, 2026
        self.colorCUBlack = "#000000"
        self.colorCUGold = "#cfb87c"
        self.colorDarkGray = "#565a5c"
        self.colorLightGray = "#a2a4a3"
        self.colorWhite = "#ffffff"
        # Variables used to generalize colorations across the software
        self.colorBG = self.colorCUBlack    # Background color
        self.colorBG2 = self.colorDarkGray  # Background 2 color (used for backgrounds that should stick out more)
        self.colorBG3 = self.colorLightGray # Background 3 color (used for even brighter backgrounds)
        self.colorFG = self.colorWhite      # Foreground color (like text)
        self.colorAccent = self.colorCUGold # Accent colors

        # Style configurations
        s = ttk.Style()
        s.theme_use('default')
        s.configure("TProgressbar", 
                    thickness = 50,                 # Height of the bar
                    background  = self.colorAccent, # Color of the bar that is filled in
                    troughcolor = self.colorBG3,    # Color of the bar that isn't filled in
                    pbarrelief = SOLID)
        s.configure("TSeparator", background=self.colorBG2)
        # s.configure("TCombobox",
        #             foreground=self.colorFG,
        #             background=self.colorBG2,
        #             selectbackground=self.colorBG2,
        #             fieldbackground=self.colorBG2)

        self.root = self
        # Set the window title
        self.title("RA3D Control Software")
        # (TEMP) Set window as topmost
        # self.attributes('-topmost', True)
        self.updateDelay = 150 # Delay between update function calls in milliseconds
        # Set the window dimensions and position on screen
        w = 1220 # Window width
        h = 685 # Window height
        ws = self.winfo_screenwidth() # Get screen width
        hs = self.winfo_screenheight() # Get screen height
        x = int((ws/2) - (w/2)) # Calculate x position for window to be in the center of the screen
        y = int((hs/2) - (h/2)) # Calculate y position for window to be in the center of the screen
        self.geometry(f"{w}x{h}+{x}+{y}") # Set the width, height, x, and y values
        self.root.config(bg=self.colorBG)
        
        # Instantiate objects for the various controller classes
        self.serialController      = SerialController(self.root)
        self.armController         = ArmController(self.root, self.serialController)
        self.printController       = PrintController(self.root, self.armController)
        self.temperatureController = TemperatureController(self.root)

        # Create and draw widgets onto the window
        self.createTabs()
        if self.DebugMode:
            self.root.terminalPrint("GUI created")

        self.serialController.refreshCOMPorts()
       
        
        self.root.protocol("WM_DELETE_WINDOW", self.shutdownProgram)

        self.printThreadStarted = False
        #Set origin last so everything is in place
        self.armController.setOrigin(origin=self.root.printController.recommendedOrigin)
         
        self.ledThread = threading.Thread(target=self.updateLED, daemon=True,name="LED Thread")
        self.ledThread.start()
        self.serialController.autoConnect()
        self.root.bind("<Button-1>", self.clearEntryFocus)
        # Set up a call to the update function after updateDelay milliseconds
        self.update()
        self.printController.autoSelectFile("../Programming Experiments/Gcode Files/one_layer.gcode") #for testing purposes, can change to None to not auto select

    #endregion init

    #region Shutdown
    # This function is meant to do various shutdown tasks so the program doesn't break anything
    def shutdownProgram(self):
        try:
            # Move the arm to a safe position before shutting down, will not move safe if not calibrated
            # This is necessary because if the arm is not calibrated
            if not self.armController.calibrationOverridden:
                self.armController.moveSafe()
            self.ledThreadRunning = False
            # Release the GPIO pins from use
            GPIO.cleanup()
            # Close the window
            self.root.destroy()
        except:
            #if failed then destroy the window anyway, we don't want it to be stuck
            self.root.destroy()
    #endregion

    #region Tabs
    # Creates the interface tabs
    def createTabs(self):
        # Create a notebook to manage the tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True)
        # Create the tabs
        self.homeTab = Frame(self.notebook, bg=self.colorBG)
        self.armTab = Frame(self.notebook, bg=self.colorBG)
        self.toolTab = Frame(self.notebook, bg=self.colorBG)
        self.debugTab = Frame(self.notebook, bg=self.colorBG)
        self.settingsTab = Frame(self.notebook, bg=self.colorBG)
        # Put the tabs on screen
        self.homeTab.pack(fill="both", expand=True)
        self.armTab.pack(fill="both", expand=True)
        self.toolTab.pack(fill="both", expand=True)
        self.debugTab.pack(fill="both", expand=True)
        self.settingsTab.pack(fill="both", expand=True)
        # Add the tabs to the notebook
        self.notebook.add(self.homeTab, text="Home")
        self.notebook.add(self.armTab, text="Arm Control")
        self.notebook.add(self.toolTab, text="Extruder")
        self.notebook.add(self.debugTab, text="Debug")
        self.notebook.add(self.settingsTab, text="Settings")
        # Call the various functions for creating the widgets in each tab
        self.fillHomeTab()
        self.fillArmTab()
        self.fillToolTab()
        self.fillDebugTab()
        self.fillSettingsTab()

        # Create a status label for immediate user response
        self.statusBarFrame = Frame(self.root, height=20, bg=self.colorAccent)
        self.statusBarFrame.pack(fill="both", expand=True, side="bottom")
        self.statusLabel = Label(self.statusBarFrame, text="Status: Example", bg=self.colorAccent, fg=self.colorBG)
        self.statusLabel.grid(row=0, column=0, sticky=W+N+S)

    def fillHomeTab(self):
        # ==========| Print Info Frame |==========
        printInfoWidth = 600
        self.printInfoHomeFrame = Frame(self.homeTab, bg=self.colorBG2, width=printInfoWidth, height=350)
        self.printInfoHomeFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+E+N+S)
        # Print status label (Printing, paused, etc.)
        self.printStatusHomeLabel = Label(self.printInfoHomeFrame, text="IDLE...", font=("TkDefaultFont", 30, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.printStatusHomeLabel.grid(row=0, column=0, columnspan=3, rowspan=2, padx=5, pady=5, sticky=N+W)
        self.printStatusHomeLabel.bind("<Button-1>", self.windowDimensions)
        # Start button
        self.startPrintHomeButton = Button(self.printInfoHomeFrame, text="Start", width=10, height=2, command=self.printController.startPrint, state="disabled", bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.startPrintHomeButton.grid(row=0, column=3, padx=5, pady=5, sticky=E)
        # Pause button
        self.pausePrintHomeButton = Button(self.printInfoHomeFrame, text="Pause", width=10, height=2, command=self.printController.pausePrint, state="disabled", bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.pausePrintHomeButton.grid(row=1, column=3, padx=5, pady=5, sticky=E)
        # Stop button
        self.stopPrintHomeButton = Button(self.printInfoHomeFrame, text="Stop", width=10, height=2, command=self.printController.cancelPrint, state="disabled", bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.stopPrintHomeButton.grid(row=2, column=3, padx=5, pady=5, sticky=E)
        # Select file button
        self.selectFileHomeButton = Button(self.printInfoHomeFrame, text="Select File", width=10, command=self.printController.selectFile, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.selectFileHomeButton.grid(row=4, column=0, padx=5, pady=5, sticky=W)
        # File name label
        self.selectedFileHomeLabel = Label(self.printInfoHomeFrame, text="Please select a file", bg=self.colorBG2, fg=self.colorFG)
        self.selectedFileHomeLabel.grid(row=4, column=1, padx=5, pady=5, sticky=W)
        # Progress label
        self.progressHomeLabel = Label(self.printInfoHomeFrame, text="0%", bg=self.colorBG2, fg=self.colorFG)
        self.progressHomeLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W+S)
        # Progress bar
        self.printProgressBarHome = ttk.Progressbar(self.printInfoHomeFrame, orient=HORIZONTAL, length=printInfoWidth - 20, mode="determinate")
        self.printProgressBarHome.grid(row=3, column=0, columnspan=4, padx=10, pady=5, sticky=W+E)
        self.printProgressBarHome['value'] = 50

        self.currentJ7 = Label(self.printInfoHomeFrame,text="Extruded: 0 mm")
        self.currentJ7.grid(row=6,column=0, padx=5, pady=5, sticky=N+S)

        # ==========| Thermals Frame |==========
        self.thermalsHomeFrame = Frame(self.homeTab, width=printInfoWidth, height=350, bg=self.colorBG2)
        self.thermalsHomeFrame.grid(row=0, column=1, padx=5, pady=5, sticky=W+E+N+S)
        self.thermalsHomeLabel = Label(self.thermalsHomeFrame, text="Thermals", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.thermalsHomeLabel.grid(row=0, column=0, columnspan=2, sticky=W+N+S, padx=5, pady=5)
        ttk.Separator(self.thermalsHomeFrame, orient='horizontal').grid(row=1, column=0, columnspan=6, sticky=W+E)
        self.thermalNameLabel = Label(self.thermalsHomeFrame, text="Name", bg=self.colorBG2, fg=self.colorFG)
        self.thermalNameLabel.grid(row=2, column=0, columnspan=2, sticky=W, padx=5, pady=5)
        self.thermalActualLabel = Label(self.thermalsHomeFrame, text="Actual (°C)", bg=self.colorBG2, fg=self.colorFG)
        self.thermalActualLabel.grid(row=2, column=2, sticky=E, padx=5, pady=5)
        self.thermalTargetLabel = Label(self.thermalsHomeFrame, text="Target (°C)", bg=self.colorBG2, fg=self.colorFG)
        self.thermalTargetLabel.grid(row=2, column=3, sticky=E, padx=5, pady=5)
        self.thermalEnabledLabel = Label(self.thermalsHomeFrame, text="Enabled", bg=self.colorBG2, fg=self.colorFG)
        self.thermalEnabledLabel.grid(row=2, column=4, sticky=E, padx=5, pady=5)
        ttk.Separator(self.thermalsHomeFrame, orient='horizontal').grid(row=3, column=0, columnspan=6, sticky=W+E)
        # Hotend temp info
        self.hotendHomeLabel = Label(self.thermalsHomeFrame, text="Hotend", bg=self.colorBG2, fg=self.colorFG)
        self.hotendHomeLabel.grid(row=4, column=0, sticky=W, padx=5, pady=5)
        self.hotendHomeActual = Label(self.thermalsHomeFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG)
        self.hotendHomeActual.grid(row=4, column=2, sticky=E, padx=5, pady=5)
        #self.hotendHomeTarget = Label(self.thermalsHomeFrame, text="xxx")
        self.hotendHomeTarget = Entry(self.thermalsHomeFrame, width=5, justify="right", bg=self.colorBG2, fg=self.colorFG)
        self.hotendHomeTarget.insert(0, "0")
        self.hotendHomeTarget.grid(row=4, column=3, sticky=E, padx=5, pady=5)
        self.hotendHomeEnableButton = Button(self.thermalsHomeFrame, text="OFF", width=4, command=self.temperatureController.enableHotendControl, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.hotendHomeEnableButton.grid(row=4, column=4, sticky=E, padx=5, pady=5)
        self.hotendHomeLegend = Label(self.thermalsHomeFrame, text=" ", bg="#FF0000", width=2)
        self.hotendHomeLegend.grid(row=4, column=5, padx=5, pady=5, sticky=W)

        # Bed temp info
        self.bedHomeLabel = Label(self.thermalsHomeFrame, text="Bed", bg=self.colorBG2, fg=self.colorFG)
        self.bedHomeLabel.grid(row=5, column=0, sticky=W, padx=5, pady=5)
        self.bedHomeActual = Label(self.thermalsHomeFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG)
        self.bedHomeActual.grid(row=5, column=2, sticky=E, padx=5, pady=5)
        #self.bedHomeTarget = Label(self.thermalsHomeFrame, text="xxx")
        self.bedHomeTarget = Entry(self.thermalsHomeFrame, width=5, justify="right", bg=self.colorBG2, fg=self.colorFG)
        self.bedHomeTarget.insert(0, "0")
        self.bedHomeTarget.grid(row=5, column=3, sticky=E, padx=5, pady=5)
        self.bedHomeEnableButton = Button(self.thermalsHomeFrame, text="OFF", width=4, command=self.temperatureController.enableBedControl, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.bedHomeEnableButton.grid(row=5, column=4, sticky=E, padx=5, pady=5)
        self.bedHomeLegend = Label(self.thermalsHomeFrame, text=" ", bg="#0000FF", width=2)
        self.bedHomeLegend.grid(row=5, column=5, padx=5, pady=5, sticky=W)
        
        # Live temperatures plot
        ttk.Separator(self.thermalsHomeFrame, orient='horizontal').grid(row=6, column=0, columnspan=6, sticky=W+E)
        self.thermalFig = Figure(figsize=(5.75, 2.5), dpi=100)
        self.thermalPlot = self.thermalFig.add_subplot(111)
        self.thermalHotendLine, = self.thermalPlot.plot([], [], 'r-', label="Hotend")
        self.thermalBedLine, = self.thermalPlot.plot([], [], 'b-', label="Bed")
        self.thermalCanvas = FigureCanvasTkAgg(self.thermalFig, master=self.thermalsHomeFrame)
        self.thermalCanvas.get_tk_widget().grid(row=7, column=0, columnspan=6, sticky=W+E+N+S, padx=5, pady=5)
        self.thermalPlot.set_ylabel("Temperature (C)")
        self.thermalPlot.set_xlabel("Time (s)")
        self.thermalPlot.margins(x=0.1, y=0.05)
        self.thermalPlot.spines['right'].set_visible(False)
        self.thermalPlot.spines['top'].set_visible(False)
        # Lots of color setting
        self.thermalFig.set_facecolor(self.thermalsHomeFrame.cget("bg"))
        self.thermalPlot.spines["left"].set_color(self.colorFG)
        self.thermalPlot.spines["bottom"].set_color(self.colorFG)
        self.thermalPlot.set_facecolor(self.colorBG3)
        self.thermalPlot.tick_params(axis='x', colors=self.colorFG)
        self.thermalPlot.tick_params(axis='y', colors=self.colorFG)
        self.thermalPlot.xaxis.label.set_color(self.colorFG)
        self.thermalPlot.yaxis.label.set_color(self.colorFG)
        self.thermalPlot.grid(True) # Add a grid
        self.hotendTargetLine = self.thermalPlot.axhline(y=self.temperatureController.hotendTargetTemp, color='r', linestyle='--')
        self.bedTargetLine = self.thermalPlot.axhline(y=self.temperatureController.bedTargetTemp, color='b', linestyle='--')
        self.yMax = 100 # Used for y-axis scaling of the plot
        self.thermalCanvas.get_tk_widget().bind("<Button-1>", self.clearEntryFocus)

        # ==========| Arm Info Frame |==========
        self.armInfoHomeFrame = Frame(self.homeTab, width=600, height=200, bg=self.colorBG2)
        self.armInfoHomeFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W+E+N+S)
        Label(self.armInfoHomeFrame, text="Arm Info", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG).grid(row=0, column=0, columnspan=2, sticky=W+N+S, padx=5, pady=5)
        ttk.Separator(self.armInfoHomeFrame, orient='horizontal').grid(row=1, column=0, columnspan=4, sticky=W+E)
        self.connectedStatusHome = Label(self.armInfoHomeFrame, text="Arm Disconnected", width=30, justify=LEFT, anchor=W, bg=self.colorBG2)
        self.connectedStatusHome.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        self.calibrationStatusHome = Label(self.armInfoHomeFrame, text="Arm NOT Calibrated", width=30, justify=LEFT, anchor=W, bg=self.colorBG2)
        self.calibrationStatusHome.grid(row=3, column=0, rowspan=2, padx=5, pady=5, sticky=W)
        self.calibrationHomeButton = Button(self.armInfoHomeFrame, text="Calibrate", command=self.armController.startArmCalibration, width=10, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calibrationHomeButton.grid(row=5, column=0, padx=5, pady=5, sticky=W)

        ttk.Separator(self.armInfoHomeFrame, orient='vertical').grid(row=2, column=1, rowspan=5, sticky=N+S)
        Label(self.armInfoHomeFrame, text="Bed Leveling", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG).grid(row=2, column=2, columnspan=2, sticky=W+N+E, padx=5, pady=5)
        ttk.Separator(self.armInfoHomeFrame, orient='horizontal').grid(row=3, column=2, columnspan=2, sticky=W+E)
        self.startLevelHome = Button(self.armInfoHomeFrame, text="Start Level", width=15, command=self.printController.startPrintBedCalibration, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.startLevelHome.grid(row=4, column=2, padx=5, pady=5)
        self.nextLevelHome = Button(self.armInfoHomeFrame, text= "Next Corner", width=15, command=self.printController.nextBedCalibration, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.nextLevelHome.grid(row=4, column=3, padx=5, pady=5)
        self.sweepCornersHome = Button(self.armInfoHomeFrame, text= "Sweep Corners", width=15, command=self.printController.startCornerSweep, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.sweepCornersHome.grid(row=5, column=2, padx=5, pady=5)
        self.cornerLabelHome = Label(self.armInfoHomeFrame, text="Current corner: N/A", bg=self.colorBG2, fg=self.colorFG)
        self.cornerLabelHome.grid(row=5, column=3, padx=5, pady=5)
        self.sweepCornersFullHome = Button(self.armInfoHomeFrame, text= "Full Corner Sweep", width=15, command=self.printController.startFullCornerSweep, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.sweepCornersFullHome.grid(row=6, column=2, padx=5, pady=5)
        self.cancelAnyHome = Button(self.armInfoHomeFrame, text= "Cancel Any", width=15, command=self.printController.cancelAny, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.cancelAnyHome.grid(row=6, column=3, padx=5, pady=5)

        # ==========| Credits Frame |==========
        self.creditsHomeFrame = Frame(self.homeTab, width=600, height=200, bg=self.colorBG2)
        self.creditsHomeFrame.grid(row=1, column=1, padx=5, pady=5, sticky=W+E+N+S)
        Label(self.creditsHomeFrame,text="RA3D",font=("TkDefaultFont", 20, "bold"), bg=self.colorBG2, fg=self.colorFG).grid(row=0,column=0,sticky=EW)
        CreditsText1 = "Designed and implemented by the RA3D Team (Robotic Arm 3D)"
        CreditsText2 = "Team Members: Justin Fauson, Cody Blough, Jonathan Dinan, Mateo Osorio, \n and Jonathan Pederson"
        CreditsText3 = "Sponsor: Dr. Sezer Ozerinc"
        Label(self.creditsHomeFrame,text=CreditsText1, bg=self.colorBG2, fg=self.colorFG).grid(row=1,column=0,sticky=W, padx=5)
        Label(self.creditsHomeFrame,text=CreditsText2,justify="left", bg=self.colorBG2, fg=self.colorFG).grid(row=2,column=0,sticky=W, padx=5)
        Label(self.creditsHomeFrame, text=CreditsText3,justify="left", bg=self.colorBG2, fg=self.colorFG).grid(row=3,column=0,sticky=W, padx=5)

    def fillArmTab(self):
        # ==========| Serial Frame |==========
        self.serialFrame = Frame(self.armTab, bg=self.colorBG2)
        self.serialFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+N+E+S)
        # Create label for Serial Frame
        self.serialLabel = Label(self.serialFrame, text="Serial Port", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.serialLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Refresh button
        self.refreshCOMButton = Button(self.serialFrame, text="⟳", command=self.serialController.refreshCOMPorts, width=2, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.refreshCOMButton.grid(row=1, column=0, padx=5, pady=5)
        # Create dropdown list of all serial COM ports
        self.portList = [] # Start with blank list
        self.portSelection = StringVar(value="Select Port") # Create a StringVar to hold the current dropdown selection
        self.portSelection.trace("w", self.portSelectionChanged) # Connect portSelection changing to a function call to detect when the selected option changes
        self.portDropdown = ttk.Combobox(self.serialFrame, width=10, textvariable=self.portSelection, state="readonly") # Create the dropdown
        self.portDropdown["values"] = self.portList
        self.portDropdown.grid(row=1, column=1, padx=5, pady=5)
        
        # Create button for connecting to port
        self.connectButton = Button(self.serialFrame, text="Connect", command=self.serialController.serialConnect, width=10, state="disabled", bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.connectButton.grid(row=1, column=2, padx=5, pady=5)
        # Status label
        self.portStatusLabel = Label(self.serialFrame, text="Status: Disconnected", bg=self.colorBG2, fg=self.colorFG)
        self.portStatusLabel.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Reset button
        self.resetButton = Button(self.serialFrame, text="Reset", command=self.armController.reset, width=10, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.resetButton.grid(row=3, column=1, padx=5, pady=5)
        # ==========| Reported Position Frame |==========
        self.reportedPosFrame = Frame(self.armTab, bg=self.colorBG2)
        self.reportedPosFrame.grid(row=0, column=1, columnspan=2, padx=5, pady=5, sticky=W+N+E+S)
        # Reported Position label
        self.reportedPosLabel = Label(self.reportedPosFrame, text="Reported Position:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.reportedPosLabel.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky=W)
        # Request position button
        self.requestPosButton = Button(self.reportedPosFrame, text="Request Position", command=self.armController.requestPositionManual, width=15, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.requestPosButton.grid(row=0, column=1, padx=5, pady=5, sticky=E)
        # Reported position coordinate labels
        # XYZ
        self.xyzPosFrame = Frame(self.reportedPosFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.xyzPosFrame.grid(row=1, column=0, padx=5, pady=5)
        # Create them
        self.xCurCoordLabel  = Label(self.xyzPosFrame, text="X:", bg=self.colorBG2, fg=self.colorFG)
        self.xCurCoord       = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.yCurCoordLabel  = Label(self.xyzPosFrame, text="Y:", bg=self.colorBG2, fg=self.colorFG)
        self.yCurCoord       = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.zCurCoordLabel  = Label(self.xyzPosFrame, text="Z:", bg=self.colorBG2, fg=self.colorFG)
        self.zCurCoord       = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.RxCurCoordLabel = Label(self.xyzPosFrame, text="Rx:", bg=self.colorBG2, fg=self.colorFG)
        self.RxCurCoord      = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.RyCurCoordLabel = Label(self.xyzPosFrame, text="Ry:", bg=self.colorBG2, fg=self.colorFG)
        self.RyCurCoord      = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.RzCurCoordLabel = Label(self.xyzPosFrame, text="Rz:", bg=self.colorBG2, fg=self.colorFG)
        self.RzCurCoord      = Label(self.xyzPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        # Display them
        self.xCurCoordLabel.grid(row=0, column=0, padx=5, pady=5)
        self.xCurCoord.grid(row=0, column=1, padx=5, pady=5)
        self.yCurCoordLabel.grid(row=0, column=2, padx=5, pady=5)
        self.yCurCoord.grid(row=0, column=3, padx=5, pady=5)
        self.zCurCoordLabel.grid(row=0, column=4, padx=5, pady=5)
        self.zCurCoord.grid(row=0, column=5, padx=5, pady=5)
        self.RxCurCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.RxCurCoord.grid(row=1, column=5, padx=5, pady=5)
        self.RyCurCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.RyCurCoord.grid(row=1, column=3, padx=5, pady=5)
        self.RzCurCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.RzCurCoord.grid(row=1, column=1, padx=5, pady=5)
        
        # Angles
        self.jointPosFrame = Frame(self.reportedPosFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.jointPosFrame.grid(row=1, column=1, padx=5, pady=5)
        # Create them
        self.J1CurCoordLabel = Label(self.jointPosFrame, text="J1:", bg=self.colorBG2, fg=self.colorFG)
        self.J1CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.J2CurCoordLabel = Label(self.jointPosFrame, text="J2:", bg=self.colorBG2, fg=self.colorFG)
        self.J2CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.J3CurCoordLabel = Label(self.jointPosFrame, text="J3:", bg=self.colorBG2, fg=self.colorFG)
        self.J3CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.J4CurCoordLabel = Label(self.jointPosFrame, text="J4:", bg=self.colorBG2, fg=self.colorFG)
        self.J4CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.J5CurCoordLabel = Label(self.jointPosFrame, text="J5:", bg=self.colorBG2, fg=self.colorFG)
        self.J5CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.J6CurCoordLabel = Label(self.jointPosFrame, text="J6:", bg=self.colorBG2, fg=self.colorFG)
        self.J6CurCoord      = Label(self.jointPosFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        # Display them
        self.J1CurCoordLabel.grid(row=0, column=0, padx=5, pady=5)
        self.J1CurCoord.grid(row=0, column=1, padx=5, pady=5)
        self.J2CurCoordLabel.grid(row=0, column=2, padx=5, pady=5)
        self.J2CurCoord.grid(row=0, column=3, padx=5, pady=5)
        self.J3CurCoordLabel.grid(row=0, column=4, padx=5, pady=5)
        self.J3CurCoord.grid(row=0, column=5, padx=5, pady=5)
        self.J4CurCoordLabel.grid(row=1, column=0, padx=5, pady=5)
        self.J4CurCoord.grid(row=1, column=1, padx=5, pady=5)
        self.J5CurCoordLabel.grid(row=1, column=2, padx=5, pady=5)
        self.J5CurCoord.grid(row=1, column=3, padx=5, pady=5)
        self.J6CurCoordLabel.grid(row=1, column=4, padx=5, pady=5)
        self.J6CurCoord.grid(row=1, column=5, padx=5, pady=5)

        # ==========| Calibration Frame |==========
        self.calibrationFrame = Frame(self.armTab, bg=self.colorBG2)
        self.calibrationFrame.grid(row=1, column=0, padx=5, pady=5)
        # Calibration label
        self.calibrationLabel = Label(self.calibrationFrame, text="Calibration:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.calibrationLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W+N)
        # Full calibration button
        self.calibrateButton = Button(self.calibrationFrame, text="Calibrate", command=self.armController.startArmCalibration, width=10, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calibrateButton.grid(row=1, column=0, padx=5, pady=5, sticky=W+E)
        # Calibration offsets
        self.calOffsetFrame = Frame(self.calibrationFrame, bg=self.colorBG2)
        self.calOffsetFrame.grid(row=2, column=0, padx=5, pady=5, sticky=W+E)
        self.offsetLabel = Label(self.calOffsetFrame, text="Joint Offsets:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.offsetLabel.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky=W)
        # Make the widgets
        self.J1OffsetLabel = Label(self.calOffsetFrame,text="J1:", bg=self.colorBG2, fg=self.colorFG)
        self.J1OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        self.J2OffsetLabel = Label(self.calOffsetFrame,text="J2:", bg=self.colorBG2, fg=self.colorFG)
        self.J2OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        self.J3OffsetLabel = Label(self.calOffsetFrame,text="J3:", bg=self.colorBG2, fg=self.colorFG)
        self.J3OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        self.J4OffsetLabel = Label(self.calOffsetFrame,text="J4:", bg=self.colorBG2, fg=self.colorFG)
        self.J4OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        self.J5OffsetLabel = Label(self.calOffsetFrame,text="J5:", bg=self.colorBG2, fg=self.colorFG)
        self.J5OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        self.J6OffsetLabel = Label(self.calOffsetFrame,text="J6:", bg=self.colorBG2, fg=self.colorFG)
        self.J6OffsetEntry = Entry(self.calOffsetFrame, width=4, bg=self.colorBG2, fg=self.colorFG)
        # Grid the widgets
        self.J1OffsetLabel.grid(row=1, column=0, padx=5, pady=5)
        self.J1OffsetEntry.grid(row=1, column=1, padx=5, pady=5)
        self.J2OffsetLabel.grid(row=1, column=2, padx=5, pady=5)
        self.J2OffsetEntry.grid(row=1, column=3, padx=5, pady=5)
        self.J3OffsetLabel.grid(row=1, column=4, padx=5, pady=5)
        self.J3OffsetEntry.grid(row=1, column=5, padx=5, pady=5)
        self.J4OffsetLabel.grid(row=2, column=0, padx=5, pady=5)
        self.J4OffsetEntry.grid(row=2, column=1, padx=5, pady=5)
        self.J5OffsetLabel.grid(row=2, column=2, padx=5, pady=5)
        self.J5OffsetEntry.grid(row=2, column=3, padx=5, pady=5)
        self.J6OffsetLabel.grid(row=2, column=4, padx=5, pady=5)
        self.J6OffsetEntry.grid(row=2, column=5, padx=5, pady=5)
        # Auto fill a value of '0'
        self.J1OffsetEntry.insert(0, "0")
        self.J2OffsetEntry.insert(0, "0")
        self.J3OffsetEntry.insert(0, "0")
        self.J4OffsetEntry.insert(0, "0")
        self.J5OffsetEntry.insert(0, "0")
        self.J6OffsetEntry.insert(0, "0")

        # Individual calibration buttons
        self.indivCalFrame = Frame(self.calibrationFrame, bg=self.colorBG2)
        self.indivCalFrame.grid(row=3, column=0, padx=5, pady=5, sticky=W+E)
        # Label
        self.indivCalLabel = Label(self.indivCalFrame, text="Individual Calibrations:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.indivCalLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Make the buttons
        self.calJ1Button = Button(self.indivCalFrame, text="Cal J1", command=lambda: self.armController.startSpecificCalibration(1, 0, 0, 0, 0, 0), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calJ2Button = Button(self.indivCalFrame, text="Cal J2", command=lambda: self.armController.startSpecificCalibration(0, 1, 0, 0, 0, 0), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calJ3Button = Button(self.indivCalFrame, text="Cal J3", command=lambda: self.armController.startSpecificCalibration(0, 0, 1, 0, 0, 0), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calJ4Button = Button(self.indivCalFrame, text="Cal J4", command=lambda: self.armController.startSpecificCalibration(0, 0, 0, 1, 0, 0), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calJ5Button = Button(self.indivCalFrame, text="Cal J5", command=lambda: self.armController.startSpecificCalibration(0, 0, 0, 0, 1, 0), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.calJ6Button = Button(self.indivCalFrame, text="Cal J6", command=lambda: self.armController.startSpecificCalibration(0, 0, 0, 0, 0, 1), width=7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        # Place buttons
        self.calJ1Button.grid(row=1, column=0, padx=5, pady=5,)
        self.calJ2Button.grid(row=1, column=1, padx=5, pady=5)
        self.calJ3Button.grid(row=1, column=2, padx=5, pady=5)
        self.calJ4Button.grid(row=2, column=0, padx=5, pady=5)
        self.calJ5Button.grid(row=2, column=1, padx=5, pady=5)
        self.calJ6Button.grid(row=2, column=2, padx=5, pady=5)
        # ==== Open Post Calibration ====
        self.postCalibrationButton = Button(self.calibrationFrame, text="Post Calibration", command=self.createPostCalibration, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.postCalibrationButton.grid(row=4,column=0, padx=5, pady=5)
        # ==========| Tests Frame |==========
        self.armTestsFrame = Frame(self.armTab, bg=self.colorBG2)
        self.armTestsFrame.grid(row=1, column=1, padx=5, pady=5, sticky=W+E+N+S)
        # Label
        self.armTestsLabel = Label(self.armTestsFrame, text="Arm Tests:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.armTestsLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W+N)

        # Limit switch test
        self.limitTestFrame = Frame(self.armTestsFrame, bg=self.colorBG2)
        self.limitTestFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.limitTestButton = Button(self.limitTestFrame, text="Test Limit Switches", command=self.armController.toggleLimitTest, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.limitTestButton.grid(row=0, column=0, columnspan=6, sticky=W)
        # Create the widgets
        self.J1LimLabel = Label(self.limitTestFrame, text="J1:", bg=self.colorBG2, fg=self.colorFG)
        self.J1LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        self.J2LimLabel = Label(self.limitTestFrame, text="J2:", bg=self.colorBG2, fg=self.colorFG)
        self.J2LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        self.J3LimLabel = Label(self.limitTestFrame, text="J3:", bg=self.colorBG2, fg=self.colorFG)
        self.J3LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        self.J4LimLabel = Label(self.limitTestFrame, text="J4:", bg=self.colorBG2, fg=self.colorFG)
        self.J4LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        self.J5LimLabel = Label(self.limitTestFrame, text="J5:", bg=self.colorBG2, fg=self.colorFG)
        self.J5LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        self.J6LimLabel = Label(self.limitTestFrame, text="J6:", bg=self.colorBG2, fg=self.colorFG)
        self.J6LimState = Label(self.limitTestFrame, text="x", bg=self.colorBG2, fg=self.colorFG)
        
        # Display the widgets
        self.J1LimLabel.grid(row=1, column=0)
        self.J1LimState.grid(row=1, column=1)
        self.J2LimLabel.grid(row=1, column=2)
        self.J2LimState.grid(row=1, column=3)
        self.J3LimLabel.grid(row=1, column=4)
        self.J3LimState.grid(row=1, column=5)
        self.J4LimLabel.grid(row=2, column=0)
        self.J4LimState.grid(row=2, column=1)
        self.J5LimLabel.grid(row=2, column=2)
        self.J5LimState.grid(row=2, column=3)
        self.J6LimLabel.grid(row=2, column=4)
        self.J6LimState.grid(row=2, column=5)

        # Encoder test
        self.encoderTestFrame = Frame(self.armTestsFrame, bg=self.colorBG2)
        self.encoderTestFrame.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        self.encoderTestButton = Button(self.encoderTestFrame, text="Test Encoders", command=self.armController.toggleEncoderTest, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.encoderTestButton.grid(row=0, column=0, columnspan=6, sticky=W)
        # Create the widgets
        self.J1EncLabel = Label(self.encoderTestFrame, text="J1:", bg=self.colorBG2, fg=self.colorFG)
        self.J1EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)
        self.J2EncLabel = Label(self.encoderTestFrame, text="J2:", bg=self.colorBG2, fg=self.colorFG)
        self.J2EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)
        self.J3EncLabel = Label(self.encoderTestFrame, text="J3:", bg=self.colorBG2, fg=self.colorFG)
        self.J3EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)
        self.J4EncLabel = Label(self.encoderTestFrame, text="J4:", bg=self.colorBG2, fg=self.colorFG)
        self.J4EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)
        self.J5EncLabel = Label(self.encoderTestFrame, text="J5:", bg=self.colorBG2, fg=self.colorFG)
        self.J5EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)
        self.J6EncLabel = Label(self.encoderTestFrame, text="J6:", bg=self.colorBG2, fg=self.colorFG)
        self.J6EncState = Label(self.encoderTestFrame, text="xxxx", bg=self.colorBG2, fg=self.colorFG)

        # Display the widgets
        self.J1EncLabel.grid(row=1, column=0)
        self.J1EncState.grid(row=1, column=1)
        self.J2EncLabel.grid(row=1, column=2)
        self.J2EncState.grid(row=1, column=3)
        self.J3EncLabel.grid(row=1, column=4)
        self.J3EncState.grid(row=1, column=5)
        self.J4EncLabel.grid(row=2, column=0)
        self.J4EncState.grid(row=2, column=1)
        self.J5EncLabel.grid(row=2, column=2)
        self.J5EncState.grid(row=2, column=3)
        self.J6EncLabel.grid(row=2, column=4)
        self.J6EncState.grid(row=2, column=5)

        # ==========| Movement Frame |==========
        self.moveFrame = Frame(self.armTab, bg=self.colorBG2)
        self.moveFrame.grid(row=1, column=2, padx=5, pady=5, sticky=W+E+N+S)
        # ===| XYZ Move |===
        self.linearMoveFrame = Frame(self.moveFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.linearMoveFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+E+N+S)
        self.linearMoveLabel = Label(self.linearMoveFrame, text="XYZ Move:", bg=self.colorBG2, fg=self.colorFG)
        self.linearMoveLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Populate xyz button
        self.getXYZButton = Button(self.linearMoveFrame, text="Get XYZ", command = self.armController.populateMJ, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.getXYZButton.grid(row=0, column = 2, columnspan=2, padx=5, pady=5)
        # Send command button
        self.linearMoveButton = Button(self.linearMoveFrame, text="Send MJ", command=self.armController.prepMJCommand, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.linearMoveButton.grid(row=0, column=4, columnspan=2, padx=5, pady=5, sticky=E)
        self.linearMoveButton = Button(self.linearMoveFrame, text="Send ML", command=self.armController.prepMLCommand, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.linearMoveButton.grid(row=0, column=6, columnspan=2, padx=5, pady=5, sticky=E)

        # Coordinate labels and text boxes
        # Create them
        self.xCoordLabel  = Label(self.linearMoveFrame, text="X:", bg=self.colorBG2, fg=self.colorFG)
        self.xCoordEntry  = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.yCoordLabel  = Label(self.linearMoveFrame, text="Y:", bg=self.colorBG2, fg=self.colorFG)
        self.yCoordEntry  = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.zCoordLabel  = Label(self.linearMoveFrame, text="Z:", bg=self.colorBG2, fg=self.colorFG)
        self.zCoordEntry  = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J7CoordLabel = Label(self.linearMoveFrame, text="J7:", bg=self.colorBG2, fg=self.colorFG)
        self.J7CoordEntry = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.RxCoordLabel = Label(self.linearMoveFrame, text="Rx:", bg=self.colorBG2, fg=self.colorFG)
        self.RxCoordEntry = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.RyCoordLabel = Label(self.linearMoveFrame, text="Ry:", bg=self.colorBG2, fg=self.colorFG)
        self.RyCoordEntry = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.RzCoordLabel = Label(self.linearMoveFrame, text="Rz:", bg=self.colorBG2, fg=self.colorFG)
        self.RzCoordEntry = Entry(self.linearMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        # Display them
        self.xCoordLabel.grid(row=1, column=0, padx=(5, 0), pady=5)
        self.xCoordEntry.grid(row=1, column=1, padx=(0, 5), pady=5)
        self.yCoordLabel.grid(row=1, column=2, padx=(5, 0), pady=5)
        self.yCoordEntry.grid(row=1, column=3, padx=(0, 5), pady=5)
        self.zCoordLabel.grid(row=1, column=4, padx=(5, 0), pady=5)
        self.zCoordEntry.grid(row=1, column=5, padx=(0, 5), pady=5)
        self.J7CoordLabel.grid(row=1, column=6, padx=(0, 5), pady=5)
        self.J7CoordEntry.grid(row=1, column=7, padx=(0, 5), pady=5)
        self.RxCoordLabel.grid(row=2, column=4, padx=(5, 0), pady=5)
        self.RxCoordEntry.grid(row=2, column=5, padx=(0, 5), pady=5)
        self.RyCoordLabel.grid(row=2, column=2, padx=(5, 0), pady=5)
        self.RyCoordEntry.grid(row=2, column=3, padx=(0, 5), pady=5)
        self.RzCoordLabel.grid(row=2, column=0, padx=(5, 0), pady=5)
        self.RzCoordEntry.grid(row=2, column=1, padx=(0, 5), pady=5)

        # ===| Joint move |===
        self.jointMoveFrame = Frame(self.moveFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.jointMoveFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W+E+N+S)
        self.jointMoveLabel = Label(self.jointMoveFrame, text="Joint Move:", bg=self.colorBG2, fg=self.colorFG)
        self.jointMoveLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        # Get Joints
        self.getJointsButton = Button(self.jointMoveFrame, text="Get Joints", command = self.armController.populateJoints, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.getJointsButton.grid(row=0, column = 2, columnspan=2, padx=5, pady=5)
        # Send command button
        self.jointMoveButton = Button(self.jointMoveFrame, text="Send RJ", command=self.armController.prepRJCommand, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.jointMoveButton.grid(row=0, column=4, columnspan=3, padx=5, pady=5)
        # Joint labels and text boxes
        # Create them
        self.J1CoordLabel = Label(self.jointMoveFrame, text="J1:", bg=self.colorBG2, fg=self.colorFG)
        self.J1CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J2CoordLabel = Label(self.jointMoveFrame, text="J2:", bg=self.colorBG2, fg=self.colorFG)
        self.J2CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J3CoordLabel = Label(self.jointMoveFrame, text="J3:", bg=self.colorBG2, fg=self.colorFG)
        self.J3CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J4CoordLabel = Label(self.jointMoveFrame, text="J4:", bg=self.colorBG2, fg=self.colorFG)
        self.J4CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J5CoordLabel = Label(self.jointMoveFrame, text="J5:", bg=self.colorBG2, fg=self.colorFG)
        self.J5CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J6CoordLabel = Label(self.jointMoveFrame, text="J6:", bg=self.colorBG2, fg=self.colorFG)
        self.J6CoordEntry = Entry(self.jointMoveFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        # Display them
        self.J1CoordLabel.grid(row=1, column=0, padx=(5, 0), pady=5)
        self.J1CoordEntry.grid(row=1, column=1, padx=(0, 5), pady=5)
        self.J2CoordLabel.grid(row=1, column=2, padx=(5, 0), pady=5)
        self.J2CoordEntry.grid(row=1, column=3, padx=(0, 5), pady=5)
        self.J3CoordLabel.grid(row=1, column=4, padx=(5, 0), pady=5)
        self.J3CoordEntry.grid(row=1, column=5, padx=(0, 5), pady=5)
        self.J4CoordLabel.grid(row=2, column=0, padx=(5, 0), pady=5)
        self.J4CoordEntry.grid(row=2, column=1, padx=(0, 5), pady=5)
        self.J5CoordLabel.grid(row=2, column=2, padx=(5, 0), pady=5)
        self.J5CoordEntry.grid(row=2, column=3, padx=(0, 5), pady=5)
        self.J6CoordLabel.grid(row=2, column=4, padx=(5, 0), pady=5)
        self.J6CoordEntry.grid(row=2, column=5, padx=(0, 5), pady=5)
        #Move to safe position button
        self.moveToSafeButton = Button(self.moveFrame, text="Move to Safe Position", command=self.armController.prepMoveSafe, width=20, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.moveToSafeButton.grid(row=6, column=0, columnspan=6, padx=5, pady=5)
        self.moveToHomeButton = Button(self.moveFrame, text="Move to Home Position", command=self.armController.prepMoveHome, width=20, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.moveToHomeButton.grid(row=7, column=0, columnspan=6, padx=5, pady=5)
        # ==========| Loop Frame |==========
        self.loopFrame = Frame(self.armTab, bg=self.colorBG2)
        self.loopFrame.grid(row=0, column=3, padx=5, pady=5, sticky=W+E+N+S)
        self.loopFrameLabel = Label(self.loopFrame, text="Loop Mode:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.loopFrameLabel.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        self.openLoopButton = Button(self.loopFrame, text="Open Loop", command=self.armController.setOpenLoop, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.openLoopButton.grid(row=1, column=0, padx=5, pady=5)
        self.closedLoopButton = Button(self.loopFrame, text="Closed Loop", command=self.armController.setClosedLoop, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.closedLoopButton.grid(row=1, column=1, padx=5, pady=5)
        self.loopStatusLabel = Label(self.loopFrame, text="Status:", bg=self.colorBG2, fg=self.colorFG)
        self.loopStatusLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        self.loopStatus = Label(self.loopFrame, text="Unknown", bg=self.colorBG2, fg=self.colorFG)
        self.loopStatus.grid(row=2, column=1, padx=5, pady=5, sticky=W)

        # ==========| Origin Frame |==========
        self.originFrame = Frame(self.armTab, bg=self.colorBG2)
        self.originFrame.grid(row=1, column=3, padx=5, pady=5, sticky=W+E+N+S)
        self.originFrameLabel = Label(self.originFrame, text="Origin:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.originFrameLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Send command button
        self.setOrigin = Button(self.originFrame, text="Set Origin At Current Position", command=self.armController.setOrigin, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.setOrigin.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=W)

        # Coordinate labels and text boxes
        # Create them
        self.xyzOriginFrame = Frame(self.originFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.xyzOriginFrame.grid(row=2, column=0, padx=5, pady=5)
        # Create the widgets
        self.xCurCoordOriginLabel = Label(self.xyzOriginFrame, text="X:", bg=self.colorBG2, fg=self.colorFG)
        self.xCurCoordOrigin      = Label(self.xyzOriginFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.yCurCoordOriginLabel = Label(self.xyzOriginFrame, text="Y:", bg=self.colorBG2, fg=self.colorFG)
        self.yCurCoordOrigin      = Label(self.xyzOriginFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.zCurCoordOriginLabel = Label(self.xyzOriginFrame, text="Z:", bg=self.colorBG2, fg=self.colorFG)
        self.zCurCoordOrigin      = Label(self.xyzOriginFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        # Display the widgets
        self.xCurCoordOriginLabel.grid(row=0, column=0, padx=5, pady=5)
        self.xCurCoordOrigin.grid(row=0, column=1, padx=5, pady=5)
        self.yCurCoordOriginLabel.grid(row=0, column=2, padx=5, pady=5)
        self.yCurCoordOrigin.grid(row=0, column=3, padx=5, pady=5)
        self.zCurCoordOriginLabel.grid(row=0, column=4, padx=5, pady=5)
        self.zCurCoordOrigin.grid(row=0, column=5, padx=5, pady=5)
        #Move to Origin
        self.moveToOrigin = Button(self.originFrame, text="Move To Origin", command=self.armController.moveOrigin, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.moveToOrigin.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky=W)
        self.moveToRecommendedOrigin = Button(self.originFrame, text="Move To Default Origin", command=self.armController.moveRecommendedOrigin, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.moveToRecommendedOrigin.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky=W)

        #Delta Origin
        self.deltaOriginLabel = Label(self.originFrame, text="Delta from Origin:", bg=self.colorBG2, fg=self.colorFG)
        self.deltaOriginLabel.grid(row=5, column=0, columnspan=6, padx=5, pady=5, sticky=W)
        
        #Delta coordinates
        self.originDeltaFrame = Frame(self.originFrame, highlightthickness=1, highlightbackground=self.colorFG, bg=self.colorBG2)
        self.originDeltaFrame.grid(row=6, column=0, padx=5, pady=5)

        self.xDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔX:", bg=self.colorBG2, fg=self.colorFG)
        self.xDeltaOrigin      = Label(self.originDeltaFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.yDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔY:", bg=self.colorBG2, fg=self.colorFG)
        self.yDeltaOrigin      = Label(self.originDeltaFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.zDeltaOriginLabel = Label(self.originDeltaFrame, text="ΔZ:", bg=self.colorBG2, fg=self.colorFG)
        self.zDeltaOrigin      = Label(self.originDeltaFrame, text="xxx", bg=self.colorBG2, fg=self.colorFG) # 'xxx' until value reported
        self.xDeltaOriginLabel.grid(row=3, column=0, padx=5, pady=5)
        self.xDeltaOrigin.grid(row=3, column=1, padx=5, pady=5)
        self.yDeltaOriginLabel.grid(row=3, column=2, padx=5, pady=5)
        self.yDeltaOrigin.grid(row=3, column=3, padx=5, pady=5)
        self.zDeltaOriginLabel.grid(row=3, column=4, padx=5, pady=5)
        self.zDeltaOrigin.grid(row=3, column=5, padx=5, pady=5)

    def fillToolTab(self):
        # The tool jog frame is buggy and not currently useful so it has been commented out
        # Still here in case we want to use it in the future
        '''        
        # ===Tool jog frame===
        self.toolJogFrame = Frame(self.toolTab, highlightthickness=2, highlightbackground="#000000")
        self.toolJogFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W+N+E+S)
        # Label
        self.toolJogLabel = Label(self.toolJogFrame, text="Tool Jog:")
        self.toolJogLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        #Start and stop
        self.startToolJog = Button(self.toolJogFrame, text="Start tool jog", command=self.armController.startToolJog)
        self.startToolJog.grid(row=1, column=0, padx=5, pady=5,sticky=W)
        self.stopToolJog = Button(self.toolJogFrame, text="Stop tool jog", command=self.armController.stopToolJog)
        self.stopToolJog.grid(row=2, column=0, padx=5, pady=5,sticky=W)
        self.toolJogDirection = Label(self.toolJogFrame, text="Direction: N/A")
        self.toolJogDirection.grid(row=1,column=1)
        self.changeToolJogDirection = Button(self.toolJogFrame, text="Change direction",command=self.armController.changeToolJogDirection)
        self.changeToolJogDirection.grid(row=2,column=1)
        #Select Axis
        self.selectAxisFrame = Frame(self.toolJogFrame)
        self.selectAxisFrame.grid(row=3, column=0, padx=5, pady=5, sticky=W+E)
        self.selectAxisLabel = Label(self.selectAxisFrame, text="Select Axis For Jog")
        self.selectAxisLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)

        # Make the select buttons
        self.toolJogSetX = Button(self.selectAxisFrame, text="X", command=lambda: self.armController.selectToolJogAxis(1), width=7)
        self.toolJogSetY = Button(self.selectAxisFrame, text="Y", command=lambda: self.armController.selectToolJogAxis(2), width=7)
        self.toolJogSetZ = Button(self.selectAxisFrame, text="Z", command=lambda: self.armController.selectToolJogAxis(3), width=7)
        self.toolJogSetRz = Button(self.selectAxisFrame, text="Rz", command=lambda: self.armController.selectToolJogAxis(4), width=7)
        self.toolJogSetRy = Button(self.selectAxisFrame, text="Ry", command=lambda: self.armController.selectToolJogAxis(5), width=7)
        self.toolJogSetRx = Button(self.selectAxisFrame, text="Rx", command=lambda: self.armController.selectToolJogAxis(6), width=7)
    
        # Place buttons
        self.toolJogSetX.grid(row=1, column=0, padx=5, pady=5,)
        self.toolJogSetY.grid(row=1, column=1, padx=5, pady=5)
        self.toolJogSetZ.grid(row=1, column=2, padx=5, pady=5)
        self.toolJogSetRz.grid(row=2, column=0, padx=5, pady=5)
        self.toolJogSetRy.grid(row=2, column=1, padx=5, pady=5)
        self.toolJogSetRx.grid(row=2, column=2, padx=5, pady=5)'''
        # ========= Extruder Frame ===========
        self.extruderFrame = Frame(self.toolTab, bg=self.colorBG2)
        self.extruderFrame.grid(row=0, column=1, padx=5, pady=5, sticky=W+N+E+S)

        self.extruderLabel = Label(self.extruderFrame, text="Extruder Control For Testing", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.extruderLabel.grid(row=0,column=0, columnspan=2)
        self.J7CoordLabel2 = Label(self.extruderFrame, text="J7:", bg=self.colorBG2, fg=self.colorFG)
        self.J7CoordEntry2 = Entry(self.extruderFrame, width=6, bg=self.colorBG2, fg=self.colorFG)
        self.J7CoordLabel2.grid(row=1, column=0, padx=(0, 5), pady=5)
        self.J7CoordEntry2.grid(row=1, column=1, padx=(0, 5), pady=5)
        self.extrudeButton = Button(self.extruderFrame, text = "Extrude", command=self.armController.extrudeButton, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.extrudeButton.grid(row=2, column=0,padx=(0, 5), pady=5)
        self.zeroJ7Button = Button(self.extruderFrame, text = "Zero", command=self.armController.zeroJ7, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.zeroJ7Button.grid(row=2,column=1,padx=(0, 5), pady=5)

        self.loadButton = Button(self.extruderFrame, text = "Load", command=self.armController.loadFilament, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.loadButton.grid(row=3, column=0,padx=(0, 5), pady=5)
        self.unloadButton = Button(self.extruderFrame, text = "Unload", command=self.armController.unloadFilament, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.unloadButton.grid(row=3, column=1,padx=(0, 5), pady=5)
        self.currentJ72 = Label(self.extruderFrame,text="Extruded: 0 mm", bg=self.colorBG2, fg=self.colorFG)
        self.currentJ72.grid(row=6,column=0, padx=5, pady=5, sticky=N+S)


    def fillDebugTab(self):
        # ==========| Variables Frame |==========
        self.debugVarFrame = Frame(self.debugTab, width=300, height=620, bg=self.colorBG2)
        self.debugVarFrame.grid(row=0, column=0, padx=5, pady=5)
        self.debugVarFrame.grid_propagate(False)
        # ===| SerialController Variables |===
        self.serDebugFrame = Frame(self.debugVarFrame, highlightthickness=1, highlightbackground="#000000", bg=self.colorBG2)
        self.serDebugFrame.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        self.serDebugLabel = Label(self.serDebugFrame, text="SerialController:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.serDebugLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # boardConnected
        self.serDebugBoardLabel = Label(self.serDebugFrame, text="boardConnected = ", bg=self.colorBG2, fg=self.colorFG)
        self.serDebugBoardLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        # responseReady
        self.serDebugRespLabel = Label(self.serDebugFrame, text="responseReady = ", bg=self.colorBG2, fg=self.colorFG)
        self.serDebugRespLabel.grid(row=3, column=0, padx=5, pady=5, sticky=W)
        #Override calibration button
        self.printQueueButton = Button(self.serDebugFrame,text="Print Serial Queue", command=self.serialController.printQueue, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.printQueueButton.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky=W)

        # ===| ArmController Variables |===
        self.armDebugFrame = Frame(self.debugVarFrame, highlightthickness=1, highlightbackground="#000000", bg=self.colorBG2)
        self.armDebugFrame.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        self.armDebugLabel = Label(self.armDebugFrame, text="ArmController:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.armDebugLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # armCalibrated
        self.armDebugCalLabel = Label(self.armDebugFrame, text="armCalibrated = ", bg=self.colorBG2, fg=self.colorFG)
        self.armDebugCalLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)
        # calibrationinProgress
        self.armDebugCalInProgLabel = Label(self.armDebugFrame, text="calibrationInProgress = ", bg=self.colorBG2, fg=self.colorFG)
        self.armDebugCalInProgLabel.grid(row=2, column=0, padx=5, pady=5, sticky=W)
        # calibrationState
        self.armDebugCalStateLabel = Label(self.armDebugFrame, text="calibrationState = ", bg=self.colorBG2, fg=self.colorFG)
        self.armDebugCalStateLabel.grid(row=3, column=0, padx=5, pady=5, sticky=W)
        #Override calibration button
        self.overrideCalibrationButton = Button(self.debugVarFrame,text="Override Calibration", command=self.armController.overrideCalibration, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.overrideCalibrationButton.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky=W)
    
        # ===| PrintController Variables |===
        self.printDebugFrame = Frame(self.debugVarFrame, highlightthickness=1, highlightbackground="#000000", bg=self.colorBG2)
        self.printDebugFrame.grid(row=4, column=0, padx=5, pady=5, sticky=W)
        self.printDebugLabel = Label(self.printDebugFrame, text="PrintController:", font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        self.printDebugLabel.grid(row=0, column=0, padx=5, pady=5, sticky=W)
        # armCalibrated
        self.printDebugPrintLabel = Label(self.printDebugFrame, text="printing = ", bg=self.colorBG2, fg=self.colorFG)
        self.printDebugPrintLabel.grid(row=1, column=0, padx=5, pady=5, sticky=W)
    
        # ==========| Terminal Frame |==========

        self.termFrame = Frame(self.debugTab, bg=self.colorBG2, highlightthickness=2, highlightbackground="#000000", width=900)
        self.termFrame.grid(row=0, column=1, padx=5, pady=5, sticky=W+E+N+S)

        self.termVertScroll = Scrollbar(self.termFrame, orient="vertical")
        self.termHorzScroll = Scrollbar(self.termFrame, orient="horizontal")

        self.terminal = Text(self.termFrame,
                            wrap=NONE,
                            width=109,
                            height=37,
                            yscrollcommand=self.termVertScroll.set,
                            xscrollcommand=self.termHorzScroll.set,
                            state="disabled",
                            background=self.colorBG2,
                            foreground=self.colorFG
                            )
        
        self.termVertScroll.pack(side=RIGHT, fill=Y)
        self.termVertScroll.config(command=self.terminal.yview)
        
        self.termHorzScroll.pack(side=BOTTOM, fill=X)
        self.termHorzScroll.config(command=self.terminal.xview)
        #self.terminal.grid(row=1, column=0, columnspan=5, padx=5, pady=5)
        self.terminal.pack(fill=BOTH)

    def fillSettingsTab(self):
        self.settingsFrame = Frame(self.settingsTab, bg=self.colorBG2)
        self.settingsFrame.pack(side="left", fill="y", padx=10,pady=5)#grid(row=0,column=0)
        self.settingsFrame2 = Frame(self.settingsTab, bg=self.colorBG2)
        self.settingsFrame2.pack(side="left", fill="y", padx=10,pady=5)#grid(row=0,column=0)
        maxSettingsPerColumn = 13

        if len(self.settingsDict) <= maxSettingsPerColumn:
            self.settingsLength1 = len(self.settingsDict)
            self.settingsLength2 = 0
        else:
            self.settingsLength1 = maxSettingsPerColumn
            self.settingsLength2 = len(self.settingsDict) - maxSettingsPerColumn

        # Headers for all settings
        header1 = Label(self.settingsFrame, text="Setting",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        header2 = Label(self.settingsFrame, text="Current",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        header3 = Label(self.settingsFrame, text="Change To",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
        horizontalLine1 = ttk.Separator(self.settingsFrame,orient="horizontal")
        verticalLine1 = ttk.Separator(self.settingsFrame,orient="vertical")
        verticalLine2 = ttk.Separator(self.settingsFrame,orient="vertical")
        horizontalLine2 = ttk.Separator(self.settingsFrame,orient="horizontal")
        header1.grid(row=0, column=0,pady=(5,0))
        header2.grid(row=0, column=2,pady=(5,0))
        header3.grid(row=0, column=4,pady=(5,0))
        horizontalLine1.grid(row=1,column=0,columnspan=5,sticky=EW)
        horizontalLine2.grid(row=self.settingsLength1+2,column=0,columnspan=5,pady=5,sticky=EW)
        verticalLine1.grid(row=0,column=1,rowspan=self.settingsLength1+3,sticky=NS,pady=(0,5))
        verticalLine2.grid(row=0,column=3,rowspan=self.settingsLength1+3,sticky=NS,pady=(0,5))
        
        if self.settingsLength2 > 0:
            # Headers for all settings
            header1_1 = Label(self.settingsFrame2, text="Setting",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
            header2_1 = Label(self.settingsFrame2, text="Current",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
            header3_1 = Label(self.settingsFrame2, text="Change To",padx=5, font=("TkDefaultFont", 12, "bold"), bg=self.colorBG2, fg=self.colorFG)
            horizontalLine1_1 = ttk.Separator(self.settingsFrame2,orient="horizontal")
            verticalLine1_1 = ttk.Separator(self.settingsFrame2,orient="vertical")
            verticalLine2_1 = ttk.Separator(self.settingsFrame2,orient="vertical")
            horizontalLine2_1 = ttk.Separator(self.settingsFrame2,orient="horizontal")
            header1_1.grid(row=0, column=0,pady=(5,0))
            header2_1.grid(row=0, column=2,pady=(5,0))
            header3_1.grid(row=0, column=4,pady=(5,0))
            horizontalLine1_1.grid(row=1,column=0,columnspan=5,sticky=EW)
            horizontalLine2_1.grid(row=self.settingsLength2+2,column=0,columnspan=5,pady=5,sticky=EW)
            verticalLine1_1.grid(row=0,column=1,rowspan=self.settingsLength2+3,sticky=NS,pady=(0,5))
            verticalLine2_1.grid(row=0,column=3,rowspan=self.settingsLength2+3,sticky=NS,pady=(0,5))
        
        #Stores the label and entry objects
        self.entries = {}
        self.currents = {}
        PC = self.printController
        AC = self.armController
        SC = self.serialController

        row = 2 #start below headers
        for item in self.settingsDict:
             
            attr, object = self.settingsDict[item]

            match object:
                case "PC":
                    object = PC
                case "AC":
                    object = AC
                case "SC":
                    object = SC
                case "self":
                    object = self
            currentValue = getattr(object,attr)
            if row < maxSettingsPerColumn + 2: #if there is still room in the first column
                settingLabel = Label(self.settingsFrame, text=item, bg=self.colorBG2, fg=self.colorFG)
                self.currents[item] = Label(self.settingsFrame, text=str(currentValue), bg=self.colorBG2, fg=self.colorFG)
                self.entries[item] = Entry(self.settingsFrame,width=10, bg=self.colorBG2, fg=self.colorFG)
                #Place
                settingLabel.grid(row=row, column=0)
                self.currents[item].grid(row=row, column=2)
                self.entries[item].grid(row=row, column=4,padx=5,pady=5)
            else:
                settingLabel = Label(self.settingsFrame2, text=item, bg=self.colorBG2, fg=self.colorFG)
                self.currents[item] = Label(self.settingsFrame2, text=str(currentValue), bg=self.colorBG2, fg=self.colorFG)
                self.entries[item] = Entry(self.settingsFrame2,width=10, bg=self.colorBG2, fg=self.colorFG)
            
                #Place and adjust row number for second column
                settingLabel.grid(row=row-maxSettingsPerColumn, column=0)
                self.currents[item].grid(row=row-maxSettingsPerColumn, column=2)
                self.entries[item].grid(row=row-maxSettingsPerColumn, column=4,padx=5,pady=5)
            row += 1


        self.setAllSettingsButton = Button(self.settingsFrame,text="Set All Settings",command=self.setAllSettings, bg=self.colorAccent, fg=self.colorBG, font=("TkDefaultFont", 10, "bold"))
        self.setAllSettingsButton.grid(row=maxSettingsPerColumn+4,column=0,columnspan=5,padx=5,pady=5) #Settings will always be at the bottom

    #endregion Tabs
    #Set settings
    def setAllSettings(self):

        PC = self.printController
        AC = self.armController
        SC = self.serialController

        for item in self.settingsDict:
            valueToSet = self.entries[item].get().strip()
           
            attr, object = self.settingsDict[item]

            match object:
                case "PC":
                    object = PC
                case "AC":
                    object = AC
                case "SC":
                    object = SC
                case "self":
                    object = self
            
            currentValue = getattr(object,attr)
            #If there is a value to set
            if valueToSet != "":
                try:
                    # Convert to the same type as the existing value
                    correctType = type(currentValue)
                    #Special case to handle bool because any non empty string is converted to True
                    if correctType == bool:
                        #Option for several true cases and false cases
                        if valueToSet.lower() in ("true", "1", "yes", "y"):
                            valueToSet = True
                        elif valueToSet.lower() in ("false", "0", "no", "n"):
                            valueToSet = False
                        else:
                            raise ValueError
                    if correctType != None:
                        valueToSet = correctType(valueToSet)
                    setattr(object, attr, valueToSet)

                except ValueError:
                    self.terminalPrint(f"Invalid value for {item}")

            #Get current value to make sure the value was actually set and display
            #Also update all values even if unchanged by user
            currentValue = getattr(object,attr)
            self.currents[item].config(text=str(currentValue))

        #Recreate parameters
        PC.defaultPrintParameters = MoveParameters(PC.speed,PC.acceleration,PC.decceleration,PC.ramp,PC.printOpenLoopControl,"m")
        PC.changeRecommendedOrigin(PC.plateHeight) #change origin with plate height
        AC.extruder_deg_per_mm = AC.extruder_deg_per_mm_cool * AC.heatedFilamentMultiplier

    #region main update function
    def update(self):
        self.updateDebugVars() # Update the debug tab variables

        # ==========| SerialController |==========

        # ===========| ArmController |============
        
        # ==========| PrintController |==========
        #Each print loop runs in the thread so that it can "wait" and not halt the UI
        if self.printController.printing and not self.printThreadStarted and not self.printController.printPaused:
                printThread = threading.Thread(target=self.printController.printLoop, name = "Print Loop Thread")
                printThread.start()
                #Variable to signal when the thread finishes
                self.printThreadStarted = True
        
        # TODO: Temporary
        self.temperatureController.updateTemp()

        # ==========| Home Tab |==========

        # Thermals
        hotendData = self.temperatureController.prevHotendTemps
        bedData = self.temperatureController.prevBedTemps
        if ((len(hotendData) == len(bedData) > 0) and (len(hotendData) > 1)):
            # Update the labels
            self.hotendHomeActual.config(text=f"{self.temperatureController.hotendTempCelsius:.2f}")
            #self.hotendHomeTarget.config(text=round(self.temperatureController.hotendTargetTemp, 2))
            self.bedHomeActual.config(text=f"{self.temperatureController.bedTempCelsius:.2f}")
            #self.bedHomeTarget.config(text=round(self.temperatureController.bedTargetTemp, 2))

            # Update the plot
            xData = np.linspace(self.updateDelay / -1000 * len(hotendData), 0, len(hotendData))
            self.thermalHotendLine.set_data(xData, hotendData)
            self.thermalBedLine.set_data(xData, bedData)
            self.thermalPlot.set_xlim(min(xData), max(xData))
            self.yMax = max([max(hotendData), max(bedData), self.temperatureController.hotendTargetTemp, self.temperatureController.bedTargetTemp]) + 10

            self.thermalPlot.set_ylim(0, self.yMax)
            self.hotendTargetLine.remove()
            self.bedTargetLine.remove()
            self.hotendTargetLine = self.thermalPlot.axhline(y=self.temperatureController.hotendTargetTemp, color='r', linestyle='--')
            self.bedTargetLine = self.thermalPlot.axhline(y=self.temperatureController.bedTargetTemp, color='b', linestyle='--')
            self.thermalCanvas.draw()
        
        # Arm Info
        if self.serialController.boardConnected:
            self.connectedStatusHome.config(text="Arm Connected", fg="#00FF00")
        else:
            self.connectedStatusHome.config(text="Arm Disconnected", fg="#FF0000")
        if self.armController.armCalibrated:
            self.calibrationStatusHome.config(text="Arm Calibrated", fg="#00FF00")
        else:
            self.calibrationStatusHome.config(text="Arm NOT Calibrated", fg="#FF0000")

        # Set up another call to the update function after updateDelay milliseconds
        self.after(self.updateDelay, self.update)

    #endregion main update function
    #region other functions
    def updateDebugVars(self):
        # SerialController vars
        self.serDebugBoardLabel.config(text=f"boardConnected = {self.serialController.boardConnected}")
        self.serDebugRespLabel.config(text=f"responseReady = {self.serialController.responseReady}")

        # ArmController vars
        self.armDebugCalLabel.config(text=f"armCalibrated = {self.armController.armCalibrated}")
        self.armDebugCalInProgLabel.config(text=f"calibrationInProgress = {self.armController.calibrationInProgress}")
        self.armDebugCalStateLabel.config(text=f"calibrationState = {self.armController.calibrationState}")
        

        # PrintController vars
        self.printDebugPrintLabel.config(text=f"printing = {self.printController.printing}")

    # Called whenever the selection in the port dropdown is changed
    def portSelectionChanged(self, *args):
        print("Port Selection Changed")
        # Check if the selected port is the default "Select Port" text
        if self.portSelection.get() == "Select Port":
            # If so, disable the connect button to avoid connecting to nothing
            self.connectButton.config(state="disabled")
        else:
            # If not, enable the connect button
            self.connectButton.config(state="normal")
    
    def clearEntryFocus(self, event):
        # Check if our current focus is a specific entry widget and if the new focus is different
        # If so we unfocus the entry and do some other stuff if needed
        if self.root.focus_get() == self.hotendHomeTarget and event.widget != self.hotendHomeTarget:
            self.temperatureController.setHotendTargetTemp(float(self.hotendHomeTarget.get()), False)
            self.root.focus_set()
        elif self.root.focus_get() == self.bedHomeTarget and event.widget != self.bedHomeTarget:
            self.temperatureController.setBedTargetTemp(float(self.bedHomeTarget.get()), False)
            self.root.focus_set()

    def windowDimensions(self, event):
        # Used for debugging purposes but it simply prints the window width and height
        print(f"{self.winfo_width()}x{self.winfo_height()}")
    #endregion other functions


    #region Print Functions
    # Used to print to the in window terminal
    def terminalPrint(self, message):
        print(message) # Print to the Python terminal as well
        self.terminal.config(state="normal") # Need to enable to modify
        self.terminal.insert(END, f"{datetime.now().now()}| {message}\n") # Print the message with the current time fixated at the front
        self.terminal.config(state="disabled") # Disable again to avoid user changes
        self.terminal.see("end") # Forces terminal to autoscroll with new text. Probably want to make this a toggle option

    # Used to print important info to the status bar
    def statusPrint(self, message):
        # TODO: Add extra check for if text is too long and do a fancy '...' or something to show that there is more
        # Update the status label
        self.statusLabel.config(text=f"Status: {message}")
        # Also print the full message to the terminal
        self.terminalPrint(message)

    def warningPrint(self, message):
        self.BlinkLED = True # Used to signal the update loop to start blinking the LED
        #Display a 2nd message based on ignoreflags
        message2 = ""
        if self.printController.ignoreFlags and self.printController.printing:
            message2 = "\nPressing ok will resume movement"
        elif self.printController.printing:
            message2 = "\nPrinting will stop"
        #show message
        messagebox.showinfo("Warning! ", message+message2)
        self.statusPrint(message)
        self.BlinkLED = False # Stop blinking the LED

    #endregion print functions
    #region LED Update Loop
    def updateLED(self):
        while self.ledThreadRunning:
            # LED Blinking takes priority over simple on/off
            if self.BlinkLED:
                # Check the current state of the LED
                tempState = GPIO.input(self.LEDPin)
                # Invert the state
                if tempState == GPIO.HIGH:
                    GPIO.output(self.LEDPin, GPIO.LOW)
                else:
                    GPIO.output(self.LEDPin, GPIO.HIGH)
            # If the LEDOn flag is set, turn the LED on
            elif self.LEDOn:
                GPIO.output(self.LEDPin, GPIO.HIGH)
            # If no flags are set, turn the LED off
            else:
                GPIO.output(self.LEDPin, GPIO.LOW)
            # Use this to set blinking rate and how quickly the LED updates
            time.sleep(0.25)

    #endregion
    #region popup
    def createPostCalibration(self):
        # Create a new top-level window
        popup = Toplevel(self.root)
        popup.wm_title("Post Calibration")
        ws = self.winfo_screenwidth() # Get screen width
        hs = self.winfo_screenheight() # Get screen height
        x = int((ws/2) - (200/2)) # Calculate x position for window to be in the center of the screen
        y = int((hs/2) - (200/2)) # Calculate y position for window to be in the center of the screen
        popup.geometry(f"250x300+{x}+{y}")
        # Ensure the popup stays on top of the main window
        popup.grab_set() 
        popup.lift()
        # Add widgets to the popup
        label2 = Label(popup, text="Adjust calibration without limit switches.\n Works accumalatively. \n Be careful to not move past limit switches.")
        label2.grid(row=1,column=0)
        self.calOffsetFrameP = Frame(popup)
        self.calOffsetFrameP.grid(row=2, column=0, padx=5, pady=5, sticky=W+E)
        self.offsetLabel = Label(self.calOffsetFrameP, text="Joint Offsets:")
        self.offsetLabel.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky=W)
        # Make the widgets
        self.J1OffsetLabelP = Label(self.calOffsetFrameP,text="J1:")
        self.J1OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        self.J2OffsetLabelP = Label(self.calOffsetFrameP,text="J2:")
        self.J2OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        self.J3OffsetLabelP = Label(self.calOffsetFrameP,text="J3:")
        self.J3OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        self.J4OffsetLabelP = Label(self.calOffsetFrameP,text="J4:")
        self.J4OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        self.J5OffsetLabelP = Label(self.calOffsetFrameP,text="J5:")
        self.J5OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        self.J6OffsetLabelP = Label(self.calOffsetFrameP,text="J6:")
        self.J6OffsetEntryP = Entry(self.calOffsetFrameP, width=4)
        # Grid the widgets
        self.J1OffsetLabelP.grid(row=1, column=0, padx=5, pady=5)
        self.J1OffsetEntryP.grid(row=1, column=1, padx=5, pady=5)
        self.J2OffsetLabelP.grid(row=1, column=2, padx=5, pady=5)
        self.J2OffsetEntryP.grid(row=1, column=3, padx=5, pady=5)
        self.J3OffsetLabelP.grid(row=1, column=4, padx=5, pady=5)
        self.J3OffsetEntryP.grid(row=1, column=5, padx=5, pady=5)
        self.J4OffsetLabelP.grid(row=2, column=0, padx=5, pady=5)
        self.J4OffsetEntryP.grid(row=2, column=1, padx=5, pady=5)
        self.J5OffsetLabelP.grid(row=2, column=2, padx=5, pady=5)
        self.J5OffsetEntryP.grid(row=2, column=3, padx=5, pady=5)
        self.J6OffsetLabelP.grid(row=2, column=4, padx=5, pady=5)
        self.J6OffsetEntryP.grid(row=2, column=5, padx=5, pady=5)
        # Auto fill a value of '0'
        self.J1OffsetEntryP.insert(0, "0")
        self.J2OffsetEntryP.insert(0, "0")
        self.J3OffsetEntryP.insert(0, "0")
        self.J4OffsetEntryP.insert(0, "0")
        self.J5OffsetEntryP.insert(0, "0")
        self.J6OffsetEntryP.insert(0, "0")

        # Individual calibration buttons
        self.indivCalPFrame = Frame(popup)
        self.indivCalPFrame.grid(row=3, column=0, padx=5, pady=5, sticky=W+E)
        # Label
        self.indivCalPLabel = Label(self.indivCalPFrame, text="Individual Calibrations:")
        self.indivCalPLabel.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky=W)
        # Make the buttons
        self.calPJ1Button = Button(self.indivCalPFrame, text="Cal J1", command=lambda: self.armController.startPostCalibration(1, 0, 0, 0, 0, 0), width=7)
        self.calPJ2Button = Button(self.indivCalPFrame, text="Cal J2", command=lambda: self.armController.startPostCalibration(0, 1, 0, 0, 0, 0), width=7)
        self.calPJ3Button = Button(self.indivCalPFrame, text="Cal J3", command=lambda: self.armController.startPostCalibration(0, 0, 1, 0, 0, 0), width=7)
        self.calPJ4Button = Button(self.indivCalPFrame, text="Cal J4", command=lambda: self.armController.startPostCalibration(0, 0, 0, 1, 0, 0), width=7)
        self.calPJ5Button = Button(self.indivCalPFrame, text="Cal J5", command=lambda: self.armController.startPostCalibration(0, 0, 0, 0, 1, 0), width=7)
        self.calPJ6Button = Button(self.indivCalPFrame, text="Cal J6", command=lambda: self.armController.startPostCalibration(0, 0, 0, 0, 0, 1), width=7)
        # Place buttons
        self.calPJ1Button.grid(row=1, column=0, padx=5, pady=5,)
        self.calPJ2Button.grid(row=1, column=1, padx=5, pady=5)
        self.calPJ3Button.grid(row=1, column=2, padx=5, pady=5)
        self.calPJ4Button.grid(row=2, column=0, padx=5, pady=5)
        self.calPJ5Button.grid(row=2, column=1, padx=5, pady=5)
        self.calPJ6Button.grid(row=2, column=2, padx=5, pady=5)
        # Add a button to close the popup
        close_button = Button(popup, text="Close", command=popup.destroy)
        close_button.pack(pady=10)

    #endregion popup
    
#Start the Program
if __name__ == "__main__":
    app = TkWindow()
    app.mainloop()