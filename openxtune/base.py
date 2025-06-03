# main.py - Core application
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget
from modules.ecu_tuner import ECUTuner
from modules.tcu_tuner import TCUTuner
from modules.abs_tuner import ABSTuner
from modules.tcs_tuner import TCSTuner

class CANxTune(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CANxTune - Open Source Vehicle Tuning Suite")
        self.setGeometry(100, 100, 1024, 768)
        
        # Initialize core components
        self.init_connection_manager()
        self.init_safety_systems()
        
        # Setup UI
        self.init_ui()
        
    def init_connection_manager(self):
        """Initialize vehicle communication interface"""
        self.connection = ConnectionManager()
        
    def init_safety_systems(self):
        """Initialize safety and validation systems"""
        self.safety = SafetySystem()
        
    def init_ui(self):
        """Setup main interface"""
        tabs = QTabWidget()
        
        # Add module tabs
        tabs.addTab(ECUTuner(self.connection, self.safety), "ECU Tuning")
        tabs.addTab(TCUTuner(self.connection, self.safety), "TCU Tuning")
        tabs.addTab(ABSTuner(self.connection, self.safety), "ABS Tuning")
        tabs.addTab(TCSTuner(self.connection, self.safety), "TCS Tuning")
        
        self.setCentralWidget(tabs)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CANxTune()
    window.show()
    sys.exit(app.exec_())