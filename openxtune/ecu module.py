# modules/ecu_tuner.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGroupBox
from PyQt5.QtCore import pyqtSignal

class ECUTuner(QWidget):
    def __init__(self, connection, safety):
        super().__init__()
        self.connection = connection
        self.safety = safety
        self.current_map = None
        
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Connection status
        self.conn_status = QGroupBox("Vehicle Connection")
        
        # Map selection
        self.map_selector = QComboBox()
        self.map_selector.addItems(["Fuel Map", "Ignition Map", "Boost Map", "VVT Map"])
        self.map_selector.currentIndexChanged.connect(self.load_map)
        
        # Map visualization
        self.map_view = TableWidget()
        self.map_view.setRowCount(16)
        self.map_view.setColumnCount(16)
        
        # Controls
        controls = QHBoxLayout()
        self.read_btn = QPushButton("Read ECU")
        self.write_btn = QPushButton("Write ECU")
        self.flash_btn = QPushButton("Flash ECU")
        
        self.read_btn.clicked.connect(self.read_ecu)
        self.write_btn.clicked.connect(self.write_ecu)
        self.flash_btn.clicked.connect(self.flash_ecu)
        
        layout.addWidget(self.conn_status)
        layout.addWidget(self.map_selector)
        layout.addWidget(self.map_view)
        layout.addLayout(controls)
        
        self.setLayout(layout)
    
    def load_map(self, index):
        """Load selected map from ECU"""
        map_name = self.map_selector.currentText()
        self.current_map = self.connection.read_map(map_name)
        self.display_map(self.current_map)
    
    def display_map(self, map_data):
        """Display map data in table"""
        for row in range(16):
            for col in range(16):
                self.map_view.setItem(row, col, QTableWidgetItem(str(map_data[row][col])))
    
    def read_ecu(self):
        """Read full ECU data"""
        pass
        
    def write_ecu(self):
        """Write modified maps to ECU"""
        if self.safety.validate_write():
            self.connection.write_map(self.current_map)
    
    def flash_ecu(self):
        """Flash complete ECU image"""
        if self.safety.validate_flash():
            self.connection.flash_ecu()