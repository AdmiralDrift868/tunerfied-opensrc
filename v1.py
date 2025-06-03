import sys
import os
import json
import uuid
import numpy as np
import can
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QComboBox, QTableWidget, QTableWidgetItem, QPushButton, 
                             QLabel, QLineEdit, QMessageBox, QFileDialog, QAction, QStatusBar,
                             QSplitter, QTreeWidget, QTreeWidgetItem, QHeaderView, QProgressBar)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

matplotlib.use('Qt5Agg')

# ======================
# CORE SYSTEM COMPONENTS
# ======================

class ProtocolHandler:
    """Base class for protocol handlers"""
    def __init__(self):
        self.name = "Base Protocol"
        self.supported_ecus = []
        self.map_definitions = {}
        
    def connect(self, bus):
        """Establish connection to vehicle"""
        pass
        
    def read_map(self, map_name):
        """Read specific map from ECU"""
        return np.zeros((10, 10))  # Return dummy data
        
    def write_map(self, map_data):
        """Write map to ECU"""
        pass
        
    def flash_ecu(self, image_data):
        """Flash complete ECU image"""
        pass
        
    def read_live_data(self, pids):
        """Read live data parameters"""
        return {}
        
    def get_supported_maps(self):
        """Get list of supported maps"""
        return list(self.map_definitions.keys())

class OBD2Protocol(ProtocolHandler):
    """OBD-II Protocol Implementation"""
    def __init__(self):
        super().__init__()
        self.name = "OBD-II"
        self.supported_ecus = ["ECU", "TCU"]
        self.map_definitions = {
            "Fuel Map": {"address": 0x7E0, "size": (16, 16), "units": "mg/stroke"},
            "Ignition Map": {"address": 0x7E0, "size": (16, 16), "units": "° BTDC"},
            "Boost Map": {"address": 0x7E0, "size": (12, 12), "units": "kPa"}
        }
        
    def connect(self, bus):
        self.bus = bus
        # Send initialization sequence
        init_msg = can.Message(arbitration_id=0x7DF, data=[0x01, 0x00], is_extended_id=False)
        self.bus.send(init_msg)
        
    def read_map(self, map_name):
        if map_name not in self.map_definitions:
            raise ValueError(f"Map {map_name} not supported")
            
        # Simulate map reading
        map_def = self.map_definitions[map_name]
        rows, cols = map_def["size"]
        return np.random.uniform(low=0, high=100, size=(rows, cols))
        
    def read_live_data(self, pids):
        # Simulate live data reading
        return {
            "RPM": np.random.randint(800, 7000),
            "Speed": np.random.randint(0, 180),
            "Coolant Temp": np.random.randint(70, 110),
            "Throttle Position": np.random.uniform(0, 100)
        }

class CANProtocol(ProtocolHandler):
    """Modern CAN Protocol Implementation"""
    def __init__(self):
        super().__init__()
        self.name = "CAN"
        self.supported_ecus = ["ECU", "TCU", "ABS", "TCS"]
        self.map_definitions = {
            "Fuel Map": {"address": 0x7E0, "size": (20, 20), "units": "mg/stroke"},
            "Ignition Map": {"address": 0x7E0, "size": (20, 20), "units": "° BTDC"},
            "Boost Map": {"address": 0x7E0, "size": (16, 16), "units": "kPa"},
            "Shift Map": {"address": 0x7E1, "size": (12, 12), "units": "RPM"},
            "ABS Pressure Map": {"address": 0x7E2, "size": (8, 8), "units": "bar"},
            "TCS Slip Map": {"address": 0x7E3, "size": (10, 10), "units": "%"}
        }
        
    def connect(self, bus):
        self.bus = bus
        # Send CAN initialization
        init_msg = can.Message(arbitration_id=0x000, data=[0x01, 0x0F], is_extended_id=True)
        self.bus.send(init_msg)
        
    def read_map(self, map_name):
        if map_name not in self.map_definitions:
            raise ValueError(f"Map {map_name} not supported")
            
        map_def = self.map_definitions[map_name]
        rows, cols = map_def["size"]
        
        # Generate realistic map data
        if "Fuel" in map_name:
            base = np.linspace(5, 25, cols)
            return np.array([base * (1 + i/10) for i in range(rows)])
        elif "Ignition" in map_name:
            base = np.linspace(35, 5, cols)
            return np.array([base + i for i in range(rows)])
        elif "Boost" in map_name:
            return np.linspace(90, 200, rows*cols).reshape(rows, cols)
        elif "Shift" in map_name:
            return np.linspace(2000, 6500, rows*cols).reshape(rows, cols)
        elif "ABS" in map_name:
            return np.linspace(10, 150, rows*cols).reshape(rows, cols)
        elif "TCS" in map_name:
            return np.linspace(1, 15, rows*cols).reshape(rows, cols)
            
        return np.zeros((rows, cols))

class ConnectionManager:
    """Manage vehicle connection and protocols"""
    def __init__(self):
        self.bus = None
        self.protocol = None
        self.connected = False
        self.protocols = {
            'obd2': OBD2Protocol,
            'can': CANProtocol
        }
        
    def connect(self, protocol='can', interface='socketcan', channel='can0'):
        """Connect to vehicle bus"""
        try:
            self.bus = can.interface.Bus(channel=channel, bustype=interface)
            self.protocol = self._get_protocol(protocol)()
            self.protocol.connect(self.bus)
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from vehicle"""
        if self.bus:
            self.bus.shutdown()
        self.connected = False
        self.protocol = None
        
    def _get_protocol(self, protocol):
        """Get protocol handler"""
        return self.protocols.get(protocol.lower(), CANProtocol)
        
    def read_map(self, map_name):
        """Read specific map from ECU"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
        return self.protocol.read_map(map_name)
        
    def write_map(self, map_data, map_name):
        """Write map to ECU"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
        # In real implementation, this would send the data
        print(f"Writing {map_name} to ECU")
        return True
        
    def flash_ecu(self, image_data):
        """Flash complete ECU image"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
        # In real implementation, this would flash the ECU
        print("Flashing ECU...")
        return True
        
    def read_live_data(self, pids):
        """Read live data parameters"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
        return self.protocol.read_live_data(pids)
        
    def get_supported_maps(self):
        """Get supported maps for current protocol"""
        if not self.connected:
            return []
        return self.protocol.get_supported_maps()

class SafetySystem:
    """Safety and validation system"""
    def __init__(self):
        self.checksums = {}
        self.backups = {}
        self.write_protection = True
        
    def validate_write(self, map_data, original_data=None):
        """Validate write operation"""
        if self.write_protection:
            # Check if changes are within safe limits
            if original_data is not None:
                max_change = np.max(np.abs(map_data - original_data))
                if max_change > 20:  # Arbitrary safety threshold
                    return False, "Changes exceed safe limits"
                    
            # Check for extreme values
            if np.any(map_data > 1000) or np.any(map_data < 0):
                return False, "Values out of acceptable range"
                
        return True, ""
        
    def validate_flash(self, image_data):
        """Validate flash operation"""
        # Check image signature and size
        if len(image_data) < 1024:
            return False, "Invalid image size"
            
        # Check for known bad images
        if image_data[:4] == b'DEAD':
            return False, "Known bad image"
            
        return True, ""
        
    def create_backup(self, data, map_name):
        """Create safety backup"""
        backup_id = str(uuid.uuid4())
        import time
        self.backups[backup_id] = {
            'data': data,
            'map_name': map_name,
            'timestamp': time.time()  # current timestamp in seconds
        }
        return backup_id
        
    def restore_backup(self, backup_id):
        """Restore from backup"""
        if backup_id in self.backups:
            return self.backups[backup_id]['data']
        return None
        
    def checksum_data(self, data):
        """Calculate checksum for validation"""
        if isinstance(data, np.ndarray):
            flat_data = data.flatten()
            return sum(flat_data) % 65536
        return sum(data) % 65536

class VehicleDatabase:
    """Vehicle definition database"""
    def __init__(self):
        self.db = {}
        self.load_database()
        
    def load_database(self):
        """Load vehicle definitions from JSON"""
        try:
            # In a real implementation, this would load from a file
            self.db = {
                "toyota_supra_2020": {
                    "make": "Toyota",
                    "model": "Supra",
                    "year": 2020,
                    "protocol": "can",
                    "ecus": ["ECU", "TCU", "ABS", "TCS"],
                    "maps": {
                        "ECU": ["Fuel Map", "Ignition Map", "Boost Map"],
                        "TCU": ["Shift Map"],
                        "ABS": ["ABS Pressure Map"],
                        "TCS": ["TCS Slip Map"]
                    }
                },
                "honda_civic_2015": {
                    "make": "Honda",
                    "model": "Civic",
                    "year": 2015,
                    "protocol": "obd2",
                    "ecus": ["ECU", "TCU"],
                    "maps": {
                        "ECU": ["Fuel Map", "Ignition Map"],
                        "TCU": ["Shift Map"]
                    }
                }
            }
        except Exception as e:
            print(f"Error loading database: {e}")
            self.db = {}
            
    def get_vehicle(self, make, model, year):
        """Get vehicle definition"""
        key = f"{make.lower()}_{model.lower()}_{year}"
        return self.db.get(key)
        
    def get_map_definitions(self, make, model, year, ecu_type):
        """Get map locations for vehicle"""
        vehicle = self.get_vehicle(make, model, year)
        if vehicle:
            return vehicle['maps'].get(ecu_type, [])
        return []
        
    def get_supported_vehicles(self):
        """Get list of supported vehicles"""
        return list(self.db.keys())

class FlashManager:
    """Manage ECU flashing operations"""
    def __init__(self, connection, safety):
        self.connection = connection
        self.safety = safety
        self.flash_in_progress = False
        
    def read_ecu_image(self):
        """Read full ECU image"""
        # Simulate reading ECU image
        return b'ECU_IMAGE_' + os.urandom(1024)
        
    def write_ecu_image(self, image_data):
        """Write full ECU image"""
        valid, msg = self.safety.validate_flash(image_data)
        if not valid:
            return False, msg
            
        result = self.connection.flash_ecu(image_data)
        return result, "Flash successful" if result else "Flash failed"
        
    def recover_ecu(self, backup_image):
        """Recover ECU from backup"""
        return self.write_ecu_image(backup_image)

# =================
# GUI COMPONENTS
# =================

class MapVisualizationWidget(QWidget):
    """2D/3D Map Visualization Widget"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
    def plot_map(self, map_data, title="", xlabel="", ylabel="", zlabel=""):
        """Plot 2D map data"""
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        
        if map_data.ndim == 2:
            c = self.ax.imshow(map_data, cmap='viridis', origin='lower', aspect='auto')
            self.figure.colorbar(c, ax=self.ax, label=zlabel)
            self.ax.set_title(title)
            self.ax.set_xlabel(xlabel)
            self.ax.set_ylabel(ylabel)
        elif map_data.ndim == 1:
            self.ax.plot(map_data)
            self.ax.set_title(title)
            self.ax.set_xlabel(xlabel)
            self.ax.set_ylabel(ylabel)
            
        self.canvas.draw()

class LiveDataWidget(QWidget):
    """Real-time parameter monitoring widget"""
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        layout.addWidget(QLabel("Real-time Parameters"))
        layout.addWidget(self.table)
        self.setLayout(layout)
        
    def update_data(self, data):
        """Update with new live data"""
        self.table.setRowCount(len(data))
        for i, (param, value) in enumerate(data.items()):
            self.table.setItem(i, 0, QTableWidgetItem(param))
            self.table.setItem(i, 1, QTableWidgetItem(str(value)))

class BaseTunerModule(QWidget):
    """Base class for all tuning modules"""
    def __init__(self, connection, safety, vehicle_db, module_name, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.safety = safety
        self.vehicle_db = vehicle_db
        self.module_name = module_name
        self.current_map = None
        self.original_map = None
        self.map_name = ""
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        main_layout = QVBoxLayout()
        
        # Top controls
        top_layout = QHBoxLayout()
        self.map_selector = QComboBox()
        self.map_selector.currentIndexChanged.connect(self.load_map)
        top_layout.addWidget(QLabel("Select Map:"))
        top_layout.addWidget(self.map_selector, 1)
        
        self.load_btn = QPushButton("Read from ECU")
        self.load_btn.clicked.connect(self.read_ecu)
        top_layout.addWidget(self.load_btn)
        
        self.save_btn = QPushButton("Save to File")
        self.save_btn.clicked.connect(self.save_map)
        top_layout.addWidget(self.save_btn)
        
        self.apply_btn = QPushButton("Write to ECU")
        self.apply_btn.clicked.connect(self.write_ecu)
        top_layout.addWidget(self.apply_btn)
        
        main_layout.addLayout(top_layout)
        
        # Splitter for map visualization and editor
        splitter = QSplitter(Qt.Horizontal)
        
        # Map visualization
        self.visualization = MapVisualizationWidget()
        splitter.addWidget(self.visualization)
        
        # Map editor table
        self.map_table = QTableWidget()
        self.map_table.setEditTriggers(QTableWidget.DoubleClicked)
        self.map_table.cellChanged.connect(self.map_cell_changed)
        splitter.addWidget(self.map_table)
        
        splitter.setSizes([400, 400])
        main_layout.addWidget(splitter, 1)
        
        # Live data monitoring
        self.live_data = LiveDataWidget()
        main_layout.addWidget(self.live_data)
        
        self.setLayout(main_layout)
        
    def update_map_list(self, maps):
        """Update the map selection dropdown"""
        self.map_selector.clear()
        self.map_selector.addItems(maps)
        
    def load_map(self, index):
        """Load selected map from ECU"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        self.map_name = self.map_selector.currentText()
        try:
            self.current_map = self.connection.read_map(self.map_name)
            self.original_map = self.current_map.copy()
            self.display_map()
            self.backup_id = self.safety.create_backup(self.current_map, self.map_name)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load map: {str(e)}")
            
    def display_map(self):
        """Display current map in table and visualization"""
        if self.current_map is None:
            return
            
        # Update table
        rows, cols = self.current_map.shape
        self.map_table.setRowCount(rows)
        self.map_table.setColumnCount(cols)
        
        for r in range(rows):
            for c in range(cols):
                item = QTableWidgetItem(f"{self.current_map[r, c]:.2f}")
                self.map_table.setItem(r, c, item)
                
        # Update visualization
        self.visualization.plot_map(
            self.current_map, 
            title=self.map_name,
            xlabel="Load",
            ylabel="RPM",
            zlabel="Value"
        )
        
    def map_cell_changed(self, row, column):
        """Handle cell editing in map table"""
        if self.current_map is None:
            return
            
        try:
            new_value = float(self.map_table.item(row, column).text())
            self.current_map[row, column] = new_value
            self.visualization.plot_map(self.current_map)
        except ValueError:
            # Revert to original value if invalid input
            self.map_table.item(row, column).setText(f"{self.current_map[row, column]:.2f}")
            
    def read_ecu(self):
        """Read full ECU data"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        try:
            # For demo, just reload the current map
            if self.map_name:
                self.load_map(self.map_selector.currentIndex())
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to read ECU: {str(e)}")
            
    def write_ecu(self):
        """Write modified maps to ECU"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        if self.current_map is None:
            QMessageBox.warning(self, "No Map", "Please load a map first")
            return
            
        valid, msg = self.safety.validate_write(self.current_map, self.original_map)
        if not valid:
            QMessageBox.warning(self, "Safety Check Failed", msg)
            return
            
        try:
            success = self.connection.write_map(self.current_map, self.map_name)
            if success:
                QMessageBox.information(self, "Success", "Map written to ECU")
                self.original_map = self.current_map.copy()
            else:
                QMessageBox.critical(self, "Error", "Failed to write map to ECU")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to write map: {str(e)}")
            
    def save_map(self):
        """Save current map to file"""
        if self.current_map is None:
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Map", "", "Map Files (*.map);;All Files (*)"
        )
        
        if file_path:
            try:
                np.savetxt(file_path, self.current_map, fmt='%.6f')
                QMessageBox.information(self, "Success", "Map saved successfully")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save map: {str(e)}")
                
    def update_live_data(self, data):
        """Update live data display"""
        self.live_data.update_data(data)

class ECUTuner(BaseTunerModule):
    """ECU Tuning Module"""
    def __init__(self, connection, safety, vehicle_db, parent=None):
        super().__init__(connection, safety, vehicle_db, "ECU", parent)

class TCUTuner(BaseTunerModule):
    """Transmission Control Tuning Module"""
    def __init__(self, connection, safety, vehicle_db, parent=None):
        super().__init__(connection, safety, vehicle_db, "TCU", parent)

class ABSTuner(BaseTunerModule):
    """ABS Tuning Module"""
    def __init__(self, connection, safety, vehicle_db, parent=None):
        super().__init__(connection, safety, vehicle_db, "ABS", parent)

class TCSTuner(BaseTunerModule):
    """Traction Control Tuning Module"""
    def __init__(self, connection, safety, vehicle_db, parent=None):
        super().__init__(connection, safety, vehicle_db, "TCS", parent)

class CANxTuneMainWindow(QMainWindow):
    """Main Application Window"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CANxTune - Open Source Vehicle Tuning Suite")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize core systems
        self.connection = ConnectionManager()
        self.safety = SafetySystem()
        self.vehicle_db = VehicleDatabase()
        self.flash_manager = FlashManager(self.connection, self.safety)
        
        # Setup UI
        self.init_ui()
        
        # Setup live data timer
        self.live_data_timer = QTimer()
        self.live_data_timer.timeout.connect(self.update_live_data)
        self.live_data_timer.start(1000)  # Update every second
        
    def init_ui(self):
        """Initialize the main UI"""
        # Create menu bar
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('File')
        
        connect_action = QAction('Connect to Vehicle', self)
        connect_action.triggered.connect(self.connect_to_vehicle)
        file_menu.addAction(connect_action)
        
        disconnect_action = QAction('Disconnect', self)
        disconnect_action.triggered.connect(self.disconnect_from_vehicle)
        file_menu.addAction(disconnect_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu('Tools')
        
        flash_action = QAction('Flash ECU', self)
        flash_action.triggered.connect(self.flash_ecu)
        tools_menu.addAction(flash_action)
        
        backup_action = QAction('Create Backup', self)
        backup_action.triggered.connect(self.create_backup)
        tools_menu.addAction(backup_action)
        
        restore_action = QAction('Restore Backup', self)
        restore_action.triggered.connect(self.restore_backup)
        tools_menu.addAction(restore_action)
        
        # Safety menu
        safety_menu = menubar.addMenu('Safety')
        
        protection_action = QAction('Toggle Write Protection', self)
        protection_action.triggered.connect(self.toggle_write_protection)
        safety_menu.addAction(protection_action)
        
        # Create central tab widget
        self.tabs = QTabWidget()
        
        # Create tuning modules
        self.ecu_tuner = ECUTuner(self.connection, self.safety, self.vehicle_db)
        self.tcu_tuner = TCUTuner(self.connection, self.safety, self.vehicle_db)
        self.abs_tuner = ABSTuner(self.connection, self.safety, self.vehicle_db)
        self.tcs_tuner = TCSTuner(self.connection, self.safety, self.vehicle_db)
        
        # Add tabs
        self.tabs.addTab(self.ecu_tuner, "ECU Tuning")
        self.tabs.addTab(self.tcu_tuner, "TCU Tuning")
        self.tabs.addTab(self.abs_tuner, "ABS Tuning")
        self.tabs.addTab(self.tcs_tuner, "TCS Tuning")
        
        self.setCentralWidget(self.tabs)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # Connection status
        self.connection_status = QLabel("Disconnected")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        self.status_bar.addPermanentWidget(self.connection_status)
        
        # Safety status
        self.safety_status = QLabel("Write Protection: ON")
        self.safety_status.setStyleSheet("color: green;")
        self.status_bar.addPermanentWidget(self.safety_status)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumWidth(200)
        self.progress_bar.setVisible(False)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
    def connect_to_vehicle(self):
        """Connect to vehicle"""
        # In a real implementation, this would show a dialog for vehicle selection
        vehicle = self.vehicle_db.get_vehicle("toyota", "supra", 2020)
        if not vehicle:
            QMessageBox.critical(self, "Error", "Vehicle not supported")
            return
            
        try:
            # For demo, we'll use simulated connection
            success = self.connection.connect(protocol=vehicle['protocol'])
            if success:
                self.connection_status.setText("Connected to Toyota Supra 2020")
                self.connection_status.setStyleSheet("color: green; font-weight: bold;")
                
                # Update map lists for each module
                self.ecu_tuner.update_map_list(
                    self.vehicle_db.get_map_definitions("toyota", "supra", 2020, "ECU")
                )
                self.tcu_tuner.update_map_list(
                    self.vehicle_db.get_map_definitions("toyota", "supra", 2020, "TCU")
                )
                self.abs_tuner.update_map_list(
                    self.vehicle_db.get_map_definitions("toyota", "supra", 2020, "ABS")
                )
                self.tcs_tuner.update_map_list(
                    self.vehicle_db.get_map_definitions("toyota", "supra", 2020, "TCS")
                )
            else:
                QMessageBox.critical(self, "Connection Failed", "Could not connect to vehicle")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Connection failed: {str(e)}")
            
    def disconnect_from_vehicle(self):
        """Disconnect from vehicle"""
        self.connection.disconnect()
        self.connection_status.setText("Disconnected")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        
    def flash_ecu(self):
        """Flash ECU"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate progress
        
        try:
            # Read current ECU image
            ecu_image = self.flash_manager.read_ecu_image()
            
            # Save backup
            backup_id = self.safety.create_backup(ecu_image, "ECU_IMAGE")
            
            # Flash ECU (in real implementation, this would write modified image)
            success, msg = self.flash_manager.write_ecu_image(ecu_image)
            
            self.progress_bar.setVisible(False)
            
            if success:
                QMessageBox.information(self, "Success", msg)
            else:
                QMessageBox.critical(self, "Error", msg)
        except Exception as e:
            self.progress_bar.setVisible(False)
            QMessageBox.critical(self, "Error", f"Flashing failed: {str(e)}")
            
    def create_backup(self):
        """Create backup of current maps"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        # For demo, we'll just show a message
        QMessageBox.information(self, "Backup Created", "Current maps have been backed up")
        
    def restore_backup(self):
        """Restore from backup"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        # For demo, we'll just show a message
        QMessageBox.information(self, "Restore", "Select a backup to restore")
        
    def toggle_write_protection(self):
        """Toggle write protection"""
        self.safety.write_protection = not self.safety.write_protection
        status = "ON" if self.safety.write_protection else "OFF"
        color = "green" if self.safety.write_protection else "red"
        self.safety_status.setText(f"Write Protection: {status}")
        self.safety_status.setStyleSheet(f"color: {color};")
        
    def update_live_data(self):
        """Update live data for all modules"""
        if not self.connection.connected:
            return
            
        try:
            # Read common parameters
            live_data = self.connection.read_live_data(["RPM", "Speed", "Coolant Temp", "Throttle Position"])
            
            # Update each module
            self.ecu_tuner.update_live_data(live_data)
            self.tcu_tuner.update_live_data(live_data)
            self.abs_tuner.update_live_data(live_data)
            self.tcs_tuner.update_live_data(live_data)
        except Exception as e:
            print(f"Error reading live data: {e}")

# =================
# APPLICATION START
# =================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle("Fusion")
    
    window = CANxTuneMainWindow()
    window.show()
    
    sys.exit(app.exec_())