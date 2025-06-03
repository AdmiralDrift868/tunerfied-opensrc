import sys
import os
import json
import uuid
import numpy as np
import can
import logging
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QComboBox, QTableWidget, QTableWidgetItem, QPushButton, 
                             QLabel, QLineEdit, QMessageBox, QFileDialog, QAction, QStatusBar,
                             QSplitter, QTreeWidget, QTreeWidgetItem, QHeaderView, QProgressBar,
                             QDialog, QFormLayout, QDialogButtonBox)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QIcon
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import hashlib

matplotlib.use('Qt5Agg')

# =======================
# ENHANCED LOGGING SYSTEM
# =======================
class LogManager:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(LogManager, cls).__new__(cls)
            cls._instance.setup_logging()
        return cls._instance

    def setup_logging(self):
        self.logger = logging.getLogger('CANxTune')
        self.logger.setLevel(logging.DEBUG)
        
        # Create file handler
        log_file = os.path.join(os.path.dirname(__file__), 'canxtune.log')
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.DEBUG)
        
        # Create console handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        # Add handlers
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)
        
    def log(self, level, message, exc_info=None):
        getattr(self.logger, level)(message, exc_info=exc_info)
        
    @staticmethod
    def get_logger():
        return LogManager().logger

logger = LogManager.get_logger()

# ===================
# TYPE DEFINITIONS
# ===================
class ECUType(Enum):
    ENGINE = "ECU"
    TRANSMISSION = "TCU"
    ABS = "ABS"
    TRACTION_CONTROL = "TCS"

class ProtocolType(Enum):
    OBD2 = "obd2"
    CAN = "can"
    J1939 = "j1939"
    UDS = "uds"

class OperationStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    WARNING = 2
    SAFETY_BLOCK = 3

# ======================
# CORE SYSTEM COMPONENTS
# ======================
@dataclass
class MapDefinition:
    name: str
    address: int
    size: Tuple[int, int]
    units: str
    min_value: float
    max_value: float
    axis_labels: Tuple[str, str] = ("RPM", "Load")
    data_type: str = "float32"
    scaling_factor: float = 1.0
    offset: float = 0.0
    checksum_address: Optional[int] = None
    description: str = ""
    read_only: bool = False

@dataclass
class ECUDefinition:
    ecu_type: ECUType
    protocol: ProtocolType
    maps: Dict[str, MapDefinition] = field(default_factory=dict)
    base_address: int = 0x7E0
    response_id: int = 0x7E8
    security_level: int = 0
    flash_size: int = 1024 * 1024  # 1MB default

@dataclass
class VehicleDefinition:
    make: str
    model: str
    year: int
    vin_pattern: str
    ecus: Dict[ECUType, ECUDefinition]
    supported_protocols: List[ProtocolType]
    validation_key: Optional[bytes] = None
    requires_signature: bool = False

class ProtocolHandler(QObject):
    """Base class for protocol handlers with thread-safe operations"""
    connection_established = pyqtSignal(bool)
    data_received = pyqtSignal(dict)
    operation_completed = pyqtSignal(OperationStatus, str)
    
    def __init__(self):
        super().__init__()
        self.name = "Base Protocol"
        self.connected = False
        self.bus = None
        self.executor = ThreadPoolExecutor(max_workers=4)
        
    def connect_async(self, interface='socketcan', channel='can0'):
        """Establish connection to vehicle asynchronously"""
        future = self.executor.submit(self._connect, interface, channel)
        future.add_done_callback(self._handle_connect_result)
        
    def _connect(self, interface, channel):
        try:
            self.bus = can.interface.Bus(channel=channel, bustype=interface)
            self._initialize_protocol()
            self.connected = True
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}", exc_info=True)
            return False
            
    def _handle_connect_result(self, future):
        success = future.result()
        self.connection_established.emit(success)
        if success:
            logger.info(f"Connected using {self.name} protocol")
        else:
            logger.error(f"Connection failed using {self.name} protocol")
            
    def _initialize_protocol(self):
        """Protocol-specific initialization"""
        pass
        
    def disconnect(self):
        """Disconnect from vehicle"""
        if self.bus:
            self.bus.shutdown()
        self.connected = False
        self.bus = None
        logger.info("Disconnected from vehicle")
        
    def read_map_async(self, map_def: MapDefinition):
        """Read specific map from ECU asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        future = self.executor.submit(self._read_map, map_def)
        future.add_done_callback(self._handle_read_result)
        
    def _read_map(self, map_def: MapDefinition) -> np.ndarray:
        """Protocol-specific map reading implementation"""
        raise NotImplementedError
        
    def _handle_read_result(self, future):
        try:
            map_data = future.result()
            self.data_received.emit({"type": "map_data", "data": map_data})
            self.operation_completed.emit(OperationStatus.SUCCESS, "Map read successfully")
        except Exception as e:
            logger.error(f"Map read failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            
    def write_map_async(self, map_data: np.ndarray, map_def: MapDefinition):
        """Write map to ECU asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        future = self.executor.submit(self._write_map, map_data, map_def)
        future.add_done_callback(self._handle_write_result)
        
    def _write_map(self, map_data: np.ndarray, map_def: MapDefinition) -> bool:
        """Protocol-specific map writing implementation"""
        raise NotImplementedError
        
    def _handle_write_result(self, future):
        try:
            success = future.result()
            if success:
                self.operation_completed.emit(OperationStatus.SUCCESS, "Map written successfully")
                logger.info("Map written to ECU")
            else:
                self.operation_completed.emit(OperationStatus.FAILURE, "Map write failed")
                logger.warning("Map write operation failed")
        except Exception as e:
            logger.error(f"Map write failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            
    def flash_ecu_async(self, image_data: bytes, ecu_def: ECUDefinition):
        """Flash complete ECU image asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        future = self.executor.submit(self._flash_ecu, image_data, ecu_def)
        future.add_done_callback(self._handle_flash_result)
        
    def _flash_ecu(self, image_data: bytes, ecu_def: ECUDefinition) -> bool:
        """Protocol-specific ECU flashing implementation"""
        raise NotImplementedError
        
    def _handle_flash_result(self, future):
        try:
            success = future.result()
            if success:
                self.operation_completed.emit(OperationStatus.SUCCESS, "ECU flashed successfully")
                logger.info("ECU flashed successfully")
            else:
                self.operation_completed.emit(OperationStatus.FAILURE, "ECU flash failed")
                logger.error("ECU flash operation failed")
        except Exception as e:
            logger.error(f"ECU flash failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            
    def read_live_data_async(self, pids: List[str]):
        """Read live data parameters asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        future = self.executor.submit(self._read_live_data, pids)
        future.add_done_callback(self._handle_live_data_result)
        
    def _read_live_data(self, pids: List[str]) -> Dict[str, Any]:
        """Protocol-specific live data reading"""
        raise NotImplementedError
        
    def _handle_live_data_result(self, future):
        try:
            live_data = future.result()
            self.data_received.emit({"type": "live_data", "data": live_data})
        except Exception as e:
            logger.error(f"Live data read failed: {e}", exc_info=True)

class CANProtocol(ProtocolHandler):
    """Modern CAN Protocol Implementation"""
    def __init__(self):
        super().__init__()
        self.name = ProtocolType.CAN.value
        logger.info("Initializing CAN protocol handler")
        
    def _initialize_protocol(self):
        # Send CAN initialization sequence
        init_msg = can.Message(
            arbitration_id=0x000, 
            data=[0x01, 0x0F], 
            is_extended_id=True
        )
        self.bus.send(init_msg)
        logger.debug("Sent CAN initialization sequence")
        
    def _read_map(self, map_def: MapDefinition) -> np.ndarray:
        logger.info(f"Reading map: {map_def.name}")
        rows, cols = map_def.size
        
        # Simulate realistic CAN map reading with checksum verification
        map_data = np.zeros((rows, cols), dtype=map_def.data_type)
        
        # Generate realistic map data patterns
        if "Fuel" in map_def.name:
            base = np.linspace(5, 25, cols)
            map_data = np.array([base * (1 + i/10) for i in range(rows)])
        elif "Ignition" in map_def.name:
            base = np.linspace(35, 5, cols)
            map_data = np.array([base + i for i in range(rows)])
        elif "Boost" in map_def.name:
            map_data = np.linspace(90, 200, rows*cols).reshape(rows, cols)
        
        # Apply scaling and offset
        map_data = (map_data * map_def.scaling_factor) + map_def.offset
        
        # Simulate transmission delay
        QThread.msleep(rows * cols * 5)
        
        return map_data
        
    def _write_map(self, map_data: np.ndarray, map_def: MapDefinition) -> bool:
        logger.info(f"Writing map: {map_def.name}")
        
        # Simulate write operation with verification
        # In real implementation, would include checksum update
        rows, cols = map_def.size
        if map_data.shape != (rows, cols):
            logger.error(f"Map size mismatch: expected {rows}x{cols}, got {map_data.shape}")
            return False
            
        # Simulate write process with progress
        for i in range(rows):
            # Simulate row write
            QThread.msleep(50)
            
        # Verify write
        read_back = self._read_map(map_def)
        if not np.allclose(map_data, read_back, atol=0.1):
            logger.error("Write verification failed")
            return False
            
        return True

class ConnectionManager(QObject):
    """Manage vehicle connection and protocols with thread safety"""
    connection_status_changed = pyqtSignal(bool)
    vehicle_identified = pyqtSignal(VehicleDefinition)
    
    def __init__(self):
        super().__init__()
        self.protocol_handler = None
        self.connected = False
        self.current_vehicle = None
        self.protocols = {
            ProtocolType.CAN: CANProtocol,
            # Other protocols would be added here
        }
        logger.info("Connection manager initialized")
        
    def connect_async(self, protocol: ProtocolType, vehicle: VehicleDefinition, 
                     interface='socketcan', channel='can0'):
        """Connect to vehicle bus asynchronously"""
        if self.connected:
            self.disconnect()
            
        self.protocol_handler = self.protocols[protocol]()
        self.protocol_handler.connection_established.connect(self._handle_connection_result)
        self.protocol_handler.connect_async(interface, channel)
        self.current_vehicle = vehicle
        logger.info(f"Connecting to vehicle using {protocol.value} protocol")
        
    def _handle_connection_result(self, success):
        self.connected = success
        self.connection_status_changed.emit(success)
        if success:
            self.vehicle_identified.emit(self.current_vehicle)
            
    def disconnect(self):
        """Disconnect from vehicle"""
        if self.protocol_handler:
            self.protocol_handler.disconnect()
        self.connected = False
        self.protocol_handler = None
        self.connection_status_changed.emit(False)
        logger.info("Disconnected from vehicle")
        
    def read_map_async(self, map_def: MapDefinition):
        """Read specific map from ECU asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.read_map_async(map_def)
        
    def write_map_async(self, map_data: np.ndarray, map_def: MapDefinition):
        """Write map to ECU asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.write_map_async(map_data, map_def)
        
    def flash_ecu_async(self, image_data: bytes, ecu_def: ECUDefinition):
        """Flash complete ECU image asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.flash_ecu_async(image_data, ecu_def)
        
    def read_live_data_async(self, pids: List[str]):
        """Read live data parameters asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.read_live_data_async(pids)

class SafetySystem:
    """Enhanced safety and validation system"""
    def __init__(self):
        self.backups = {}
        self.write_protection = True
        self.max_value_change = 20.0  # Percentage
        self.max_absolute_deviation = 50.0
        self.logger = logger.getChild("SafetySystem")
        self.logger.info("Safety system initialized")
        
    def validate_write(self, map_data: np.ndarray, 
                      original_data: np.ndarray, 
                      map_def: MapDefinition) -> Tuple[bool, str]:
        """Enhanced write validation with multiple safety checks"""
        if map_def.read_only:
            return False, "Map is read-only"
            
        # Value range check
        if np.any(map_data < map_def.min_value) or np.any(map_data > map_def.max_value):
            return False, f"Values out of range ({map_def.min_value}-{map_def.max_value})"
            
        if original_data is not None:
            # Percentage change check
            abs_change = np.abs(map_data - original_data)
            percent_change = (abs_change / np.abs(original_data)) * 100
            max_percent_change = np.max(percent_change)
            
            if max_percent_change > self.max_value_change:
                return False, f"Change exceeds {self.max_value_change}% limit"
                
            # Absolute deviation check
            max_abs_deviation = np.max(abs_change)
            if max_abs_deviation > self.max_absolute_deviation:
                return False, f"Deviation exceeds {self.max_absolute_deviation} absolute limit"
                
        # Pattern analysis (simple example)
        if "Fuel" in map_def.name:
            if np.max(map_data) > 50:  # Arbitrary safety limit
                return False, "Fuel map value exceeds safety threshold"
                
        return True, ""
        
    def validate_flash(self, image_data: bytes, ecu_def: ECUDefinition) -> Tuple[bool, str]:
        """Enhanced flash validation"""
        # Size validation
        if len(image_data) != ecu_def.flash_size:
            return False, f"Invalid image size: expected {ecu_def.flash_size} bytes"
            
        # Signature verification
        if ecu_def.security_level > 0:
            if not self._verify_signature(image_data):
                return False, "Invalid image signature"
                
        # Checksum validation
        if not self._verify_checksum(image_data):
            return False, "Checksum validation failed"
            
        return True, ""
        
    def create_backup(self, data: Any, identifier: str) -> str:
        """Create safety backup with versioning"""
        backup_id = str(uuid.uuid4())
        timestamp = QDateTime.currentDateTime().toString(Qt.ISODate)
        
        self.backups[backup_id] = {
            'id': backup_id,
            'data': data,
            'identifier': identifier,
            'timestamp': timestamp,
            'checksum': self._calculate_checksum(data),
            'version': 1
        }
        self.logger.info(f"Created backup {backup_id} for {identifier}")
        return backup_id
        
    def restore_backup(self, backup_id: str) -> Any:
        """Restore from backup with validation"""
        backup = self.backups.get(backup_id)
        if not backup:
            return None
            
        # Validate checksum
        if backup['checksum'] != self._calculate_checksum(backup['data']):
            self.logger.error(f"Backup {backup_id} checksum validation failed")
            return None
            
        self.logger.info(f"Restored backup {backup_id}")
        return backup['data']
        
    def _calculate_checksum(self, data: Any) -> str:
        """Calculate secure checksum for validation"""
        if isinstance(data, np.ndarray):
            data_bytes = data.tobytes()
        elif isinstance(data, bytes):
            data_bytes = data
        else:
            data_bytes = str(data).encode('utf-8')
            
        return hashlib.sha256(data_bytes).hexdigest()
        
    def _verify_signature(self, image_data: bytes) -> bool:
        """Verify digital signature (placeholder)"""
        # In real implementation, would use cryptographic signature verification
        return True
        
    def _verify_checksum(self, image_data: bytes) -> bool:
        """Verify checksum (placeholder)"""
        # In real implementation, would validate against ECU-specific checksum
        return True

class VehicleDatabase:
    """Enhanced vehicle definition database with versioning"""
    def __init__(self):
        self.db = {}
        self.version = "1.0.0"
        self.load_database()
        logger.info(f"Vehicle database initialized (version {self.version})")
        
    def load_database(self):
        """Load vehicle definitions from JSON"""
        try:
            # In real implementation, this would load from a file/API
            self.db = {
                "toyota_supra_2020": VehicleDefinition(
                    make="Toyota",
                    model="Supra",
                    year=2020,
                    vin_pattern="JTD*",
                    ecus={
                        ECUType.ENGINE: ECUDefinition(
                            ecu_type=ECUType.ENGINE,
                            protocol=ProtocolType.CAN,
                            maps={
                                "Fuel Map": MapDefinition(
                                    name="Fuel Map",
                                    address=0x7E0,
                                    size=(20, 20),
                                    units="mg/stroke",
                                    min_value=0,
                                    max_value=50
                                ),
                                # Other maps...
                            }
                        ),
                        # Other ECUs...
                    },
                    supported_protocols=[ProtocolType.CAN],
                    requires_signature=True
                ),
                # Other vehicles...
            }
        except Exception as e:
            logger.error(f"Error loading database: {e}", exc_info=True)
            self.db = {}
            
    def get_vehicle(self, make: str, model: str, year: int) -> Optional[VehicleDefinition]:
        """Get vehicle definition"""
        key = f"{make.lower()}_{model.lower()}_{year}"
        return self.db.get(key)
        
    def get_map_definition(self, vehicle: VehicleDefinition, 
                          ecu_type: ECUType, map_name: str) -> Optional[MapDefinition]:
        """Get map definition for vehicle"""
        ecu_def = vehicle.ecus.get(ecu_type)
        if not ecu_def:
            return None
        return ecu_def.maps.get(map_name)
        
    def get_supported_vehicles(self) -> List[str]:
        """Get list of supported vehicles"""
        return list(self.db.keys())

class FlashManager:
    """Professional-grade ECU flashing manager"""
    def __init__(self, connection: ConnectionManager, safety: SafetySystem):
        self.connection = connection
        self.safety = safety
        self.flash_in_progress = False
        self.logger = logger.getChild("FlashManager")
        self.logger.info("Flash manager initialized")
        
    def read_ecu_image(self, ecu_def: ECUDefinition) -> bytes:
        """Read full ECU image with validation"""
        self.logger.info(f"Reading ECU image for {ecu_def.ecu_type.value}")
        # Simulate reading ECU image with progress
        image_data = b'ECU_IMAGE_' + os.urandom(ecu_def.flash_size - 10)
        
        # Add validation signature
        image_data += b'SIG'  # Placeholder for real signature
        
        return image_data
        
    def write_ecu_image(self, image_data: bytes, ecu_def: ECUDefinition) -> Tuple[bool, str]:
        """Write full ECU image with enhanced safety checks"""
        self.logger.info(f"Writing ECU image for {ecu_def.ecu_type.value}")
        valid, msg = self.safety.validate_flash(image_data, ecu_def)
        if not valid:
            self.logger.error(f"Flash validation failed: {msg}")
            return False, msg
            
        # Simulate flash process with progress reporting
        self.flash_in_progress = True
        try:
            # In real implementation, would send to connection manager
            # For simulation, just "flash"
            QThread.msleep(3000)  # Simulate flash time
            self.logger.info("ECU flash successful")
            return True, "Flash successful"
        except Exception as e:
            self.logger.error(f"Flash failed: {e}", exc_info=True)
            return False, str(e)
        finally:
            self.flash_in_progress = False
            
    def recover_ecu(self, backup_image: bytes, ecu_def: ECUDefinition) -> Tuple[bool, str]:
        """Recover ECU from backup"""
        return self.write_ecu_image(backup_image, ecu_def)

# =================
# GUI COMPONENTS
# =================
class ConnectionDialog(QDialog):
    """Professional connection setup dialog"""
    def __init__(self, vehicle_db: VehicleDatabase, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Vehicle Connection")
        self.setWindowIcon(QIcon(":/icons/connect.png"))
        self.vehicle_db = vehicle_db
        self.selected_vehicle = None
        self.init_ui()
        
    def init_ui(self):
        layout = QFormLayout()
        
        # Vehicle selection
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(self.vehicle_db.get_supported_vehicles())
        self.vehicle_combo.currentIndexChanged.connect(self.update_vehicle_info)
        layout.addRow("Vehicle:", self.vehicle_combo)
        
        # Vehicle info display
        self.vehicle_info = QLabel()
        layout.addRow(self.vehicle_info)
        
        # Protocol selection
        self.protocol_combo = QComboBox()
        layout.addRow("Protocol:", self.protocol_combo)
        
        # Interface selection
        self.interface_combo = QComboBox()
        self.interface_combo.addItems(["socketcan", "vector", "pcan", "virtual"])
        layout.addRow("Interface:", self.interface_combo)
        
        # Channel input
        self.channel_edit = QLineEdit("can0")
        layout.addRow("Channel:", self.channel_edit)
        
        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        self.setLayout(layout)
        self.update_vehicle_info(0)
        
    def update_vehicle_info(self, index):
        vehicle_name = self.vehicle_combo.currentText()
        vehicle = self.vehicle_db.get_vehicle(*vehicle_name.split('_'))
        
        if vehicle:
            self.selected_vehicle = vehicle
            info_text = (f"{vehicle.make} {vehicle.model} {vehicle.year}\n"
                         f"ECUs: {', '.join([e.value for e in vehicle.ecus.keys()])}")
            self.vehicle_info.setText(info_text)
            
            # Update protocols
            self.protocol_combo.clear()
            self.protocol_combo.addItems([p.value for p in vehicle.supported_protocols])
            
    def get_connection_params(self):
        return {
            "vehicle": self.selected_vehicle,
            "protocol": ProtocolType(self.protocol_combo.currentText()),
            "interface": self.interface_combo.currentText(),
            "channel": self.channel_edit.text()
        }

class MapVisualizationWidget(QWidget):
    """Enhanced 2D/3D Map Visualization with professional features"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
    def plot_map(self, map_data, title="", xlabel="", ylabel="", zlabel="", 
                contours=False, surface=False, interpolation='bilinear'):
        """Professional-grade map plotting"""
        self.figure.clear()
        
        if map_data.ndim == 2:
            if surface:
                self.ax = self.figure.add_subplot(111, projection='3d')
                x = np.arange(map_data.shape[1])
                y = np.arange(map_data.shape[0])
                X, Y = np.meshgrid(x, y)
                surf = self.ax.plot_surface(X, Y, map_data, cmap='viridis', 
                                           linewidth=0, antialiased=True)
                self.figure.colorbar(surf, ax=self.ax, label=zlabel)
            else:
                self.ax = self.figure.add_subplot(111)
                c = self.ax.imshow(map_data, cmap='viridis', origin='lower', 
                                  aspect='auto', interpolation=interpolation)
                
                if contours:
                    contour = self.ax.contour(map_data, colors='white', linewidths=0.5)
                    self.ax.clabel(contour, inline=True, fontsize=8)
                    
                self.figure.colorbar(c, ax=self.ax, label=zlabel)
                
            self.ax.set_title(title, fontsize=10)
            self.ax.set_xlabel(xlabel)
            self.ax.set_ylabel(ylabel)
            
            # Professional touch: add grid and improve tick labels
            self.ax.grid(True, linestyle='--', alpha=0.6)
            self.ax.tick_params(axis='both', which='major', labelsize=8)
            
        elif map_data.ndim == 1:
            self.ax = self.figure.add_subplot(111)
            self.ax.plot(map_data, linewidth=1.5)
            self.ax.set_title(title, fontsize=10)
            self.ax.set_xlabel(xlabel)
            self.ax.set_ylabel(ylabel)
            self.ax.grid(True, linestyle='--', alpha=0.6)
            
        self.canvas.draw()

class BaseTunerModule(QWidget):
    """Professional-grade base tuning module with enhanced features"""
    operation_status = pyqtSignal(OperationStatus, str)
    
    def __init__(self, connection: ConnectionManager, safety: SafetySystem, 
                 vehicle_db: VehicleDatabase, ecu_type: ECUType, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.safety = safety
        self.vehicle_db = vehicle_db
        self.ecu_type = ecu_type
        self.current_vehicle = None
        self.current_map = None
        self.original_map = None
        self.map_def = None
        
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        main_layout = QVBoxLayout()
        
        # Top toolbar
        toolbar = QHBoxLayout()
        
        # Map selector with search
        self.map_selector = QComboBox()
        self.map_selector.setEditable(True)
        self.map_selector.setInsertPolicy(QComboBox.NoInsert)
        toolbar.addWidget(QLabel("Map:"), 1)
        toolbar.addWidget(self.map_selector, 3)
        
        # Action buttons
        self.read_btn = QPushButton(QIcon(":/icons/read.png"), "Read")
        self.read_btn.setToolTip("Read from ECU")
        toolbar.addWidget(self.read_btn)
        
        self.save_btn = QPushButton(QIcon(":/icons/save.png"), "Save")
        self.save_btn.setToolTip("Save to file")
        toolbar.addWidget(self.save_btn)
        
        self.apply_btn = QPushButton(QIcon(":/icons/write.png"), "Write")
        self.apply_btn.setToolTip("Write to ECU")
        toolbar.addWidget(self.apply_btn)
        
        self.compare_btn = QPushButton(QIcon(":/icons/compare.png"), "Compare")
        self.compare_btn.setToolTip("Compare with original")
        toolbar.addWidget(self.compare_btn)
        
        self.undo_btn = QPushButton(QIcon(":/icons/undo.png"), "Undo")
        self.undo_btn.setToolTip("Revert changes")
        toolbar.addWidget(self.undo_btn)
        
        main_layout.addLayout(toolbar)
        
        # Splitter for visualization and editor
        splitter = QSplitter(Qt.Horizontal)
        
        # Enhanced visualization
        self.visualization = MapVisualizationWidget()
        splitter.addWidget(self.visualization)
        
        # Professional editor components
        editor_tabs = QTabWidget()
        
        # Table editor
        self.map_table = QTableWidget()
        self.map_table.setEditTriggers(QTableWidget.DoubleClicked)
        self.map_table.cellChanged.connect(self.map_cell_changed)
        editor_tabs.addTab(self.map_table, "Table Editor")
        
        # Graph editor
        graph_editor = QWidget()
        editor_tabs.addTab(graph_editor, "Graph Editor")
        
        # 3D editor
        editor_3d = QWidget()
        editor_tabs.addTab(editor_3d, "3D Editor")
        
        splitter.addWidget(editor_tabs)
        splitter.setSizes([500, 500])
        main_layout.addWidget(splitter, 1)
        
        # Status bar for module
        self.status_bar = QStatusBar()
        self.status_bar.setSizeGripEnabled(False)
        main_layout.addWidget(self.status_bar)
        
        self.setLayout(main_layout)
        
    def setup_connections(self):
        self.read_btn.clicked.connect(self.read_map)
        self.save_btn.clicked.connect(self.save_map)
        self.apply_btn.clicked.connect(self.write_map)
        self.compare_btn.clicked.connect(self.compare_map)
        self.undo_btn.clicked.connect(self.undo_changes)
        
    def set_vehicle(self, vehicle: VehicleDefinition):
        """Set current vehicle and update map list"""
        self.current_vehicle = vehicle
        self.update_map_list()
        
    def update_map_list(self):
        """Update the map selection dropdown"""
        if not self.current_vehicle:
            return
            
        ecu_def = self.current_vehicle.ecus.get(self.ecu_type)
        if not ecu_def:
            return
            
        self.map_selector.clear()
        self.map_selector.addItems(ecu_def.maps.keys())
        
    def read_map(self):
        """Read selected map from ECU"""
        if not self.connection.connected:
            self.operation_status.emit(OperationStatus.FAILURE, "Not connected to vehicle")
            return
            
        map_name = self.map_selector.currentText()
        if not map_name:
            return
            
        # Get map definition
        self.map_def = self.vehicle_db.get_map_definition(
            self.current_vehicle, self.ecu_type, map_name
        )
        
        if not self.map_def:
            self.operation_status.emit(OperationStatus.FAILURE, "Map definition not found")
            return
            
        self.status_bar.showMessage(f"Reading {map_name} from ECU...")
        self.connection.read_map_async(self.map_def)
        
    def display_map(self, map_data: np.ndarray):
        """Professional map display with multiple representations"""
        if map_data is None:
            return
            
        self.current_map = map_data
        self.original_map = map_data.copy()
        
        # Update table
        self.update_map_table()
        
        # Update visualization
        self.update_visualization()
        
        self.status_bar.showMessage("Map loaded successfully")
        
    def update_map_table(self):
        """Update table with current map data"""
        if self.current_map is None or self.map_def is None:
            return
            
        rows, cols = self.current_map.shape
        self.map_table.setRowCount(rows)
        self.map_table.setColumnCount(cols)
        
        # Set axis labels
        for c in range(cols):
            self.map_table.setHorizontalHeaderItem(c, QTableWidgetItem(f"{c}"))
        for r in range(rows):
            self.map_table.setVerticalHeaderItem(r, QTableWidgetItem(f"{r}"))
            
        # Fill data with value validation
        for r in range(rows):
            for c in range(cols):
                value = self.current_map[r, c]
                item = QTableWidgetItem(f"{value:.2f}")
                
                # Highlight out-of-range values
                if value < self.map_def.min_value or value > self.map_def.max_value:
                    item.setBackground(QColor(255, 200, 200))  # Light red
                    
                self.map_table.setItem(r, c, item)
                
    def update_visualization(self):
        """Update visualization with current map"""
        if self.current_map is None or self.map_def is None:
            return
            
        self.visualization.plot_map(
            self.current_map, 
            title=self.map_def.name,
            xlabel=self.map_def.axis_labels[0],
            ylabel=self.map_def.axis_labels[1],
            zlabel=self.map_def.units,
            contours=True
        )
        
    def write_map(self):
        """Write modified map to ECU with enhanced safety checks"""
        if not self.connection.connected:
            self.operation_status.emit(OperationStatus.FAILURE, "Not connected to vehicle")
            return
            
        if self.current_map is None or self.map_def is None:
            self.operation_status.emit(OperationStatus.FAILURE, "No map loaded")
            return
            
        # Validate changes
        valid, msg = self.safety.validate_write(
            self.current_map, self.original_map, self.map_def
        )
        if not valid:
            self.operation_status.emit(OperationStatus.SAFETY_BLOCK, msg)
            return
            
        # Create backup before writing
        backup_id = self.safety.create_backup(self.original_map, self.map_def.name)
        
        self.status_bar.showMessage(f"Writing {self.map_def.name} to ECU...")
        self.connection.write_map_async(self.current_map, self.map_def)
        
    def save_map(self):
        """Save current map to file in multiple formats"""
        if self.current_map is None:
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Map", "", 
            "MAP Files (*.map);;CSV Files (*.csv);;Binary Files (*.bin);;All Files (*)"
        )
        
        if file_path:
            try:
                if file_path.endswith('.csv'):
                    np.savetxt(file_path, self.current_map, delimiter=',', fmt='%.6f')
                elif file_path.endswith('.bin'):
                    self.current_map.tofile(file_path)
                else:  # Default to .map
                    np.savetxt(file_path, self.current_map, fmt='%.6f')
                    
                self.operation_status.emit(OperationStatus.SUCCESS, "Map saved successfully")
            except Exception as e:
                self.operation_status.emit(OperationStatus.FAILURE, f"Save failed: {str(e)}")
                
    def compare_map(self):
        """Compare current map with original"""
        if self.current_map is None or self.original_map is None:
            return
            
        # Create difference visualization
        diff = self.current_map - self.original_map
        self.visualization.plot_map(
            diff, 
            title=f"{self.map_def.name} - Changes",
            xlabel=self.map_def.axis_labels[0],
            ylabel=self.map_def.axis_labels[1],
            zlabel=f"Δ {self.map_def.units}",
            surface=True
        )
        
    def undo_changes(self):
        """Revert to original map"""
        if self.original_map is not None:
            self.current_map = self.original_map.copy()
            self.display_map(self.current_map)
            self.operation_status.emit(OperationStatus.SUCCESS, "Changes reverted")

class CANxTuneMainWindow(QMainWindow):
    """Professional-grade main application window"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CANxTune Professional - Vehicle Tuning Suite")
        self.setWindowIcon(QIcon(":/icons/app_icon.png"))
        self.resize(1400, 900)
        
        # Initialize core systems
        self.connection = ConnectionManager()
        self.safety = SafetySystem()
        self.vehicle_db = VehicleDatabase()
        self.flash_manager = FlashManager(self.connection, self.safety)
        
        # Setup UI
        self.init_ui()
        self.setup_connections()
        
        # Setup live data timer
        self.live_data_timer = QTimer()
        self.live_data_timer.timeout.connect(self.update_live_data)
        self.live_data_timer.start(1000)  # Update every second
        
        logger.info("Application initialized")
        
    def init_ui(self):
        """Initialize the professional UI"""
        # Create menu bar
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('File')
        
        connect_action = QAction(QIcon(":/icons/connect.png"), 'Connect to Vehicle', self)
        file_menu.addAction(connect_action)
        
        disconnect_action = QAction(QIcon(":/icons/disconnect.png"), 'Disconnect', self)
        file_menu.addAction(disconnect_action)
        
        file_menu.addSeparator()
        
        backup_action = QAction(QIcon(":/icons/backup.png"), 'Create Backup', self)
        file_menu.addAction(backup_action)
        
        restore_action = QAction(QIcon(":/icons/restore.png"), 'Restore Backup', self)
        file_menu.addAction(restore_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction(QIcon(":/icons/exit.png"), 'Exit', self)
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu('Tools')
        
        flash_action = QAction(QIcon(":/icons/flash.png"), 'Flash ECU', self)
        tools_menu.addAction(flash_action)
        
        compare_action = QAction(QIcon(":/icons/compare.png"), 'Compare Maps', self)
        tools_menu.addAction(compare_action)
        
        # View menu
        view_menu = menubar.addMenu('View')
        # Would contain options for different visualizations
        
        # Safety menu
        safety_menu = menubar.addMenu('Safety')
        
        protection_action = QAction('Toggle Write Protection', self)
        protection_action.setCheckable(True)
        protection_action.setChecked(True)
        safety_menu.addAction(protection_action)
        
        limits_action = QAction('Set Safety Limits...', self)
        safety_menu.addAction(limits_action)
        
        # Help menu
        help_menu = menubar.addMenu('Help')
        help_action = QAction('Documentation', self)
        help_menu.addAction(help_action)
        
        # Create central tab widget
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.West)
        
        # Create professional tuning modules
        self.ecu_tuner = ECUTuner(self.connection, self.safety, self.vehicle_db, ECUType.ENGINE)
        self.tcu_tuner = TCUTuner(self.connection, self.safety, self.vehicle_db, ECUType.TRANSMISSION)
        self.abs_tuner = ABSTuner(self.connection, self.safety, self.vehicle_db, ECUType.ABS)
        self.tcs_tuner = TCSTuner(self.connection, self.safety, self.vehicle_db, ECUType.TRACTION_CONTROL)
        
        # Add tabs with icons
        self.tabs.addTab(self.ecu_tuner, QIcon(":/icons/engine.png"), "Engine")
        self.tabs.addTab(self.tcu_tuner, QIcon(":/icons/transmission.png"), "Transmission")
        self.tabs.addTab(self.abs_tuner, QIcon(":/icons/abs.png"), "ABS")
        self.tabs.addTab(self.tcs_tuner, QIcon(":/icons/traction.png"), "Traction Control")
        
        # Create diagnostics tab
        self.diagnostics_tab = DiagnosticsWidget()
        self.tabs.addTab(self.diagnostics_tab, QIcon(":/icons/diagnostics.png"), "Diagnostics")
        
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
        
        # Operation status indicator
        self.status_indicator = QLabel()
        self.status_indicator.setMaximumWidth(20)
        self.status_bar.addPermanentWidget(self.status_indicator)
        
    def setup_connections(self):
        # Connect signals
        self.connection.connection_status_changed.connect(self.update_connection_status)
        self.connection.vehicle_identified.connect(self.set_vehicle)
        
        # Connect menu actions
        self.findChild(QAction, 'Connect to Vehicle').triggered.connect(self.connect_to_vehicle)
        self.findChild(QAction, 'Disconnect').triggered.connect(self.disconnect_from_vehicle)
        self.findChild(QAction, 'Flash ECU').triggered.connect(self.flash_ecu)
        self.findChild(QAction, 'Toggle Write Protection').triggered.connect(self.toggle_write_protection)
        
        # Connect module signals
        self.ecu_tuner.operation_status.connect(self.handle_operation_status)
        self.tcu_tuner.operation_status.connect(self.handle_operation_status)
        self.abs_tuner.operation_status.connect(self.handle_operation_status)
        self.tcs_tuner.operation_status.connect(self.handle_operation_status)
        
    def connect_to_vehicle(self):
        """Professional connection dialog"""
        dialog = ConnectionDialog(self.vehicle_db, self)
        if dialog.exec_() == QDialog.Accepted:
            params = dialog.get_connection_params()
            self.connection.connect_async(
                params["protocol"],
                params["vehicle"],
                params["interface"],
                params["channel"]
            )
            
    def set_vehicle(self, vehicle: VehicleDefinition):
        """Set current vehicle for all modules"""
        self.current_vehicle = vehicle
        self.ecu_tuner.set_vehicle(vehicle)
        self.tcu_tuner.set_vehicle(vehicle)
        self.abs_tuner.set_vehicle(vehicle)
        self.tcs_tuner.set_vehicle(vehicle)
        
        # Update status
        self.connection_status.setText(
            f"Connected: {vehicle.make} {vehicle.model} {vehicle.year}"
        )
        
    def update_connection_status(self, connected):
        """Update UI based on connection status"""
        if connected:
            self.connection_status.setText("Connected")
            self.connection_status.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status.setText("Disconnected")
            self.connection_status.setStyleSheet("color: red; font-weight: bold;")
            self.current_vehicle = None
            
    def flash_ecu(self):
        """Professional ECU flashing with progress tracking"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        # Select ECU to flash
        ecu_types = list(self.current_vehicle.ecus.keys())
        ecu_type, ok = QInputDialog.getItem(
            self, "Select ECU", "Choose ECU to flash:", [e.value for e in ecu_types], 0, False
        )
        
        if not ok or not ecu_type:
            return
            
        ecu_type = ECUType(ecu_type)
        ecu_def = self.current_vehicle.ecus[ecu_type]
        
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 100)
        
        # Read ECU image
        self.progress_bar.setValue(10)
        ecu_image = self.flash_manager.read_ecu_image(ecu_def)
        
        # Create backup
        self.progress_bar.setValue(30)
        backup_id = self.safety.create_backup(ecu_image, f"{ecu_type.value}_Backup")
        
        # Flash ECU
        self.progress_bar.setValue(50)
        success, msg = self.flash_manager.write_ecu_image(ecu_image, ecu_def)
        
        self.progress_bar.setValue(100)
        QTimer.singleShot(1000, lambda: self.progress_bar.setVisible(False))
        
        if success:
            QMessageBox.information(self, "Success", msg)
        else:
            QMessageBox.critical(self, "Error", msg)
            
    def handle_operation_status(self, status: OperationStatus, message: str):
        """Handle operation status from modules"""
        if status == OperationStatus.SUCCESS:
            self.status_indicator.setStyleSheet("background-color: green;")
            self.status_bar.showMessage(message, 3000)
        elif status == OperationStatus.FAILURE:
            self.status_indicator.setStyleSheet("background-color: red;")
            self.status_bar.showMessage(f"Error: {message}", 5000)
        elif status == OperationStatus.SAFETY_BLOCK:
            self.status_indicator.setStyleSheet("background-color: orange;")
            self.status_bar.showMessage(f"Safety: {message}", 5000)
            
        # Reset indicator after delay
        QTimer.singleShot(5000, lambda: self.status_indicator.setStyleSheet(""))

# =================
# APPLICATION START
# =================
if __name__ == "__main__":
    # Set up high DPI support for modern displays
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    
    app = QApplication(sys.argv)
    
    # Set professional application style
    app.setStyle("Fusion")
    
    # Set application font
    font = QFont("Segoe UI", 9)
    app.setFont(font)
    
    window = CANxTuneMainWindow()
    window.show()
    
    sys.exit(app.exec_())