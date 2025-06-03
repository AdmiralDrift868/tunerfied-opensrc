import sys
import os
import json
import uuid
import numpy as np
import can
import logging
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, field
from enum import Enum
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QComboBox, QTableWidget, QTableWidgetItem, QPushButton, 
                             QLabel, QLineEdit, QMessageBox, QFileDialog, QAction, QStatusBar,
                             QSplitter, QTreeWidget, QTreeWidgetItem, QHeaderView, QProgressBar,
                             QDialog, QFormLayout, QDialogButtonBox, QInputDialog, QStackedWidget)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject, QDateTime
from PyQt5.QtGui import QFont, QColor, QIcon, QPixmap
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import hashlib
import pyqtgraph as pg
from pyqtgraph import PlotWidget, ImageView
import time

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
    BODY_CONTROL = "BCM"
    INFOTAINMENT = "IVI"

class ProtocolType(Enum):
    OBD2 = "obd2"
    CAN = "can"
    J1939 = "j1939"
    UDS = "uds"
    KWP2000 = "kwp2000"

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
    editable: bool = True

@dataclass
class ECUDefinition:
    ecu_type: ECUType
    protocol: ProtocolType
    maps: Dict[str, MapDefinition] = field(default_factory=dict)
    base_address: int = 0x7E0
    response_id: int = 0x7E8
    security_level: int = 0
    flash_size: int = 1024 * 1024  # 1MB default
    supported_services: List[str] = field(default_factory=lambda: ["read", "write", "flash"])

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
    max_safety_deviation: float = 15.0  # Percentage

class ProtocolHandler(QObject):
    """Enhanced protocol handler with thread-safe operations and diagnostics"""
    connection_established = pyqtSignal(bool)
    data_received = pyqtSignal(dict)
    operation_completed = pyqtSignal(OperationStatus, str)
    diagnostic_message = pyqtSignal(str)
    progress_updated = pyqtSignal(int)
    
    def __init__(self):
        super().__init__()
        self.name = "Base Protocol"
        self.connected = False
        self.bus = None
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.diagnostic_buffer = []
        
    def connect_async(self, interface='socketcan', channel='can0'):
        """Establish connection to vehicle asynchronously"""
        self.diagnostic_message.emit(f"Starting {self.name} connection...")
        future = self.executor.submit(self._connect, interface, channel)
        future.add_done_callback(self._handle_connect_result)
        
    def _connect(self, interface, channel):
        try:
            self.diagnostic_message.emit(f"Connecting via {interface} on {channel}")
            self.bus = can.interface.Bus(channel=channel, bustype=interface)
            self._initialize_protocol()
            self.connected = True
            self.diagnostic_message.emit("Connection established")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}", exc_info=True)
            self.diagnostic_message.emit(f"Connection failed: {str(e)}")
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
        self.diagnostic_message.emit("Initializing protocol...")
        
    def disconnect(self):
        """Disconnect from vehicle"""
        if self.bus:
            self.bus.shutdown()
        self.connected = False
        self.bus = None
        self.diagnostic_message.emit("Disconnected from vehicle")
        logger.info("Disconnected from vehicle")
        
    def read_map_async(self, map_def: MapDefinition):
        """Read specific map from ECU asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit(f"Reading map: {map_def.name}")
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
            self.diagnostic_message.emit("Map read complete")
        except Exception as e:
            logger.error(f"Map read failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            self.diagnostic_message.emit(f"Map read failed: {str(e)}")
            
    def write_map_async(self, map_data: np.ndarray, map_def: MapDefinition):
        """Write map to ECU asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit(f"Writing map: {map_def.name}")
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
                self.diagnostic_message.emit("Map write successful")
            else:
                self.operation_completed.emit(OperationStatus.FAILURE, "Map write failed")
                logger.warning("Map write operation failed")
                self.diagnostic_message.emit("Map write failed")
        except Exception as e:
            logger.error(f"Map write failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            self.diagnostic_message.emit(f"Map write error: {str(e)}")
            
    def flash_ecu_async(self, image_data: bytes, ecu_def: ECUDefinition):
        """Flash complete ECU image asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit(f"Starting ECU flash for {ecu_def.ecu_type.value}")
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
                self.diagnostic_message.emit("ECU flash successful")
            else:
                self.operation_completed.emit(OperationStatus.FAILURE, "ECU flash failed")
                logger.error("ECU flash operation failed")
                self.diagnostic_message.emit("ECU flash failed")
        except Exception as e:
            logger.error(f"ECU flash failed: {e}", exc_info=True)
            self.operation_completed.emit(OperationStatus.FAILURE, str(e))
            self.diagnostic_message.emit(f"ECU flash error: {str(e)}")
            
    def read_live_data_async(self, pids: List[str]):
        """Read live data parameters asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit(f"Reading live data: {', '.join(pids)}")
        future = self.executor.submit(self._read_live_data, pids)
        future.add_done_callback(self._handle_live_data_result)
        
    def _read_live_data(self, pids: List[str]) -> Dict[str, Any]:
        """Protocol-specific live data reading"""
        raise NotImplementedError
        
    def _handle_live_data_result(self, future):
        try:
            live_data = future.result()
            self.data_received.emit({"type": "live_data", "data": live_data})
            self.diagnostic_message.emit("Live data received")
        except Exception as e:
            logger.error(f"Live data read failed: {e}", exc_info=True)
            self.diagnostic_message.emit(f"Live data error: {str(e)}")
            
    def read_dtcs_async(self):
        """Read Diagnostic Trouble Codes asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit("Reading DTCs...")
        future = self.executor.submit(self._read_dtcs)
        future.add_done_callback(self._handle_dtc_result)
        
    def _read_dtcs(self) -> List[str]:
        """Protocol-specific DTC reading"""
        raise NotImplementedError
        
    def _handle_dtc_result(self, future):
        try:
            dtcs = future.result()
            self.data_received.emit({"type": "dtcs", "data": dtcs})
            self.diagnostic_message.emit(f"Read {len(dtcs)} DTCs")
        except Exception as e:
            logger.error(f"DTC read failed: {e}", exc_info=True)
            self.diagnostic_message.emit(f"DTC read error: {str(e)}")
            
    def clear_dtcs_async(self):
        """Clear Diagnostic Trouble Codes asynchronously"""
        if not self.connected:
            raise ConnectionError("Not connected to vehicle")
            
        self.diagnostic_message.emit("Clearing DTCs...")
        future = self.executor.submit(self._clear_dtcs)
        future.add_done_callback(self._handle_clear_dtc_result)
        
    def _clear_dtcs(self) -> bool:
        """Protocol-specific DTC clearing"""
        raise NotImplementedError
        
    def _handle_clear_dtc_result(self, future):
        try:
            success = future.result()
            if success:
                self.operation_completed.emit(OperationStatus.SUCCESS, "DTCs cleared")
                self.diagnostic_message.emit("DTCs cleared successfully")
            else:
                self.operation_completed.emit(OperationStatus.FAILURE, "DTC clear failed")
                self.diagnostic_message.emit("DTC clear failed")
        except Exception as e:
            logger.error(f"DTC clear failed: {e}", exc_info=True)
            self.diagnostic_message.emit(f"DTC clear error: {str(e)}")

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
        self.diagnostic_message.emit("Sent CAN initialization sequence")
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
        
        # Simulate transmission delay with progress updates
        total_steps = rows * cols
        for i in range(total_steps):
            if i % 10 == 0:
                progress = int((i / total_steps) * 100)
                self.progress_updated.emit(progress)
            time.sleep(0.001)
        
        self.progress_updated.emit(100)
        return map_data
        
    def _write_map(self, map_data: np.ndarray, map_def: MapDefinition) -> bool:
        logger.info(f"Writing map: {map_def.name}")
        
        # Simulate write operation with verification
        rows, cols = map_data.shape
        if map_data.shape != map_def.size:
            logger.error(f"Map size mismatch: expected {map_def.size}, got {map_data.shape}")
            return False
            
        # Simulate write process with progress
        total_steps = rows
        for i in range(rows):
            if i % 2 == 0:
                progress = int((i / total_steps) * 100)
                self.progress_updated.emit(progress)
            time.sleep(0.05)
            
        # Verify write
        read_back = self._read_map(map_def)
        if not np.allclose(map_data, read_back, atol=0.1):
            logger.error("Write verification failed")
            return False
            
        self.progress_updated.emit(100)
        return True
        
    def _read_live_data(self, pids: List[str]) -> Dict[str, Any]:
        # Simulate live data reading
        live_data = {}
        for pid in pids:
            if pid == "RPM":
                live_data[pid] = np.random.randint(800, 7000)
            elif pid == "SPEED":
                live_data[pid] = np.random.randint(0, 180)
            elif pid == "THROTTLE":
                live_data[pid] = np.random.uniform(0, 100)
            elif pid == "ENGINE_LOAD":
                live_data[pid] = np.random.uniform(0, 100)
            elif pid == "COOLANT_TEMP":
                live_data[pid] = np.random.uniform(70, 110)
            else:
                live_data[pid] = 0
                
        return live_data
        
    def _read_dtcs(self) -> List[str]:
        # Simulate DTC reading
        dtcs = []
        codes = ['P0300', 'P0171', 'U0100', 'C1234']
        for _ in range(np.random.randint(0, 3)):
            dtcs.append(np.random.choice(codes))
        return dtcs
        
    def _clear_dtcs(self) -> bool:
        # Simulate DTC clearing
        time.sleep(1)
        return True

class OBD2Protocol(ProtocolHandler):
    """OBD2 Protocol Implementation"""
    def __init__(self):
        super().__init__()
        self.name = ProtocolType.OBD2.value
        logger.info("Initializing OBD2 protocol handler")
        
    def _initialize_protocol(self):
        self.diagnostic_message.emit("OBD2 protocol initialized")
        
    def _read_map(self, map_def: MapDefinition) -> np.ndarray:
        # OBD2 doesn't typically support direct map access
        raise NotImplementedError("OBD2 protocol does not support direct map access")
        
    def _read_live_data(self, pids: List[str]) -> Dict[str, Any]:
        # Simulate OBD2 live data reading
        live_data = {}
        for pid in pids:
            if pid == "RPM":
                live_data[pid] = np.random.randint(800, 7000)
            elif pid == "SPEED":
                live_data[pid] = np.random.randint(0, 180)
            elif pid == "MAF":
                live_data[pid] = np.random.uniform(1, 250)
            elif pid == "FUEL_PRESSURE":
                live_data[pid] = np.random.uniform(0, 100)
            elif pid == "INTAKE_TEMP":
                live_data[pid] = np.random.uniform(10, 50)
            else:
                live_data[pid] = 0
                
        return live_data
        
    def _read_dtcs(self) -> List[str]:
        # Simulate OBD2 DTC reading
        dtcs = []
        codes = ['P0101', 'P0420', 'B1234', 'C0123']
        for _ in range(np.random.randint(0, 4)):
            dtcs.append(np.random.choice(codes))
        return dtcs
        
    def _clear_dtcs(self) -> bool:
        # Simulate OBD2 DTC clearing
        time.sleep(1)
        return True

class UDSProtocol(ProtocolHandler):
    """UDS Protocol Implementation"""
    def __init__(self):
        super().__init__()
        self.name = ProtocolType.UDS.value
        logger.info("Initializing UDS protocol handler")
        
    def _initialize_protocol(self):
        self.diagnostic_message.emit("UDS protocol initialized")
        
    def _read_map(self, map_def: MapDefinition) -> np.ndarray:
        logger.info(f"Reading UDS map: {map_def.name}")
        rows, cols = map_def.size
        
        # Simulate UDS map reading
        map_data = np.zeros((rows, cols), dtype=map_def.data_type)
        
        # Generate realistic map data
        if "Torque" in map_def.name:
            base = np.linspace(50, 300, cols)
            map_data = np.array([base * (0.8 + i/20) for i in range(rows)])
        elif "Injection" in map_def.name:
            base = np.linspace(1, 10, cols)
            map_data = np.array([base * (1.2 - i/30) for i in range(rows)])
        
        # Apply scaling and offset
        map_data = (map_data * map_def.scaling_factor) + map_def.offset
        
        # Simulate transmission delay
        total_steps = rows * cols
        for i in range(total_steps):
            if i % 10 == 0:
                progress = int((i / total_steps) * 100)
                self.progress_updated.emit(progress)
            time.sleep(0.001)
        
        self.progress_updated.emit(100)
        return map_data
        
    def _write_map(self, map_data: np.ndarray, map_def: MapDefinition) -> bool:
        logger.info(f"Writing UDS map: {map_def.name}")
        
        # Simulate UDS write operation
        rows, cols = map_data.shape
        if map_data.shape != map_def.size:
            logger.error(f"Map size mismatch: expected {map_def.size}, got {map_data.shape}")
            return False
            
        # Simulate security access
        self.diagnostic_message.emit("Requesting security access...")
        time.sleep(1)
        
        # Simulate write process
        total_steps = rows
        for i in range(rows):
            if i % 2 == 0:
                progress = int((i / total_steps) * 100)
                self.progress_updated.emit(progress)
            time.sleep(0.1)
            
        # Verify write
        read_back = self._read_map(map_def)
        if not np.allclose(map_data, read_back, atol=0.1):
            logger.error("Write verification failed")
            return False
            
        self.progress_updated.emit(100)
        return True
        
    def _flash_ecu(self, image_data: bytes, ecu_def: ECUDefinition) -> bool:
        logger.info(f"Flashing ECU via UDS: {ecu_def.ecu_type.value}")
        
        # Simulate flash process
        total_size = len(image_data)
        chunk_size = 1024
        chunks = total_size // chunk_size
        
        # Request programming session
        self.diagnostic_message.emit("Entering programming session...")
        time.sleep(1)
        
        # Erase memory
        self.diagnostic_message.emit("Erasing ECU memory...")
        time.sleep(2)
        
        # Transfer data
        for i in range(chunks):
            progress = int((i / chunks) * 100)
            self.progress_updated.emit(progress)
            time.sleep(0.01)
            
        # Verify checksum
        self.diagnostic_message.emit("Verifying checksum...")
        time.sleep(1)
        
        # Exit programming session
        self.diagnostic_message.emit("Exiting programming session...")
        time.sleep(0.5)
        
        self.progress_updated.emit(100)
        return True

class ConnectionManager(QObject):
    """Enhanced vehicle connection manager with diagnostics"""
    connection_status_changed = pyqtSignal(bool)
    vehicle_identified = pyqtSignal(VehicleDefinition)
    diagnostic_message = pyqtSignal(str)
    progress_updated = pyqtSignal(int)
    
    def __init__(self):
        super().__init__()
        self.protocol_handler = None
        self.connected = False
        self.current_vehicle = None
        self.protocols = {
            ProtocolType.CAN: CANProtocol,
            ProtocolType.OBD2: OBD2Protocol,
            ProtocolType.UDS: UDSProtocol,
            # Other protocols would be added here
        }
        logger.info("Connection manager initialized")
        
    def connect_async(self, protocol: ProtocolType, vehicle: VehicleDefinition, 
                     interface='socketcan', channel='can0'):
        """Connect to vehicle bus asynchronously"""
        if self.connected:
            self.disconnect()
            
        if protocol not in self.protocols:
            logger.error(f"Unsupported protocol: {protocol.value}")
            self.diagnostic_message.emit(f"Unsupported protocol: {protocol.value}")
            return
            
        self.protocol_handler = self.protocols[protocol]()
        
        # Connect signals
        self.protocol_handler.connection_established.connect(self._handle_connection_result)
        self.protocol_handler.diagnostic_message.connect(self.diagnostic_message)
        self.protocol_handler.progress_updated.connect(self.progress_updated)
        
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
        
    def read_dtcs_async(self):
        """Read Diagnostic Trouble Codes asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.read_dtcs_async()
        
    def clear_dtcs_async(self):
        """Clear Diagnostic Trouble Codes asynchronously"""
        if not self.connected or not self.protocol_handler:
            raise ConnectionError("Not connected to vehicle")
        self.protocol_handler.clear_dtcs_async()

class SafetySystem:
    """Enhanced safety and validation system with cryptographic signing"""
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
            
        if not map_def.editable:
            return False, "Map is not editable"
            
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
        """Enhanced flash validation with cryptographic verification"""
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
        """Create safety backup with versioning and metadata"""
        backup_id = str(uuid.uuid4())
        timestamp = QDateTime.currentDateTime().toString(Qt.ISODate)
        
        self.backups[backup_id] = {
            'id': backup_id,
            'data': data,
            'identifier': identifier,
            'timestamp': timestamp,
            'checksum': self._calculate_checksum(data),
            'version': 1,
            'metadata': {
                'vehicle': None,
                'ecu_type': None,
                'map_name': None
            }
        }
        self.logger.info(f"Created backup {backup_id} for {identifier}")
        return backup_id
        
    def restore_backup(self, backup_id: str) -> Any:
        """Restore from backup with validation and verification"""
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
        """Verify digital signature (stub implementation)"""
        # In real implementation, would use cryptographic signature verification
        # This is a simplified simulation
        return image_data.endswith(b'SIG')
        
    def _verify_checksum(self, image_data: bytes) -> bool:
        """Verify checksum (stub implementation)"""
        # In real implementation, would validate against ECU-specific checksum
        # This is a simplified simulation
        return True

class VehicleDatabase:
    """Enhanced vehicle definition database with versioning and persistence"""
    def __init__(self):
        self.db = {}
        self.version = "1.1.0"
        self.db_file = "vehicle_db.json"
        self.load_database()
        logger.info(f"Vehicle database initialized (version {self.version})")
        
    def load_database(self):
        """Load vehicle definitions from JSON file"""
        try:
            if os.path.exists(self.db_file):
                with open(self.db_file, 'r') as f:
                    data = json.load(f)
                    self.db = self._deserialize(data)
            else:
                self._create_sample_data()
                self.save_database()
        except Exception as e:
            logger.error(f"Error loading database: {e}", exc_info=True)
            self.db = {}
            
    def save_database(self):
        """Save database to file"""
        try:
            with open(self.db_file, 'w') as f:
                json.dump(self._serialize(), f, indent=2)
        except Exception as e:
            logger.error(f"Error saving database: {e}", exc_info=True)
            
    def _create_sample_data(self):
        """Create sample vehicle data"""
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
                            "Ignition Map": MapDefinition(
                                name="Ignition Map",
                                address=0x7F0,
                                size=(20, 20),
                                units="° BTDC",
                                min_value=5,
                                max_value=45
                            ),
                        }
                    ),
                    ECUType.TRANSMISSION: ECUDefinition(
                        ecu_type=ECUType.TRANSMISSION,
                        protocol=ProtocolType.CAN,
                        maps={
                            "Shift Map": MapDefinition(
                                name="Shift Map",
                                address=0x7A0,
                                size=(10, 10),
                                units="RPM",
                                min_value=1500,
                                max_value=7000
                            )
                        }
                    ),
                },
                supported_protocols=[ProtocolType.CAN],
                requires_signature=True
            ),
            "bmw_m3_2022": VehicleDefinition(
                make="BMW",
                model="M3",
                year=2022,
                vin_pattern="WBS*",
                ecus={
                    ECUType.ENGINE: ECUDefinition(
                        ecu_type=ECUType.ENGINE,
                        protocol=ProtocolType.UDS,
                        maps={
                            "Torque Map": MapDefinition(
                                name="Torque Map",
                                address=0x1000,
                                size=(30, 30),
                                units="Nm",
                                min_value=50,
                                max_value=500
                            ),
                            "Boost Map": MapDefinition(
                                name="Boost Map",
                                address=0x1100,
                                size=(20, 20),
                                units="kPa",
                                min_value=90,
                                max_value=250
                            ),
                        },
                        flash_size=2 * 1024 * 1024  # 2MB
                    ),
                },
                supported_protocols=[ProtocolType.UDS, ProtocolType.CAN],
                requires_signature=True
            ),
        }
        
    def _serialize(self):
        """Serialize database to JSON-serializable format"""
        serialized = {}
        for key, vehicle in self.db.items():
            # Convert enums to their values for serialization
            ecus = {}
            for ecu_type, ecu_def in vehicle.ecus.items():
                ecu_data = ecu_def.__dict__.copy()
                ecu_data['ecu_type'] = ecu_type.value
                ecu_data['protocol'] = ecu_data['protocol'].value
                
                # Convert maps
                maps = {}
                for map_name, map_def in ecu_def.maps.items():
                    map_data = map_def.__dict__.copy()
                    maps[map_name] = map_data
                ecu_data['maps'] = maps
                
                ecus[ecu_type.value] = ecu_data
                
            vehicle_data = vehicle.__dict__.copy()
            vehicle_data['ecus'] = ecus
            vehicle_data['supported_protocols'] = [p.value for p in vehicle.supported_protocols]
            serialized[key] = vehicle_data
            
        return serialized
        
    def _deserialize(self, data):
        """Deserialize data from JSON format"""
        db = {}
        for key, vehicle_data in data.items():
            # Convert protocols back to enums
            protocols = [ProtocolType(p) for p in vehicle_data['supported_protocols']]
            
            # Convert ECUs
            ecus = {}
            for ecu_type_str, ecu_data in vehicle_data['ecus'].items():
                ecu_type = ECUType(ecu_type_str)
                protocol = ProtocolType(ecu_data['protocol'])
                
                # Convert maps
                maps = {}
                for map_name, map_data in ecu_data['maps'].items():
                    map_def = MapDefinition(**map_data)
                    maps[map_name] = map_def
                    
                ecu_def = ECUDefinition(
                    ecu_type=ecu_type,
                    protocol=protocol,
                    maps=maps,
                    base_address=ecu_data['base_address'],
                    response_id=ecu_data['response_id'],
                    security_level=ecu_data['security_level'],
                    flash_size=ecu_data['flash_size']
                )
                ecus[ecu_type] = ecu_def
                
            vehicle = VehicleDefinition(
                make=vehicle_data['make'],
                model=vehicle_data['model'],
                year=vehicle_data['year'],
                vin_pattern=vehicle_data['vin_pattern'],
                ecus=ecus,
                supported_protocols=protocols,
                validation_key=vehicle_data.get('validation_key'),
                requires_signature=vehicle_data['requires_signature']
            )
            db[key] = vehicle
            
        return db
        
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
        
    def add_vehicle(self, vehicle: VehicleDefinition):
        """Add a new vehicle to the database"""
        key = f"{vehicle.make.lower()}_{vehicle.model.lower()}_{vehicle.year}"
        self.db[key] = vehicle
        self.save_database()
        
    def remove_vehicle(self, make: str, model: str, year: int):
        """Remove a vehicle from the database"""
        key = f"{make.lower()}_{model.lower()}_{year}"
        if key in self.db:
            del self.db[key]
            self.save_database()

class FlashManager:
    """Professional-grade ECU flashing manager with progress tracking"""
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
        if ecu_def.security_level > 0:
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
            # Create backup before flashing
            backup_id = self.safety.create_backup(image_data, f"{ecu_def.ecu_type.value}_FlashBackup")
            
            # Start flash operation
            self.connection.flash_ecu_async(image_data, ecu_def)
            
            return True, "Flash started successfully"
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
    """Professional connection setup dialog with advanced options"""
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
        
        # Advanced options button
        self.advanced_btn = QPushButton("Advanced Options...")
        self.advanced_btn.clicked.connect(self.show_advanced_options)
        layout.addRow(self.advanced_btn)
        
        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        self.setLayout(layout)
        self.update_vehicle_info(0)
        
    def update_vehicle_info(self, index):
        vehicle_name = self.vehicle_combo.currentText()
        parts = vehicle_name.split('_')
        if len(parts) < 3:
            return
            
        make, model, year = parts[0], parts[1], int(parts[2])
        vehicle = self.vehicle_db.get_vehicle(make, model, year)
        
        if vehicle:
            self.selected_vehicle = vehicle
            info_text = (f"{vehicle.make} {vehicle.model} {vehicle.year}\n"
                         f"ECUs: {', '.join([e.value for e in vehicle.ecus.keys()])}\n"
                         f"Protocols: {', '.join([p.value for p in vehicle.supported_protocols])}")
            self.vehicle_info.setText(info_text)
            
            # Update protocols
            self.protocol_combo.clear()
            self.protocol_combo.addItems([p.value for p in vehicle.supported_protocols])
            
    def show_advanced_options(self):
        """Show advanced connection options"""
        dialog = QDialog(self)
        dialog.setWindowTitle("Advanced Connection Options")
        layout = QFormLayout()
        
        # Baud rate selection
        baud_combo = QComboBox()
        baud_combo.addItems(["500000", "250000", "125000", "1000000"])
        layout.addRow("CAN Baud Rate:", baud_combo)
        
        # Termination resistor
        term_check = QCheckBox("Enable Termination Resistor")
        term_check.setChecked(True)
        layout.addRow(term_check)
        
        # Sample point
        sample_spin = QDoubleSpinBox()
        sample_spin.setRange(0.5, 0.9)
        sample_spin.setValue(0.75)
        sample_spin.setSingleStep(0.05)
        layout.addRow("Sample Point:", sample_spin)
        
        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(dialog.accept)
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)
        
        dialog.setLayout(layout)
        dialog.exec_()
            
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
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.ax = None
        
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
    map_data_modified = pyqtSignal()
    
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
                
    def map_cell_changed(self, row, column):
        """Handle cell edits in the table"""
        if self.current_map is None or self.map_def is None:
            return
            
        item = self.map_table.item(row, column)
        if item:
            try:
                new_value = float(item.text())
                self.current_map[row, column] = new_value
                self.map_data_modified.emit()
                
                # Update visualization
                self.update_visualization()
            except ValueError:
                # Revert to previous value
                item.setText(f"{self.current_map[row, column]:.2f}")
                
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

class ECUTuner(BaseTunerModule):
    """Specialized Engine Control Unit tuning module"""
    def __init__(self, connection: ConnectionManager, safety: SafetySystem, 
                 vehicle_db: VehicleDatabase, parent=None):
        super().__init__(connection, safety, vehicle_db, ECUType.ENGINE, parent)

class TCUTuner(BaseTunerModule):
    """Specialized Transmission Control Unit tuning module"""
    def __init__(self, connection: ConnectionManager, safety: SafetySystem, 
                 vehicle_db: VehicleDatabase, parent=None):
        super().__init__(connection, safety, vehicle_db, ECUType.TRANSMISSION, parent)

class DiagnosticsWidget(QWidget):
    """Professional diagnostics module with real-time monitoring"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # DTC section
        dtc_group = QGroupBox("Diagnostic Trouble Codes")
        dtc_layout = QVBoxLayout()
        
        self.dtc_list = QTreeWidget()
        self.dtc_list.setHeaderLabels(["Code", "Description", "Status"])
        dtc_layout.addWidget(self.dtc_list)
        
        dtc_btn_layout = QHBoxLayout()
        self.read_dtc_btn = QPushButton("Read DTCs")
        self.clear_dtc_btn = QPushButton("Clear DTCs")
        dtc_btn_layout.addWidget(self.read_dtc_btn)
        dtc_btn_layout.addWidget(self.clear_dtc_btn)
        dtc_layout.addLayout(dtc_btn_layout)
        
        dtc_group.setLayout(dtc_layout)
        layout.addWidget(dtc_group)
        
        # Live data section
        live_group = QGroupBox("Live Data")
        live_layout = QVBoxLayout()
        
        self.live_data_table = QTableWidget(0, 2)
        self.live_data_table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.live_data_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.live_data_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.live_data_table.setEditTriggers(QTableWidget.NoEditTriggers)
        live_layout.addWidget(self.live_data_table)
        
        live_group.setLayout(live_layout)
        layout.addWidget(live_group)
        
        # Diagnostic messages
        diag_group = QGroupBox("Diagnostic Messages")
        diag_layout = QVBoxLayout()
        
        self.diag_text = QTextEdit()
        self.diag_text.setReadOnly(True)
        diag_layout.addWidget(self.diag_text)
        
        diag_group.setLayout(diag_layout)
        layout.addWidget(diag_group)
        
        self.setLayout(layout)
        
    def update_live_data(self, data: Dict[str, Any]):
        """Update live data display"""
        self.live_data_table.setRowCount(len(data))
        
        for i, (pid, value) in enumerate(data.items()):
            self.live_data_table.setItem(i, 0, QTableWidgetItem(pid))
            self.live_data_table.setItem(i, 1, QTableWidgetItem(str(value)))
            
    def update_dtcs(self, dtcs: List[str]):
        """Update DTC list"""
        self.dtc_list.clear()
        
        for dtc in dtcs:
            item = QTreeWidgetItem(self.dtc_list)
            item.setText(0, dtc)
            item.setText(1, self.get_dtc_description(dtc))
            item.setText(2, "Active")
            
    def get_dtc_description(self, dtc: str) -> str:
        """Get description for DTC code"""
        # In real implementation, this would come from a database
        descriptions = {
            "P0300": "Random/Multiple Cylinder Misfire Detected",
            "P0171": "System Too Lean (Bank 1)",
            "U0100": "Lost Communication with ECM/PCM",
            "C1234": "Wheel Speed Sensor Circuit Open"
        }
        return descriptions.get(dtc, "Unknown DTC")
        
    def add_diagnostic_message(self, message: str):
        """Add a diagnostic message to the log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")
        self.diag_text.append(f"[{timestamp}] {message}")
        self.diag_text.verticalScrollBar().setValue(
            self.diag_text.verticalScrollBar().maximum()
        )

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
        connect_action.setShortcut("Ctrl+C")
        file_menu.addAction(connect_action)
        
        disconnect_action = QAction(QIcon(":/icons/disconnect.png"), 'Disconnect', self)
        disconnect_action.setShortcut("Ctrl+D")
        file_menu.addAction(disconnect_action)
        
        file_menu.addSeparator()
        
        backup_action = QAction(QIcon(":/icons/backup.png"), 'Create Backup', self)
        backup_action.setShortcut("Ctrl+B")
        file_menu.addAction(backup_action)
        
        restore_action = QAction(QIcon(":/icons/restore.png"), 'Restore Backup', self)
        restore_action.setShortcut("Ctrl+R")
        file_menu.addAction(restore_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction(QIcon(":/icons/exit.png"), 'Exit', self)
        exit_action.setShortcut("Ctrl+Q")
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu('Tools')
        
        flash_action = QAction(QIcon(":/icons/flash.png"), 'Flash ECU', self)
        flash_action.setShortcut("Ctrl+F")
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
        protection_action.setShortcut("Ctrl+P")
        safety_menu.addAction(protection_action)
        
        limits_action = QAction('Set Safety Limits...', self)
        safety_menu.addAction(limits_action)
        
        # Help menu
        help_menu = menubar.addMenu('Help')
        help_action = QAction('Documentation', self)
        help_menu.addAction(help_action)
        
        about_action = QAction('About CANxTune', self)
        help_menu.addAction(about_action)
        
        # Create central tab widget
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.West)
        
        # Create professional tuning modules
        self.ecu_tuner = ECUTuner(self.connection, self.safety, self.vehicle_db)
        self.tcu_tuner = TCUTuner(self.connection, self.safety, self.vehicle_db)
        self.abs_tuner = BaseTunerModule(self.connection, self.safety, self.vehicle_db, ECUType.ABS)
        self.tcs_tuner = BaseTunerModule(self.connection, self.safety, self.vehicle_db, ECUType.TRACTION_CONTROL)
        
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
        self.connection.diagnostic_message.connect(self.diagnostics_tab.add_diagnostic_message)
        self.connection.progress_updated.connect(self.update_progress)
        
        # Connect menu actions
        connect_action = self.findChild(QAction, 'Connect to Vehicle')
        connect_action.triggered.connect(self.connect_to_vehicle)
        
        disconnect_action = self.findChild(QAction, 'Disconnect')
        disconnect_action.triggered.connect(self.disconnect_from_vehicle)
        
        flash_action = self.findChild(QAction, 'Flash ECU')
        flash_action.triggered.connect(self.flash_ecu)
        
        protection_action = self.findChild(QAction, 'Toggle Write Protection')
        protection_action.triggered.connect(self.toggle_write_protection)
        
        about_action = self.findChild(QAction, 'About CANxTune')
        about_action.triggered.connect(self.show_about)
        
        # Connect module signals
        self.ecu_tuner.operation_status.connect(self.handle_operation_status)
        self.tcu_tuner.operation_status.connect(self.handle_operation_status)
        self.abs_tuner.operation_status.connect(self.handle_operation_status)
        self.tcs_tuner.operation_status.connect(self.handle_operation_status)
        
        # Connect diagnostics signals
        self.diagnostics_tab.read_dtc_btn.clicked.connect(self.read_dtcs)
        self.diagnostics_tab.clear_dtc_btn.clicked.connect(self.clear_dtcs)
        
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
            
    def disconnect_from_vehicle(self):
        """Disconnect from vehicle"""
        self.connection.disconnect()
            
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
            
    def update_live_data(self):
        """Update live data display"""
        if not self.connection.connected:
            return
            
        # Read standard set of PIDs
        pids = ["RPM", "SPEED", "THROTTLE", "ENGINE_LOAD", "COOLANT_TEMP"]
        self.connection.read_live_data_async(pids)
        
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
            
    def read_dtcs(self):
        """Read Diagnostic Trouble Codes"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        self.connection.read_dtcs_async()
        
    def clear_dtcs(self):
        """Clear Diagnostic Trouble Codes"""
        if not self.connection.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first")
            return
            
        reply = QMessageBox.question(self, "Confirm Clear", 
                                    "Are you sure you want to clear all Diagnostic Trouble Codes?",
                                    QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.connection.clear_dtcs_async()
            
    def toggle_write_protection(self, enabled):
        """Toggle write protection state"""
        self.safety.write_protection = enabled
        status = "ON" if enabled else "OFF"
        color = "green" if enabled else "red"
        self.safety_status.setText(f"Write Protection: {status}")
        self.safety_status.setStyleSheet(f"color: {color};")
        
    def update_progress(self, value):
        """Update progress bar"""
        self.progress_bar.setValue(value)
        self.progress_bar.setVisible(value < 100)
        
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
        
    def show_about(self):
        """Show about dialog"""
        about = QMessageBox(self)
        about.setWindowTitle("About CANxTune Professional")
        about.setText(
            "<h2>CANxTune Professional</h2>"
            "<p>Version 2.0</p>"
            "<p>Advanced Vehicle Tuning Suite</p>"
            "<p>&copy; 2023 CANxTune Development Team</p>"
        )
        about.setIconPixmap(QPixmap(":/icons/app_icon.png").scaled(64, 64))
        about.exec_()

# =================
# APPLICATION START
# =================
if __name__ == "__main__":
    # Set up high DPI support for modern displays
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    
    # Set professional application style
    app.setStyle("Fusion")
    
    # Set application font
    font = QFont("Segoe UI", 9)
    app.setFont(font)
    
    # Create splash screen
    splash_pix = QPixmap(":/splash.png")
    splash = QSplashScreen(splash_pix)
    splash.show()
    app.processEvents()
    
    window = CANxTuneMainWindow()
    window.show()
    
    splash.finish(window)
    
    sys.exit(app.exec_())