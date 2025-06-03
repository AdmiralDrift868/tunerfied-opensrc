# core/vehicle_db.py
import json
import os

class VehicleDatabase:
    def __init__(self):
        self.db = {}
        self.load_database()
        
    def load_database(self):
        """Load vehicle definitions"""
        try:
            with open('vehicles.json') as f:
                self.db = json.load(f)
        except FileNotFoundError:
            self.db = {}
            
    def get_vehicle(self, make, model, year):
        """Get vehicle definition"""
        key = f"{make}_{model}_{year}"
        return self.db.get(key)
        
    def get_map_definitions(self, make, model, year):
        """Get map locations for vehicle"""
        vehicle = self.get_vehicle(make, model, year)
        if vehicle:
            return vehicle.get('maps', {})
        return {}