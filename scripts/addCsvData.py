import os
import pandas as pd

# Ensure 'data/' directory exists
os.makedirs("data", exist_ok=True)

ambulances_data = [
    {"id": 1, "name": "Ambulance A", "lat": 45.3905, "lon": -75.7221, "status": "available"},  # ~4.2 km
    {"id": 2, "name": "Ambulance B", "lat": 45.4370, "lon": -75.6705, "status": "available"},  # ~2.7 km
    {"id": 3, "name": "Ambulance C", "lat": 45.4482, "lon": -75.7150, "status": "busy"},       # ~3.3 km
    {"id": 4, "name": "Ambulance D", "lat": 45.4102, "lon": -75.6848, "status": "available"}   # ~1.7 km 
]

incidents_data = [
    {"id": 101, "type": "Critical", "lat": 45.4235, "lon": -75.6950, "severity": 3},
    {"id": 102, "type": "Moderate", "lat": 45.4200, "lon": -75.7000, "severity": 2},
    {"id": 103, "type": "Minor", "lat": 45.4195, "lon": -75.6930, "severity": 1}
]

# Save to CSV files
ambulance_df = pd.DataFrame(ambulances_data)
incident_df = pd.DataFrame(incidents_data)

ambulance_df.to_csv("data/ambulances.csv", index=False)
incident_df.to_csv("data/incidents.csv", index=False)

ambulance_df, incident_df


