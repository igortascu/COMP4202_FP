-------- 4202 Final Project - Ambulance Response Time Evaluation --------

Author: Igor Tascu 101181093

Folders & Files:
arcGisData:
 - Holds arcGIS project folder

Scripts/
 - algorithms.py: Contains the pathfinding algorithms
 - dataProcessing.py: Process OSMX data along with other needed data for our algorithms
 - evaluation.py: Evaluates our algorithms and concludes comparisons

Data/
 - holds all the data required and used for visualization by ArcGIS like roads etc.
 - nodes.shp: road intersections or stops
 - edges.shp: represent all roads
 - ambulance.csv: ambulance data
 - incident.csv: incident data

Output/
 - holds output files

Libraries Used:
 - Python OSMNX
 - Pandas for data plotting
 - geopandas
 - network
