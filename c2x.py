controlFile = '/Users/paolo/temp/San Diego AMS/TCA-A-2.3.2/TCAinput.xml'

#standard
import time
import sys
if not '/Library/Python/2.7/site-packages' in sys.path:
    sys.path.append('/Library/Python/2.7/site-packages')
from datetime import datetime as dt

#external
from AAPI import * # Aimsun API
import pandas as pd

#TCA
from TCARandom import Random_generator
from TCALoadControl import ControlFiles
from TCAAlgorithm import TCA_Algorithm
from TCAFileReader import Trajectories
from TCACore import Timer, logger, report_errors, clear_old_files, write_default, write_default_regions, control_values, strategy_values
from TCARegions import Load_Regions
from TCASpacePartitioning import Location_Tree
from TCANetData import CLNetData

#============================================================================

def AAPILoad():
    return 0

def AAPIInit():

    global enabled
    global program_st
    global CF
    global changecolor
    global Regions
    global RSE_Tree
    global Algorithm

    AKIPrintString('Initializing TCA API using Control File %s...' % controlFile)
    
    CF = ControlFiles(controlFile, TCA_version = 'aimsun')
    CF.Load_files()

    if CF.Error_count() == 0:
        enabled = True
        program_st = time.time()
        
        changecolor = CF.Control['ColorDisplay']
        if changecolor == 'False':
            changecolor = 0

        CF.Control['AccelColumn'] = True

        if CF.Control['RegionsFile'] is not None:
            unit_conversion = 100 / 2.54 / 12
            Regions = Load_Regions(CF.Control['RegionsFile'],CF.Control['Seed'],unit_conversion)
            AKIPrintString("Loading Regions File %s" % CF.Control['RegionsFile'])
        else:
            Regions = None

        if CF.Control["RSELocationFile"] is not None:
            unit_conversion = 100 / 2.54 / 12
            RSEs = CLNetData(unit_conversion)
            errors = RSEs.RSELoad(CF.Control["RSELocationFile"], CF)
            report_errors(errors)
            RSE_Tree = Location_Tree(RSEs.RSEList, CF.Strategy["MinRSERange"])
        else:
            RSEs = None
            RSE_Tree = None

        if CF.Control["SPOTLocationFile"] is not None:
            SPOTdevices = CLNetData(unit_conversion)
            errors = SPOTdevices.SPOTLoad(CF.Control["SPOTLocationFile"], CF)
            report_errors(errors)
            SPOT_Tree =Location_Tree(SPOTdevices.SPOTList, CF.Strategy["SPOTdeviceRange"])
        else:
            SPOTdevices = None
            SPOT_Tree = None

        clear_old_files(CF)
        Algorithm = TCA_Algorithm(CF, RSEs, RSE_Tree, CF.Control['Seed'], Regions, SPOT_Tree)
    
    else:
        enabled = False
        logging.critical("Errors in the control and/or strategy file, see TCA_Input_Summary.csv file for details")
        AKIPrintString("Errors in the control and/or strategy file, see TCA_Input_Summary.csv file for details")
    
    return 0

def change_vehicle_color(vehicle, veh_data, time, color_display_time):
    # TODO
    return 0

def AAPIManage(time, timeSta, timeTrans, acycle):
    return 0

def AAPIPostManage(time, timeSta, timeTrans, acycle):
    global enabled
    if enabled:
        global CF
        global changecolor
        global Regions
        global RSE_Tree
        global Algorithm
        global LastTP
        global Snapshots
        global headerBSM
        global headerCAM

        headerBSM = True
        headerCAM = True
        
        if CF.Control["OutputLevel"] > 0:
            if time % 1000 == 0:
               logger.info("Time Step: %d" % (time))
        
        vehicles = list()
        
        nba = AKIInfNetNbSectionsANG()
        for i in range(nba):
            id = AKIInfNetGetSectionANGId(i)
            nb = AKIVehStateGetNbVehiclesSection(id,True)
            for j in range(nb):
                vehicles.append(AKIVehStateGetVehicleInfSection(id,j))
        
        # TODO: vehicles in junctions

        #nbj = AKIInfNetNbJunctions()
        #for i in range(nbj):
        #    id = AKIInfNetGetJunctionId(i)
        #    nb = AKIVehStateGetNbVehiclesJunction(id)
        #    for j in range(nb):
        #        vehicles.append(AKIVehStateGetVehicleInfJunction(id,j))

        if len (vehicles) == 0:
            return 0

        vehicles_ids = []
        for veh in vehicles:
            active_veh = {'vehicle_ID' : veh.idVeh }
            vehicles_ids.append(active_veh)

        # Remove vehicle data of vehicles not seen in the concurrent timestep
        Algorithm.tbl.remove_non_active_vehicles(vehicles_ids)

        vehicles_list = []

        for veh in vehicles:
            vehicle = {
                    'vehicle_ID': veh.idVeh,
                    'time': float(time),
                    'type': AKIVehTypeGetIdVehTypeANG(veh.type),
                    'accel_instantaneous': (veh.CurrentSpeed - veh.PreviousSpeed)*5280/3600/acycle, # ft/s^2
                    'speed': float(veh.CurrentSpeed),  # mph
                    'location_x': float(veh.xCurrentPos), # ft
                    'location_y': float(veh.yCurrentPos), # ft
                    'link' : veh.idSection,
                    }
            vehicles_list.append(vehicle)
           
        vehicles_data = Algorithm.pull_veh_data(vehicles_list, time)
        if RSE_Tree is not None:
            range_data = RSE_Tree.find_ranges(vehicles_data)

        for i,veh_data in enumerate(vehicles_data):

            #if vehicle equipped
            if veh_data is not None:

                if veh_data['BSM_equipped']:
                    Algorithm.BSM.CheckBrakes(veh_data)

                if CF.Control['RegionsFile'] is not None:
                    Regions.CheckRegions(veh_data, time)

                #if SPOT equipped
                if veh_data['SPOT_equipped']:
                    Algorithm.SPOT.CheckMessage(veh_data, time) 
                    Algorithm.SPOT.CheckRange(veh_data, time) 

                #if PDM equipped
                #if (time % 1 ==0) and (veh_data['PDM_equipped']): # TODO: transmit every second
                if veh_data['PDM_equipped']: # TODO: transmit every second
                    Algorithm.PDM.CheckMessage(veh_data, time) 

                if veh_data['DSRC_enabled'] and CF.Control['RSELocationFile'] != None:
                    Algorithm.CheckDSRC(veh_data, time, range_data)

                Algorithm.CheckCellular(veh_data, time)

                #if (time % 1 ==0) and (veh_data['PDM_equipped']): # TODO: transmit every second
                if veh_data['PDM_equipped']: # TODO: transmit every second
                    Algorithm.PDM.PSNCheck(veh_data, time)

                if veh_data['BSM_equipped']:
                    Algorithm.BSM.tmp_ID_check(veh_data, time)

                #if changecolor:
                #    color_display_time = CF.Control['ColorDisplayDuration']
                #    change_vehicle_color(vehicles[i], veh_data, time, color_display_time)

                Algorithm.BSM.Write()
                Algorithm.CAM.Write()

                Algorithm.tbl.previous_values(veh_data)
        
        LastTP = time
        
    return 0

def AAPIFinish():
    global enabled
    if enabled:
        global CF
        global Algorithm
        global LastTP

        logger.info('End Run')
        if len(Algorithm.BSM.BSM_list) > 0 :
            Algorithm.BSM.Write(clear_buffer=True)

        if len(Algorithm.CAM.CAM_list) > 0 :
            Algorithm.CAM.Write(clear_all=True)

        if len(Algorithm.PDM.PDM_list) > 0 :
            Algorithm.PDM.Write(clear_buffer=True, LastTP = LastTP)

        if len(Algorithm.SPOT.Travelmsgs) > 0:
            Algorithm.SPOT.Write(clear_all=True)

        if CF.Control["OutputLevel"] > 0:
            Algorithm.tbl.list_veh_counts()

        if CF.Control["OutputLevel"] > 0:
            end_time = time.time()
            logger.info("End time %s (%f)" % (time.strftime('%X', time.localtime(end_time)), (end_time - program_st) ))
            logger.info("************  End Program  *******************")

        del Algorithm
    return 0

def AAPIUnLoad():
    return 0
    
def AAPIPreRouteChoiceCalculation(time, timeSta):
    return 0

def AAPIEnterVehicle(idveh, idsection):
    return 0

def AAPIExitVehicle(idveh, idsection):
    return 0

def AAPIEnterPedestrian(idPedestrian, originCentroid):
    return 0

def AAPIExitPedestrian(idPedestrian, destinationCentroid):
    return 0

def AAPIEnterVehicleSection(idveh, idsection, atime):
    return 0

def AAPIExitVehicleSection(idveh, idsection, atime):
    return 0
