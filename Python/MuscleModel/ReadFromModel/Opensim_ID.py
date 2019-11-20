import os
import opensim as osim

def Opensim_ID(model_dir,event,input_GRF_settings,input_motion,output_path,output_name,output_settings):
#Opensim_ID Calculates Inverse Dynamics
#   model_dir               path+name of the model
#   event                   rowvector with start and end time
#   input_GRF_settings      path+name of the input GRF settings
#   input_motion            path+name of the input motion (ik)
#   output_path             path of the output file
#   output_name             name of the output file
#   output_settings         path+name of the settings file    
      
    FunctionPath = os.path.dirname(os.path.abspath(__file__))
    path_generic_file = os.path.join(FunctionPath,'generic_ID_settings.xml')      
    # use the loaded model
    model=osim.Model(model_dir)
    model.initSystem()     # initialise the model    
    # initialise the ID tool
    idTool = osim.InverseDynamicsTool(path_generic_file)
    idTool.setModel(model)
    # input external loads
    idTool.setExternalLoadsFileName(input_GRF_settings)
    # get the name    
    (dirName, fileName) = os.path.split(input_motion)
    (name, fileExtension) = os.path.splitext(fileName)
    # Setup the idTool for this trial
    idTool.setName(name)
    idTool.setCoordinatesFileName(input_motion)
    idTool.setLowpassCutoffFrequency(6)    
    # set up the events
    idTool.setStartTime(event[0])
    idTool.setEndTime(event[1])    
    # set output of the id tool
    idTool.setResultsDir(output_path)
    idTool.setOutputGenForceFileName(output_name)    
    # Save the settings in a setup file
    idTool.printToXML(output_settings)    
    # run the idTool
    idTool.run()
    