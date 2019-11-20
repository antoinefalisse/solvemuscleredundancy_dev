import os
import opensim as osim

def OpenSim_Muscle_Analysis(motion_file,model_sel,output_path,event):
    #OpenSim_Muscle_Analysis Executes a muscle analsysis from the command line
    #   Detailed explanation goes here
    
    # get the analysis tool and change variables
    FunctionPath = os.path.dirname(os.path.abspath(__file__))
    path_generic_file = os.path.join(FunctionPath,'settings_Muscle_analysis.xml')
    
    MAtool=osim.AnalyzeTool(path_generic_file,False)
    MAtool.setLoadModelAndInput(True)
    osimModel=osim.Model(model_sel)
    MAtool.setModel(osimModel)
    MAtool.setResultsDir(output_path)
    MAtool.setInitialTime(event[0])
    MAtool.setFinalTime(event[1])
    (dirName, fileName) = os.path.split(motion_file)
    (name, fileExtension) = os.path.splitext(fileName)
    MAtool.setName(name)
    
    # run the analysis
    MAtool.setModelFilename(model_sel)
    MAtool.setCoordinatesFileName(motion_file)
    
    out_path_xml=os.path.join(output_path,'muscle_analysis_' + name + '.xml')
    MAtool.printToXML(out_path_xml)
    MAtool.run()
