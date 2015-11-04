from distutils.core import setup 
import glob  
import py2exe 
import sys

if len(sys.argv) == 1:
    sys.argv.append("py2exe")
    
includes = ["encodings", "encodings.*"]  
options = {"py2exe":  
             {   "compressed": 1,  
                 "optimize": 2,  
                 "includes": includes,  
		 "dist_dir": "bin", 
                 #"bundle_files": 1  
             }  
           }  
setup(     
     version = "1.0",  
     description = "MiniFM Application",  
     name = "MiniFM",  
     options = options,  
     zipfile=None,  
     windows=[{"script": "MiniFM.py", "icon_resources": [(1, "MiniFM.ico")] }],    
     data_files=[("", ['MiniFM.ico', ]), ("images",glob.glob("images\\*.png"))],  
     ) 