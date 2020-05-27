from distutils.core import setup 
import glob  
import py2exe 
import sys

if len(sys.argv) == 1:
    sys.argv.append("py2exe")
    
options = {"py2exe":  
             {   "compressed": 1,  
                 "optimize": 1,  
                 "includes": ["sip","encodings", "encodings.*","ctypes", "logging"],  
		 "dist_dir": "bin", 
                 "bundle_files": 3,
		 "excludes":["pythoncom"],		 
		 #"dll_excludes": ["numpy-atlas.dll"],		 
             }  
           }  
setup(     
     version = "1.0",  
     description = "MiniGCS Application",  
     name = "MiniGCS",  
     options = options,  
     zipfile=None,  
     windows=[{"script": "MiniGCS.py", "icon_resources": [(1, "MiniGCS.ico")] }],    
     data_files=[("", ['MiniGCS.ico', ]), ("images",glob.glob("images\\*.png"))],  
     ) 