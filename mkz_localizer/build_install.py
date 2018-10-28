#!/usr/bin/env python3
import os
import shutil

project_path = os.getcwd()

#create default directory
if not (os.path.isdir(project_path+'/build') and os.path.isdir(project_path+'/bin')):
    os.system('./clear_project.py')
os.chdir(project_path+'/build')

#run cmake
ret = os.system('cmake ..')
if ret!=0:
    print ("Error: cmake error, please check cmake info, exiting...")
    exit()

#run make
ret = os.system('make -j8 -l8')
if ret!=0:
    print("Error: make error, please check make info, exiting...")
    exit()

#clear project
os.chdir(project_path)
os.system(project_path+'/clear_project.py')

#copy executables
application_path = project_path+"/../applications"
if os.path.isdir(application_path):
    dirs = os.listdir(project_path+'/bin')
    for dir in dirs:
        if dir.startswith('app_') or dir.endswith('.ico') or dir.endswith('.so') or dir.endswith('.3DS') and (not (dir.endswith('INFO') or dir.endswith('WARNING') or dir.endswith('ERROR') or dir.endswith('FATAL'))) and dir.find('.log.')==-1:
            shutil.copy2(project_path+'/bin/'+dir, application_path)
