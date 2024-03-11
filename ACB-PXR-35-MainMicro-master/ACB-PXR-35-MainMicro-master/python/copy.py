import datetime
import shutil
import os
import sys

now = datetime.datetime.today() 
nTime = now.strftime("%Y%m%d-%H%M")

target_folder = 'c:\\development\\builds_pxr35\\MainMicro\\'
target_folder_batch = 'c:\\development\\builds_pxr35\\batch\\mainmicro\\'

target_path1 = sys.argv[1]
target_path2 = sys.argv[2]
target_path_separator = '\\'

bin_suffix_out = '.out'
bin_suffix_txt = '.hex'

target = 'Debug\\Exe\\'

MainMicro_binary = 'PXR35_ProtProc'
MainMicro_Source = os.path.join(target + MainMicro_binary + bin_suffix_txt)

dest = os.path.join(target_folder + target_path1 + target_path_separator + target_path2)

MainMicro_dest_dir = str(dest) + '\\' + MainMicro_binary
if not os.path.exists(MainMicro_dest_dir):
    os.makedirs(MainMicro_dest_dir)

shutil.copy2(MainMicro_Source, MainMicro_dest_dir)

shutil.copytree(target_folder_batch, dest, dirs_exist_ok=True)
#shutil.copytree(target_folder_config, os.path.join(dest + '\\config\\'), dirs_exist_ok=True)