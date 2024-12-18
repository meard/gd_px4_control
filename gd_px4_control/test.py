 
# import os
 
# # Function to Get the current 
# # working directory
# def current_path():
#     print("Current working directory before")
#     print(os.getcwd())
#     print()
 
 
# # Driver's code
# # Printing CWD before
# current_path()
 
# # Changing the CWD
# os.chdir('../model/')
 
# # Printing CWD after
# current_path()

import os

# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)

base_path =  __file__
os.chdir(base_path)

print('getcwd:      ', os.getcwd())
print('base path:    ', base_path)