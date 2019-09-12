#!/usr/bin/python3


# import glob, os
# os.chdir("../przerobione_testy")
# for file_cmd in glob.glob("cmd*.txt"):
#     file_tool = file_cmd.replace('cmd', 'tool')
#     print(file_cmd, file_tool)


import shlex, subprocess
args = ['./evaluate_ate.py', f'--plot {file_cmd.replace("cmd", "plot").replace("txt", "png")}', '--fixed_delta 1', '--verbose', f'--first_file {file_cmd}', f'--second_file {file_tool}']
print(args)
p = subprocess.Popen(args, stdout=subprocess.PIPE) # Success!

stdout = p.communicate()[0]
print ('STDOUT:{}'.format(stdout))