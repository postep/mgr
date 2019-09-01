#!/usr/bin/python3


# import glob, os
# os.chdir("../przerobione_testy")
# for file_cmd in glob.glob("cmd*.txt"):
#     file_tool = file_cmd.replace('cmd', 'tool')
#     print(file_cmd, file_tool)

import sys, os
os.chdir("../przerobione_testy")
file_cmd = sys.argv[1]
file_tool = file_cmd.replace('cmd', 'tool')
first_timestamp = int(sys.argv[2])
last_timestamp = int(sys.argv[3])
move_type = sys.argv[4]

file_cmd_move = f'{move_type}_{file_cmd}'
file_tool_move = f'{move_type}_{file_tool}'

with open(file_cmd) as cmd_fh:
    with open(file_tool) as tool_fh:
        with open(file_cmd_move, 'w') as cmd_move_fh:
            with open(file_tool_move, 'w') as tool_move_fh:
                line_cmd = cmd_fh.readline()
                line_tool = tool_fh.readline()
                while line_cmd:
                    line_cmd = cmd_fh.readline()
                    line_args_cmd = tool_fh.split(' ')
                    line_tool = file_tool.readline()
                    if int(line_args_cmd[0]) >= first_timestamp and int(line_args_cmd[0]) <= last_timestamp:
                        file_cmd_move.write(line_cmd)
                        file_tool_move.write(line_tool)
                    if int(line_args_cmd[0]) > last_timestamp:
                        break


args = ['./evaluate_ate.py', f'--plot {file_cmd.replace("cmd", f"plot_{move_type}").replace("txt", "png")}', '--fixed_delta 1', '--verbose', f'--first_file {file_cmd_move}', f'--second_file {file_tool_move}']
print(args)
p = subprocess.Popen(args, stdout=subprocess.PIPE) # Success!

stdout = p.communicate()[0]
print ('STDOUT:{}'.format(stdout))

exit(0)



import shlex, subprocess
args = ['./evaluate_ate.py', f'--plot {file_cmd.replace("cmd", "plot").replace("txt", "png")}', '--fixed_delta 1', '--verbose', f'--first_file {file_cmd}', f'--second_file {file_tool}']
print(args)
p = subprocess.Popen(args, stdout=subprocess.PIPE) # Success!

stdout = p.communicate()[0]
print ('STDOUT:{}'.format(stdout))