#!/usr/bin/python3

import sys, os, subprocess
os.chdir("../przerobione_testy")
file_cmd = sys.argv[1]
file_tool = file_cmd.replace('cmd', 'tool')
first_timestamp = int(sys.argv[2])
last_timestamp = int(sys.argv[3])
move_type = sys.argv[4]
try:
    ratio = int(sys.argv[5])
except:
    ratio = 1

try:
    mode = str(sys.argv[6])
except:
    mode = 'xy'


file_cmd_move = f'./podzielone/{move_type}_{file_cmd}'
file_tool_move = f'./podzielone/{move_type}_{file_tool}'

with open(file_cmd) as cmd_fh:
    with open(file_tool) as tool_fh:
        with open(file_cmd_move, 'w') as cmd_move_fh:
            with open(file_tool_move, 'w') as tool_move_fh:
                line_cmd = cmd_fh.readline()
                line_tool = tool_fh.readline()
                first = True
                i = 0
                while line_cmd:
                    line_cmd = cmd_fh.readline()
                    line_args_cmd = line_cmd.split(' ')
                    line_tool = tool_fh.readline()
                    if int(line_args_cmd[0]) >= first_timestamp and int(line_args_cmd[0]) <= last_timestamp and i%ratio == 0:
                        cmd_move_fh.write(line_cmd)
                        tool_move_fh.write(line_tool)
                        first = False
                    if int(line_args_cmd[0]) > last_timestamp:
                        break
                    if not first:
                        i += 1


import shlex, subprocess

args1 = ['./evaluate_ate.py', '--mode', mode, f'--plot', f'{file_cmd_move.replace("cmd", "ate_plot").replace("txt", "png")}', '--save_associations', file_cmd_move.replace('cmd', 'associations'), '--verbose', f'--first_file', f'{file_cmd_move}', f'--second_file', f'{file_tool_move}']
args2 = ['./evaluate_rpe.py', f'--plot', f'{file_cmd_move.replace("cmd", "rpe_plot").replace("txt", "png")}', '--fixed_delta', '--verbose', f'--first_file', f'{file_cmd_move}', f'--second_file', f'{file_tool_move}']
for args in [args1, args2]:
    print(args)
    p = subprocess.Popen(args, stdout=subprocess.PIPE) # Success!

    stdout = p.communicate()[0]
    std_str = str(stdout).split("\\n")
    std_str[0] = std_str[0][2:]
    std_str = std_str[:-1]
    std_str = ' '.join(std_str)
    print ('STDOUT:', std_str)

    if args == args1:
        with open(file_cmd_move.replace("cmd", "ate_out"), 'w') as fh:
            fh.write(std_str)
    else:
        with open(file_cmd_move.replace("cmd", "rpe_out"), 'w') as fh:
            fh.write(std_str)


