#!/usr/bin/python3

import sys, os, subprocess

class FileData:
    def __init__(self, filename, first_timestamp, offset, ratio, name=""):
        self.filename = filename
        self.first_timestamp = first_timestamp
        self.offset = offset
        self.ratio = ratio
        self.plot_name = name

        with open(self.filename) as fh:
            line = fh.readline()
            first = True
            i = 0
            self.data = []
            while line:
                line_args = line.split(' ')
                line_args[-1] = line_args[-1][:-1]

                line_args_float = [float(x) for x in line_args]
                if line_args_float[0] >= first_timestamp and line_args_float[0] <= self.first_timestamp + offset and i%self.ratio == 0:
                    self.data.append(line_args_float)
                    first = False
                if int(line_args_float[0]) > self.first_timestamp + offset:
                    break
                if not first:
                    i += 1

                line = fh.readline()

    def get_count(self):
        return len(self.data)

    def assert_count(self):
        estimated = (self.last_timestamp-self.first_timestamp)%self.ratio + 1
        print('ASSERT EST', estimated, 'CNT', self.get_count())
        return estimated == self.get_count()

    def writeFile(self, filename):
        with open(filename, 'w') as fh:
            for x in self.data:
                line = ' '.join([str(y) for y in x])
                line += '\n'
                fh.write(line)


import math
def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def plot_traj(ax,traj,style,color,label,mode, timestep = None):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend

    """
    x = []
    y = []
    i = 0.0
    # traj = traj +zs [traj[0]]
    for co in traj:
        rotX, rotY, rotZ = quaternion_to_euler(co[4], co[5], co[6], co[7])
        if mode == 'xy':
            x.append(co[0+1])
            y.append(co[1+1])
        if mode == 'xz':
            x.append(co[0+1])
            y.append(co[2+1])
        if mode == 'yz':
            x.append(co[1+1])
            y.append(co[2+1])

        if mode == 'rotx':
            x.append(i)
            y.append(rotX)
        if mode == 'roty':
            x.append(i)
            y.append(rotY)
        if mode == 'rotz':
            x.append(i)
            y.append(rotZ)

        if mode == 'ax':
            x.append(i)
            y.append(co[1])
        if mode == 'ay':
            x.append(i)
            y.append(co[2])
        if mode == 'az':
            x.append(i)
            y.append(co[3])
        i += timestep

    ax.plot(x,y,style,color=color,label=label)


if __name__=="__main__":
    os.chdir("../przerobione_testy")
    offset = int(sys.argv[1])
    ratio = int(sys.argv[2])

    cmd_file_list = []
    tool_file_list = []

    file_desc = sys.argv[4:]
    try:
        os.mkdir(f'out/{str(sys.argv[3])}')
    except:
        pass

    for x in range(int(len(file_desc)/3)):
        file_arg_no = 3*x
        timestamp_arg_no = 3*x + 1
        name_no = 3*x + 2
        file_cmd = f'cmd_{file_desc[file_arg_no]}'
        file_tool = file_cmd.replace('cmd', 'tool')
        timestamp = float(file_desc[timestamp_arg_no])
        print(file_cmd, timestamp)
        cmd_file_list.append(FileData(file_cmd, timestamp, offset, ratio))
        tool_file_list.append(FileData(file_tool, timestamp, offset, ratio, file_desc[name_no]))

        first_timestamp = timestamp
        last_timestamp = timestamp + offset

        file_cmd_move = f'out/{str(sys.argv[3])}/{file_cmd}'
        file_tool_move = f'out/{str(sys.argv[3])}/{file_tool}'

        cmd_file_list[-1].writeFile(file_cmd_move)
        tool_file_list[-1].writeFile(file_tool_move)
        mode_list = ['xy', 'yz', 'xz']
        for mode in mode_list:
            import shlex, subprocess
            plot_output = f'out/{str(sys.argv[3])}/{mode}_{file_tool}'.replace("tool", "ate_plot").replace("txt", "png")
            args1 = ['./evaluate_ate.py', '--mode', mode, f'--plot', plot_output, '--save_associations', file_cmd_move.replace('cmd', 'associations'), '--verbose', f'--first_file', f'{file_cmd_move}', f'--second_file', f'{file_tool_move}']
            args2 = ['./evaluate_rpe.py', f'--plot', plot_output.replace("ate_plot", "rpe_plot"), '--fixed_delta', '--verbose', f'--first_file', f'{file_cmd_move}', f'--second_file', f'{file_tool_move}']
            for args in [args1]:
                print(args)
                p = subprocess.Popen(args, stdout=subprocess.PIPE) # Success!

                stdout = p.communicate()[0]
                std_str = str(stdout).split("\\n")
                std_str[0] = std_str[0][2:]
                std_str = std_str[:-1]
                std_str = ' '.join(std_str)
                print ('STDOUT:', std_str)
                print("STDFILE", os.getcwd(), plot_output.replace("ate_plot", "ate_out").replace("png", 'txt'))

                if args == args1:
                    with open(plot_output.replace("ate_plot", "ate_out").replace("png", 'txt'), 'w') as fh:
                        fh.write(std_str)
                # else:
                #     with open(file_cmd_move.replace("ate_plot", "rpe_out"), 'w') as fh:
                #         fh.write(std_str)


    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.pylab as pylab
    from matplotlib.patches import Ellipse

    mode_list = ['xy', 'yz', 'xz', 'rotx', 'roty', 'rotz', 'ax', 'ay', 'az']
    for mode in mode_list:

        fig = plt.figure()
        ax = fig.add_subplot(111)

        color_list = ['blue', 'green', 'red', 'yellow', 'brown', 'purple']
        i = 0
        first = True
        for (cmd, tool) in zip(cmd_file_list, tool_file_list):
            if first: plot_traj(ax,cmd.data,'-',"black","traj_zadana", mode, 1/100.0*ratio)
            first = False
            plot_traj(ax,tool.data,'-',color_list[i], f"traj_{tool.plot_name}", mode, 1/100.0*ratio)
            i += 1


        ax.legend(loc='lower right', fancybox=True, framealpha=0.5)

        if mode == 'xy':
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
        if mode == 'xz':
            ax.set_xlabel('x [m]')
            ax.set_ylabel('z [m]')
        if mode == 'yz':
            ax.set_xlabel('y [m]')
            ax.set_ylabel('z [m]')

        if mode == 'rotx':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('x [deg]')
        if mode == 'rotz':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('z [deg]')
        if mode == 'roty':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('y [deg]')

        if mode == 'ax':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('x [m]')
        if mode == 'az':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('z [m]')
        if mode == 'ay':
            ax.set_xlabel('t [s]')
            ax.set_ylabel('y [m]')

        plot_name = f'out/{str(sys.argv[3])}/common_{mode}.png'
        print(plot_name)
        plt.savefig(plot_name,dpi=300)


    import sys, os, subprocess

