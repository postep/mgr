#!/usr/bin/python3

import sys, os, subprocess

class FileData:
    def __init__(self, filename, first_timestamp, offset, ratio, name=""):
        self.filename = filename
        self.first_timestamp = first_timestamp
        self.offset = offset
        self.ratio = ratio
        self.plot_name = name

        print(self.filename)
        with open(self.filename) as fh:
            line = fh.readline()
            first = True
            i = 0
            self.data = []
            line_cnt = 0
            while line:
                if line_cnt % 2 == 0:
                    line_args = str(line[1:-2]).split(', ')
                    line_float = [float(x) for x in line_args]
                    if line_cnt/2 >= first_timestamp and line_cnt/2 <= self.first_timestamp + offset and i%self.ratio == 0:
                        self.data.append(line_float)
                        first = False
                    if int(line_cnt/2) > self.first_timestamp + offset:
                        break
                    if not first:
                        i += 1
                line_cnt += 1
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



def plot_traj(ax,traj,style,color,label, timestep = None):

    x = []
    y = []
    i = 0.0
    # traj = traj +zs [traj[0]]
    for co in traj:
        x.append(i)
        y.append(co[5])
        i += timestep
            
    ax.plot(x,y,style,color=color,label=label)
            

if __name__=="__main__":
    offset = int(sys.argv[1])
    ratio = int(sys.argv[2])

    file_list = []
    
    file_desc = sys.argv[4:]
    try:
        os.mkdir(f'../przerobione_testy/out/{str(sys.argv[3])}')
    except:
        pass
    
    for x in range(int(len(file_desc)/3)):
        file_arg_no = 3*x
        timestamp_arg_no = 3*x + 1
        name_no = 3*x + 2
        filename = f'{file_desc[file_arg_no]}'
        timestamp = float(file_desc[timestamp_arg_no])
        file_list.append(FileData(filename, timestamp, offset, ratio))
        
        first_timestamp = timestamp
        last_timestamp = timestamp + offset
        

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.pylab as pylab
    from matplotlib.patches import Ellipse

    fig = plt.figure()
    ax = fig.add_subplot(111)

    color_list = ['blue', 'green', 'red', 'yellow', 'brown', 'purple']
    i = 0
    first = True
    for tool in file_list:
        plot_traj(ax,tool.data,'-',color_list[i], f"traj_{tool.plot_name}", 1/100.0*ratio)
        i += 1

        
        ax.legend(loc='lower right', fancybox=True, framealpha=0.5)  

    ax.set_xlabel('t [s]')
    ax.set_ylabel('q [deg]')
    plot_name = f'../przerobione_testy/out/{str(sys.argv[3])}/common.png'
    print(plot_name)
    plt.savefig(plot_name,dpi=900)
