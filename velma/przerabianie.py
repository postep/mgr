#!/usr/bin/python3
class PoseQuat:
    x = None
    y = None
    z = None

    qx = None
    qy = None
    qz = None
    qw = None

    def get_from_list(self, l):
        self.x = l[0]
        self.y = l[1]
        self.z = l[2]

        self.qx = l[3]
        self.qy = l[4]
        self.qz = l[5]
        self.qw = l[6]

    def __str__(self):
        return f"{self.x} {self.y} {self.z} {self.qx} {self.qy} {self.qz} {self.qw}"

import glob, os
os.chdir("./testy")
for file in glob.glob("podnoszenie*.txt"):
    print(file)
    with open(f"../przerobione_testy/cmd_{file}", 'w') as cmd_fh:
        with open(f"../przerobione_testy/tool_{file}", 'w') as tool_fh:
            with open(file) as fh:
                line = fh.readline()
                while line:
                    split_line = line.replace('[', '').replace(']', '').replace(',', '').split(' ')
                    cmd = PoseQuat()
                    cmd.get_from_list(split_line[0:7])
                    tool = PoseQuat()
                    tool.get_from_list(split_line[7:7+7])
                    timestamp = split_line[14].replace('\n', '').replace('\r', '')

                    cmd_fh.write(f"{timestamp} {cmd}\n")
                    tool_fh.write(f"{timestamp} {tool}\n")
                    line = fh.readline()

