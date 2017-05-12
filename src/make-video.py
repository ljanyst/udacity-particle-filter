#!/usr/bin/env python3
#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   29.04.2017
#-------------------------------------------------------------------------------

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import sys
import os


if len(sys.argv) != 4:
    print("Usage:", sys.argv[0], "datadir input1000 input1")
    sys.exit(1)

i = 0
files = []
gt    = []
lm    = []

with open(sys.argv[1] + '/gt_data.txt', 'r') as f:
    for l in f.readlines():
        l = l.strip().split(' ')
        l = list(map(lambda x: float(x), l))
        gt.append(l)

with open(sys.argv[1] + '/map_data.txt', 'r') as f:
    for l in f.readlines():
        l = l.strip().split(' ')
        l = list(map(lambda x: float(x), l))
        lm.append(l)

gtx, gty, gttheta = zip(*gt)
lmx, lmy, lmid = zip(*lm)

with open(sys.argv[3], 'r') as f:
    lines_other = f.readlines()

with open(sys.argv[2], 'r') as f:
    for l1 in f.readlines():
        print("step: ", i)
        l1 = l1.strip().split(' ')
        l1 = list(map(lambda x: float(x), l1))
        l2 = lines_other[i].strip().split(' ')
        l2 = list(map(lambda x: float(x), l2))
        fig, ax = plt.subplots()
        ax.plot(gtx, gty, 'y--', label="ground truth")
        ax.plot(lmx, lmy, 'cP', label="landmarks")
        ax.plot(l2[0], l2[1], 'bD', label="robot (1 particle)")
        ax.plot(l1[0], l1[1], 'mD', label="robot (best of 1000)")
        ax.set_autoscale_on(False)
        ax.axis([-60, 300, -110, 40])
        ax.set_title('step: ' + str(i))
        legend = ax.legend(loc='upper left')
        fig.set_size_inches(16, 9)
        filename = "data-{:05d}.png".format(i)
        fig.savefig(filename, bbox_inches='tight')
        plt.close(fig)
        files.append(filename)
        i += 1

print("Building the video...")
os.system('ffmpeg -y -framerate 50 -i data-%05d.png output.mp4 2>/dev/null')
for f in files:
    os.remove(f)
