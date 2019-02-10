#!/usr/bin/env python

import roslib; roslib.load_manifest('muse_armcl')
import rospy

import numpy as np
import matplotlib.pyplot as plt
import sys, getopt
import process_conf_mat as cm
from matplotlib2tikz import save as tikz_save

def main(argv):
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print('test.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is "', inputfile)
    print('Output file is "', outputfile)

    csv = cm.import_file(inputfile)
    data_map = cm.remove_header(csv)

    conf_mat = data_map['matrix']
    rel_mat = cm.get_relative_conf_mat(data_map['matrix'])
    print(cm.get_rates(conf_mat))
    print('Number of samples: ',cm.n_samples(conf_mat))

    fig = cm.plot_conf_mat(rel_mat, data_map['header'])

    if outputfile != '':
        #fig.savefig(outputfile, bbox_inches='tight')
        tikz_save(outputfile + '.tex',  figureheight='6cm', figurewidth='6cm')
    plt.show()

    #l = []
    #for key in data_map['header']:
    #    print(key)
    #    l.append(data_map['header'][key])

    plt.plot(data_map['header'], np.diag(rel_mat),'*')
    plt.show()


if __name__ == "__main__":
   main(sys.argv[1:])
