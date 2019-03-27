#!/usr/bin/env python
import roslib; roslib.load_manifest('muse_armcl')
import rospy

import numpy as np
import matplotlib.pyplot as plt
import sys, getopt
import process_conf_mat as cm
import os
import operator


def get_conf_mats(rootdir):
    res =[]
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            if ('confusion_matrix' in file) & ('rel' not in file):
                #print(os.path.join(subdir, file))
                res.append(os.path.join(subdir, file))
    return res


def process_conf_mats(lis):
    res_map = {}
    for f in lis:
        csv = cm.import_file(f)
        data_map = cm.remove_header(csv)

        conf_mat = data_map['matrix']
        rel_mat = cm.get_relative_conf_mat(data_map['matrix'])
        results = cm.get_rates(conf_mat)
        #print(f)
        res_map[f] = results

    return res_map


def find_best(result_map):
    max_val = -42
    best = ' '
    res = {}
    sort_x = {}
    for key in result_map:
        m = result_map[key]
        val = m['total_hit_rate']
        sort_x[key] = val
        if val > max_val:
            max_val = val
            best = key
            res = m
    sorted_x = sorted(sort_x.items(), key=operator.itemgetter(1))
    return {'file': best, 'value': res, 'sorted': sorted_x}


def main(argv):
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ipath=","ofile="])
    except getopt.GetoptError:
        print('evaluation_script.py -i <inputpath> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <inputpath> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ipath"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is "', inputfile)
    print('Output file is "', outputfile)

    conf_mats = get_conf_mats(inputfile)
    print(conf_mats)
    if len(conf_mats) > 0:
        results = process_conf_mats(conf_mats)
        best = find_best(results)

        if not(outputfile == ''):
            f = open(outputfile, 'w')
            f.write('Best Result is: \n')
            f.write(best['file'] + ':  ' + str(best['value']) + '\n')
            f.write('\n \n')
            f.write('====================== \n \n')
            sorted_map = best['sorted']
            for keys in sorted_map:
                f.write(keys[0] + ": " + str(results[keys[0]]) + '\n')
            f.close()
        else:
            print(best['file'] + ', ' + str(best['value']))


if __name__ == "__main__":
    main(sys.argv[1:])
