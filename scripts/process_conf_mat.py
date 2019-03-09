# import roslib; roslib.load_manifest('muse_armcl')
# import rospy

import math
import numpy as np
import matplotlib.pyplot as plt
import sys, getopt


def import_file(path):

    return np.genfromtxt (path, delimiter=",")


def remove_header(data):
    rows = data[:,0].size
    cols = data[0,:].size
    mat = data[1:rows, 1:cols]

    return {'header': data[1:rows, 0], 'matrix': mat}

def n_samples(mat):
    count = sum(mat,0)
    return sum(count)	
def get_relative_conf_mat(mat):
    count = sum(mat, 0)
    return mat / count

def removes_finger(data):
    header = data['header']
    conf_mat = data['matrix']
    cond = (np.array(header) / 100).astype(int) != 7
    idx = np.where(cond)[0].tolist()
    new_header = header[idx]

    rconf_mat = np.zeros([len(idx), len(idx)])
    for i in idx:
        for j in idx:
            rconf_mat[i,j] = conf_mat[i,j]

    return {'header': new_header, 'matrix': rconf_mat}


def get_rates(mat):
    rows = mat[:, 0].size
    cols = mat[0, :].size
    if (True in np.isnan(mat)) | (rows == 1 & cols == 1):
        contact_hit_rate = 0
        non_contact_hit_rate = 0
        total_hit_rate = 0
        not_detected_rate = 0
        #print(mat)
    else:
        #print(np.isnan(mat))
        contact_hit_count = np.sum(np.diag(mat[1:rows, 1:cols]))
        contact_hit_norm = np.sum(np.sum(mat[1:rows, 1:cols]))

        contact_hit_rate = contact_hit_count / contact_hit_norm

        non_contact_hit_rate = mat[0,0] / np.sum(mat[:,0])

        total_hit_rate = np.sum(np.diag(mat)) / np.sum(np.sum(mat))

        not_detected_rate = np.sum(mat[0,1:cols]) / np.sum(np.sum(mat[0:rows, 1:cols]))
        #print('non_contact_hit_rate ', non_contact_hit_rate,
        #    ' contact_hit_rate ', contact_hit_rate,
        #    ' not_detected_rate ', not_detected_rate,
        #    ' total_hit_rate ', total_hit_rate)
    return {'non_contact_hit_rate': non_contact_hit_rate,
            'contact_hit_rate': contact_hit_rate,
            'not_detected_rate': not_detected_rate,
            'total_hit_rate': total_hit_rate}


def combine_events(data, combine_map):
    labels = data['header']
    mat = data['matrix']

    new_labels = np.copy(labels)
    for key in combine_map:
        indces = []
        for ele in combine_map[key]:
            cond = labels == ele
            new_labels[cond] = key
            tmp = np.where(cond)[0].tolist()
            if len(tmp) > 0:
                indces.append(tmp)

    new_labels_u = np.unique(new_labels)
    nz = len(new_labels_u)
    new_mat = np.zeros((nz,nz))
    print new_labels
    for row in range(len(labels)):
        label = new_labels[row]
        new_row = np.where(np.array(new_labels_u) == label)
        for col in range(len(labels)):
            label_c = new_labels[col]
            new_c = np.where(np.array(new_labels_u) == label_c)
            new_mat[new_row,new_c] = new_mat[new_row,new_c] + mat[row,col]

    return {'header': np.unique(new_labels), 'matrix': new_mat}
    #     first = indces[0]
    #     indces.pop(0)
    #     for idx in indces:
    #         if len(idx) > 0:
    #             new_mat[first, :] = new_mat[first, :] + new_mat[idx, :]
    #     for idx in indces:
    #         if len(idx) > 0:
    #             new_mat[:, first] = new_mat[:, first] + new_mat[:, idx]
    #     new_mat = np.delete(new_mat, indces, 0)
    #     new_mat = np.delete(new_mat, indces, 1)
    # print np.unique(new_labels)
    # return {'header': np.unique(new_labels), 'matrix': new_mat}


def plot_conf_mat(mat, labels):
    fig = plt.figure()

    axes = plt.gca()
    axes.set_xticks([i for i in range(0, len(mat[0]))])
    axes.set_yticks([i for i in range(0, len(mat))])
    axes.set_xticklabels(['No Collision'] + [str(int(labels[i])) for i in range(1, len(labels))], fontsize='7', fontweight='bold', rotation='vertical')
    axes.set_yticklabels(['No Collision'] + [str(int(labels[i])) for i in range(1, len(labels))], fontsize='7', fontweight='bold')

    cax = plt.imshow(mat, cmap='jet',interpolation='none')
    cbar = fig.colorbar(cax, ticks=[0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1])

    return fig


def plot_diag(mat, labels):
    fig = plt.figure()
    axes = plt.gca()
    plt.plot(labels, np.diag(mat),'*')
    #axes.set_xticks([i for i in range(0, len(labels))])
    axes.set_xticklabels(['No Collision'] + [str(int(labels[i])) for i in range(1, len(labels))], fontsize='7',
                        fontweight='bold', rotation='vertical')

    return  fig

def combine_links(labels):
    nl = np.floor(labels / 100)
    res = {1: labels[nl == 1].tolist(),
           2: labels[nl == 2].tolist(),
           3: labels[nl == 3].tolist(),
           4: labels[nl == 4].tolist(),
           5: labels[nl == 5].tolist(),
           6: labels[nl == 6].tolist(),
           7: labels[nl == 7].tolist()}
    return res

def combine_links_base_nc(labels):
    nl = np.floor(labels / 100)
    res = {0: labels[nl == 1].tolist(),
           2: labels[nl == 2].tolist(),
           3: labels[nl == 3].tolist(),
           4: labels[nl == 4].tolist(),
           5: labels[nl == 5].tolist(),
           6: labels[nl == 6].tolist(),
           7: labels[nl == 7].tolist()}
    return res

combine_height = {120: [121, 122],
                  210: [211, 212, 213, 214], 220: [221, 222, 223], 230: [231, 232, 233, 234], 240: [241, 242, 243, 244],
                  310: [312, 313], 320: [321, 322, 323], 330: [331, 332, 333], 340: [341, 342, 343]}


combine_base_nc = {10: [-1, 111, 121, 122, 131, 141]}

combine_height_base_nc = {10: [-1, 0, 111, 121, 122, 131, 141],
                        210: [211, 212, 213, 214], 220: [221, 222, 223], 230: [231, 232, 233, 234], 240: [241, 242, 243, 244],
                        310: [312, 313], 320: [321, 322, 323], 330: [331, 332, 333], 340: [341, 342, 343]}


