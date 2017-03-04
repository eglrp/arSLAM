import numpy as np

import matplotlib.pyplot as plt

import re

if __name__ == '__main__':
    if __name__ == '__main__':
        # base file for compute pose of tag(ar code )
        base_file_list = list()
        base_file_list.append("../data1.g2o")

        target_file_name="../data2_single.g2o"
        target_file = open(target_file_name)
        tag_min_id = 10
        tag_max_id = 80
        tag_fix_id = 11

        output_file = open("../Output_mix.g2o",'w')




        # write tag
        for i in range(tag_min_id,tag_max_id):
            output_file.write('VERTEX_SE3:QUAT {0} 0 0 0 0 0 0 1 \n'.format(i))
        output_file.write('FIX {0} \n'.format(tag_fix_id))

        vertex_list = list()
        edge_list = list()

        # read target file without extract offset

        for line in target_file.readlines():
            if 'VERTEX_SE3' in line:
                if int(line.split(' ')[1]) > 1000:

                    vertex_list.append(line)
            if 'EDGE_SE3' in line:
                edge_list.append(line)


        for i in range(len(base_file_list)):
            tag_offset = (i+1) * 100000
            tmp_file = open(base_file_list[i])
            for line in tmp_file.readlines():
                if 'VERTEX_SE3' in line:
                    if int(line.split(' ')[1]) > 1000:
                        the_tag_id = int(line.split(' ')[1])
                        the_tag_id += tag_offset

                        tmp_line = line.replace(line.split(' ')[1],str(the_tag_id+10))
                        # tmp_line = tline
                        print('----\n',
                              line,'\n',tmp_line)
                        vertex_list.append(tmp_line)
                if 'EDGE_SE3' in line:
                    if int(line.split(' ')[1]) > 1000:
                        the_tag_id = int(line.split(' ')[1])
                        the_tag_id += tag_offset

                        tmp_line = line.replace(line.split(' ')[1],str(the_tag_id+10))
                        print('----\n',
                          line,'\n',tmp_line)
                        edge_list.append(tmp_line)

        output_file.writelines(vertex_list)
        output_file.writelines(edge_list)













