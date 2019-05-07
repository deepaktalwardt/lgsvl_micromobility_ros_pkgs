import csv

input_path = "/home/deepaktalwardt/Dropbox/SJSU/Semesters/Spring 2019/CMPE 297/datasets/large_dataset_1/gt_2d/gt_2d_yolo2_annotations.csv"
output_path = "/home/deepaktalwardt/Dropbox/SJSU/Semesters/Spring 2019/CMPE 297/datasets/large_dataset_1/gt_2d/gt_2d_yolo3_annotations.txt"

def convert_annotations(input_path, output_path):

    input_list = []
    with open(input_path, 'r') as f_in:
        for line in f_in:
            line_split = line.split(';')
            line_2_col = line_split[2].strip('\n')
            line_2_col.replace('"', '')
            line_arr = ['../large_dataset_1/main_camera/' + line_split[0], line_2_col]
            input_list.append(line_arr)

    with open(output_path, 'a') as f_out:
        # writer = csv.writer(f_out, delimiter=' ', quoting=csv.QUOTE_NONE, escapechar='')
        for line in input_list:
            f_out.write(line[0] + ' ' + line[1] + '\n')

if __name__ == "__main__":
    convert_annotations(input_path, output_path)