### usage

#### compile

##### g++ -O3 -DNDEBUG -o evaluate_3d eval.cpp -lglog -std=c++11

#### run

##### ./evaluate_3d gt.txt det_cyj.txt {out_file_name[can be ingored]}

#### Format
##### gt_format
###### \# 1
###### 000001.png    //(file_name)
###### 3 1280 1920 1//(img_chanel img_height img_width flip)
###### 2						//ignore num
###### 1 0.00 0 1.96 280.38 185.10 344.90 215.59 1.49 1.76 4.01 -15.71 2.16 38.26 1.57 3
###### 1 0.00 0 1.88 365.14 184.54 406.11 205.20 1.38 1.80 3.41 -15.89 2.23 51.17 1.58 2
###### 3						//gt num
###### 1 0.00 0 -1.57 599.41 156.40 629.75 189.25 2.85 2.63 12.34 0.47 1.49 69.44 -1.56 1
###### 1 0.00 0 1.85 387.63 181.54 423.81 203.12 1.67 1.87 3.69 -16.53 2.39 58.49 1.57 2
###### 2 0.00 3 -1.65 676.60 163.95 688.98 193.93 1.86 0.60 2.02 4.59 1.32 45.84 -1.55 1
###### label-truncation-occlusion-alpha-x1-y1-x2-y2-h-w-l-t1-t2-t3-ry severity_level

##### label: class_label
###### truncation: & occlusion 0.00 0
###### alpha:Observation angle of object, ranging [-pi..pi]
###### x1 y1 x2 y2: 2D BBox coordinate
###### h w l: 3D object dimensions: height, width, length (in meters)
###### t1 t2 t3: 3D object location x,y,z in camera coordinates (in meters)
###### ry:Rotation ry around Y-axis in camera coordinates [-pi..pi]
###### severity_level: 1, 2, 3

#### det_format
###### 000001.png 1 1.98 386.06 180.49 422.45 200.67 1.46 1.49 2.95 -14.20 2.06 50.00 1.70 0.650 0.77442261094 1
###### 000002.png 1 0.04 658.79 191.24 702.51 220.33 1.46 1.49 2.95 3.00 2.06 31.80 0.13 0.962 0.914835452597 1
###### 000002.png 3 -1.16 559.00 143.75 572.67 215.48 1.76 0.66 0.49 -1.30 1.15 21.00 -1.22 0.092 0.513634007548 1
###### 000002.png 3 -0.74 101.53 145.11 133.74 231.87 1.76 0.66 0.49 -14.20 1.46 20.70 -1.34 0.055 0.705416749366 1
###### img_name-cls-alpha-x1-y1-x2-y2-h-w-l-t1-t2-t3-ry-score-"1"
