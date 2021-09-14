#!/usr/bin/env python
import math
import rospy
import numpy
import geometry_msgs.msg
import yaml
import os
def generate_waypoints():
    ct=0
    way_pt=geometry_msgs.msg.Pose()
    box_dimension=[0.2,0.2,0.2]
    box_centre=[0.5,0,0.5]
    r=round(max(box_dimension)/2,1)
    no_intervals=6
    height=[x*r/(no_intervals-1) for x in range(no_intervals)]
    points={}
    points["point_lt"]={}
    for i in height:
        rad_at_h= math.sqrt(r**2 - i**2)
        pt_z=round(box_centre[-1]+i,3)
        print(pt_z)
        no_points=int(-7*i/r+8)
        angle=[360*x/no_points for x in range(no_points)]
        for j in angle:
            pt_x=box_centre[0]+rad_at_h*math.cos(math.radians(j))
            pt_y=box_centre[1]+rad_at_h*math.sin(math.radians(j))
            points["point_lt"][f'locations_{ct}']={'x':pt_x,'y':pt_y,'z':pt_z}
            ct+=1
    
    if os.path.isfile(r'/home/shaswatg/UofM_ws/src/kuka_d435/config/way_point.yaml'):
        f = open(r'/home/shaswatg/UofM_ws/src/kuka_d435/config/way_point.yaml',"r+")
        f.truncate(0)
        f.close()

    with open(r'/home/shaswatg/UofM_ws/src/kuka_d435/config/way_point.yaml', 'w') as file:
        documents = yaml.dump(points, file)
    
    


if __name__=="__main__":
    generate_waypoints()