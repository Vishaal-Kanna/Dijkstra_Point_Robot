#!/usr/bin/env python

"""
ENPM661: Project 2

Vishaal Kanna Sivakumar (vishaal@terpmail.umd.edu)
M.Eng. Student, Robotics
University of Maryland, College Park

"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import cv2

def map_gen():
	map = np.zeros((250,400))
	for y in range(1,map.shape[0]+1):
		for x in range(1, map.shape[1]+1):
			if x>=165 and x<=235 and ((map.shape[0]-140-map.shape[0]+120)/(200-235))*(x-235)+map.shape[0]-120<=y and ((map.shape[0]-140-map.shape[0]+120)/(200-165))*(x-165)+map.shape[0]-120<=y and ((map.shape[0]-80-map.shape[0]+60)/(165-200))*(x-200)+map.shape[0]-60>=y and ((map.shape[0]-80-map.shape[0]+60)/(235-200))*(x-200)+map.shape[0]-60>=y:
				map[y-1][x-1]=1
			if ((map.shape[0]-210-map.shape[0]+185)/(115-36))*(x-36)+map.shape[0]-185<=y and ((map.shape[0]-100-map.shape[0]+185)/(105-36))*(x-36)+map.shape[0]-185>=y and (((map.shape[0]-210-map.shape[0]+180)/(115-75))*(x-75)+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+100)/(75-105))*(x-105)+map.shape[0]-100<=y):
				map[y-1][x-1]=1
			if (x-300)**2+(y-map.shape[0]+185)**2<=40**2:
				map[y-1][x-1]=1
	return map

def Move_dir(map, action, state, index):
	x,y = state[0], state[1]
	cost=0
	if x + action[0]>=0 and x + action[0]<400 and y + action[1]>=0 and y + action[1]<250:
		if map[y+action[1]][x+action[0]] == 0:
			x = x + action[0]
			y = y + action[1]
			cost = (action[0]**2 + action[1]**2)**0.5
			map[y][x]=0.5
			index=index+1
			return map, x, y, cost, index, 1
	return map, x, y, cost, index, 0

def New_node(map, OpenList, ClosedList, Goal_node_x, Goal_node_y, index):
	M = heapq.heappop(OpenList)
	ClosedList.append(M)
	Action_space = ((5, 0), (-5, 0), (0, 5), (0, -5), (5, 5), (-5, 5), (5, -5), (-5, -5))
	for i in range(0,8):
		map, new_node_x, new_node_y, C2C, index, t = Move_dir(map, Action_space[i], M[3],index)
		if t==1:
			new_node = (M[0]+C2C, index, M[1], (new_node_x, new_node_y))
			heapq.heappush(OpenList, new_node)
			if new_node_x == Goal_node_x and new_node_y == Goal_node_y:
				return map, OpenList, ClosedList, 1
	return map, OpenList, ClosedList, 0

def main():
	map = map_gen()
	#plt.imshow(map)
	# creating tuple with cost to come, index, parent node index=0 and coordinate values (x,y)
	start_node = (0, 0, -1, (0, 0))
	goal_node_x, goal_node_y = 395, 5
	OpenList = [start_node]
	ClosedList = []
	heapq.heapify(OpenList)
	goal_reached = 0
	map_color_r = np.zeros((250, 400))
	map_color_b = np.zeros((250, 400))
	map_color_g = np.zeros((250, 400))
	map_color1 = np.zeros((250, 400,3))

	while len(OpenList) and not goal_reached:
		map, OpenList, ClosedList, goal_reached = New_node(map, OpenList, ClosedList, goal_node_x, goal_node_y,0)
		map_color = map
		map_color_r[map_color == 0] = 1
		map_color_b[map_color == 0.5] = 1
		map_color_g[map_color == 1] = 1
		map_color1[:, :, 0] = map_color_r * 255
		map_color1[:, :, 1] = map_color_b * 255
		map_color1[:, :, 2] = map_color_g * 255
		cv2.imshow("Map",map_color1)
		cv2.waitKey(1)

if __name__ == '__main__':
	main()