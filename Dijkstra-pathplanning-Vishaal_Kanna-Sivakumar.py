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
	c=5
	for y in range(1,map.shape[0]+1):
		for x in range(1, map.shape[1]+1):
			if x>=165-c and x<=235+c and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(235+c/1.414)))*(x-(235+c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(165-c/1.414)))*(x-(165-c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((165-c/1.414)-(200)))*(x-(200))+map.shape[0]-(60-c)>=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((235+c/1.414)-200))*(x-200)+map.shape[0]-(60-c)>=y:
				map[y-1][x-1]=10000
			if ((map.shape[0]-(210+c/1.414)-map.shape[0]+(185))/((115+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185<=y and ((map.shape[0]-(100-c/1.414)-map.shape[0]+185)/((105+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185>=y and ((map.shape[0]-(210+c/1.414)-map.shape[0]+180)/((115+c/1.414)-(75+c))*(x-(75+c))+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+(100-c/1.414))/((75+c)-(105+c/1.414)))*(x-(105+c/1.414))+map.shape[0]-(100-c/1.414)<=y):
				map[y-1][x-1]=10000
			if (x-300)**2+(y-map.shape[0]+185)**2<=(40+c)**2:
				map[y-1][x-1]=10000
			#if x>250 and x<275:
			#	map[y-1][x-1]=1
			if x>0 and x<=c:
				map[y - 1][x - 1] = 10000
			if x>400-c and x<=400:
				map[y - 1][x - 1] = 10000
			if y>0 and y<=c:
				map[y - 1][x - 1] = 10000
			if y>250-c and y<=250:
				map[y - 1][x - 1] = 10000
	return map

def Move_dir(map, action, state, index,Parent_cost):
	x,y = state[0], state[1]
	cost=0
	if x + action[0]>=0 and x + action[0]<400 and y + action[1]>=0 and y + action[1]<250:
		if map[y+action[1]+action[1]*0][x+action[0]+action[0]*0] == 0:
			x = x + action[0]
			y = y + action[1]
			cost = Parent_cost+(action[0]**2 + action[1]**2)**0.5
			map[y-action[1]][x-action[0]]=-1
			index=index+1
			return map, x, y, cost, index, 1
	return map, x, y, cost, index, 0

def New_node(map, OpenList, ClosedList, Goal_node_x, Goal_node_y, index):
	M = heapq.heappop(OpenList)
	ClosedList.append(M)
	if M[3][0] == Goal_node_x and M[3][1] == Goal_node_y:
		return map, OpenList, ClosedList, index, 1
	a=1
	Action_space = ((a, 0), (-a, 0), (0, a), (0, -a), (a, a), (-a, a), (a, -a), (-a, -a))
	for i in range(0,8):
		map, new_node_x, new_node_y, C2C, index, t = Move_dir(map, Action_space[i], M[3], index, M[0])

		count = 0
		if t==1:
			#Using the map directly to check if the node is in the ClosedList by setting the map matrix node to -1 
			#for j in range(0,len(ClosedList)):
			#	if new_node_x == ClosedList[j][3][0] and new_node_y == ClosedList[j][3][1]:
			#		count=count+1
			#		break
			for j in range(0,len(OpenList)):
				if (new_node_x == OpenList[j][3][0] and new_node_y == OpenList[j][3][1]):
					count=count+1
					if C2C < OpenList[j][0]:
						node = (C2C, OpenList[j][1], M[1], (new_node_x, new_node_y))
						heapq.heapreplace(OpenList,node)
					break

			if count == 0:
				new_node = (C2C, index, M[1], (new_node_x, new_node_y))
				heapq.heappush(OpenList, new_node)

	return map, OpenList, ClosedList, index, 0

def generate_path(lst, ClosedList, idx):
	for i in range(0, len(ClosedList)):
		if ClosedList[i][1] == idx:
			idx = i
			break
	if ClosedList[idx][2] ==-1:
		lst.append(ClosedList[idx][3])
		return lst
	else:
		lst.append(ClosedList[idx][3])
		generate_path(lst, ClosedList, ClosedList[idx][2])

def main():
	map = map_gen()
	flag=1

	while(flag):
		print('Enter the Starting Coordinates')
		start_node_x = int(input())
		start_node_y = int(input())
		print('Enter the Goal Coordinates')
		goal_node_x = int(input())
		goal_node_y = int(input())
		if start_node_x < 0 or start_node_y < 0 or start_node_x > 399 or start_node_y > 249 or goal_node_x < 0 or goal_node_y < 0 or goal_node_x > 399 or goal_node_y > 249:
			print("Coordinates entered are outside the map")
		elif start_node_x==goal_node_x and start_node_y==goal_node_y:
			print('Starting node and Goal node are same. Please enter different sets of matrices for each.')
		elif map[250-start_node_y][start_node_x] == 10000 or map[250-goal_node_y][goal_node_x] == 10000:
			print("Coordinates are inside the Obstacle")
		else:
			flag=0

	print('Starting Node: ', (start_node_x, start_node_y))
	print('Goal Node: ', (goal_node_x, goal_node_y))

	start_node = (0, 0, -1, (start_node_x, start_node_y))
	OpenList = [start_node]
	ClosedList = []
	heapq.heapify(OpenList)
	goal_reached = 0
	map_color_r = np.zeros((250, 400))
	map_color_b = np.zeros((250, 400))
	map_color_g = np.zeros((250, 400))
	map_color1 = np.zeros((250, 400,3))
	index = 0

	while len(OpenList) and not goal_reached:
		map, OpenList, ClosedList, index, goal_reached = New_node(map, OpenList, ClosedList, goal_node_x, goal_node_y,index)
		if len(OpenList) ==0 and not goal_reached:
			print('Solution Not Found')
			quit()

	lst = []
	lst.append(ClosedList[len(ClosedList) - 1][3])
	generate_path(lst, ClosedList, ClosedList[len(ClosedList) - 1][2])

	print('Map explored and optimal path found.')

	map = map_gen()

	for	i in range(0,len(ClosedList)):
		map[ClosedList[i][3][1]][ClosedList[i][3][0]]=0.7
		map_color = map
		map_color_r[map_color == 0] = 1
		map_color_b[map_color == 0.7] = 1
		map_color_g[map_color == 10000] = 1
		map_color1[:, :, 0] = map_color_r * 255
		map_color1[:, :, 1] = map_color_b * 255
		map_color1[:, :, 2] = map_color_g * 255
		cv2.imshow("Map", map_color1)
		cv2.waitKey(1)

	map_color_r1 = np.zeros((250, 400))
	map_color_b1 = np.zeros((250, 400))
	map_color_g1 = np.zeros((250, 400))
	map_color2 = np.zeros((250, 400,3))

	for i in range(0, len(lst)):
		map[lst[len(lst)-i-1][1]][lst[len(lst)-i-1 ][0]] = 0.2
		map_color = map
		map_color_r1[map_color == 0] = 1
		map_color_b1[map_color == 0.2] = 1
		map_color_g1[map_color == 10000] = 1
		map_color2[:, :, 0] = map_color_r1 * 0
		map_color2[:, :, 1] = map_color_b1 * 255
		map_color2[:, :, 2] = map_color_g1 * 255
		cv2.imshow("Map", map_color2)
		cv2.waitKey(20)

if __name__ == '__main__':
	main()

