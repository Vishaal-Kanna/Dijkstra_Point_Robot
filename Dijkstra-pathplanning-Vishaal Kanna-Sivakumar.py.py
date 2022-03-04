#!/usr/bin/env python

"""
ENPM661: Project 2

Vishaal Kanna Sivakumar (vishaal@terpmail.umd.edu)
M.Eng. Student, Robotics
University of Maryland, College Park

"""

import numpy as np
import matplotlib.pyplot as plt

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

def main():
	map = map_gen()
	plt.imshow(map)
	plt.show()

if __name__ == '__main__':
	main()