import os
import lgsvl
import sys
import time
import math
import random
import pickle
import inspect
import pprint
import sympy
import util
from sympy import Point3D, Line3D, Segment3D, Point2D, Line2D, Segment2D

#Determine who is liable based on safety violation constraints in global coordinate system of LGSVL.

def isHitEdge(ego, sim, init_degree):
	# init_degree = ego.state.rotation.y
	lane_center = sim.map_point_on_lane(ego.state.transform.position)
	
	ego_x = ego.state.transform.position.x 
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	ego_point = Point3D(ego_x, ego_y, ego_z)

	mp_x = lane_center.position.x
	mp_y = lane_center.position.y
	mp_z = lane_center.position.z
	mp_point = Point3D(mp_x, mp_y, mp_z)

	# x1, y1, z1 = 160.809997558594, 10.1931667327881, 8.11004638671875
	# x_e_1, y_e_1, z_e_1 = 101.646751403809, 10.1278858184814, 8.18318462371826
	# x6, y6, z6 = 24.9999961853027, 10.1931667327881, -3.77267646789551
	# x_e_6, y_e_6, z_e_6 = 84.163330078125, 10.1277523040771, -3.77213048934937
	# l1 = Line3D(Point3D(x1, y1, z1), Point3D(x_e_1, y_e_1, z_e_1))
	# l6 = Line3D(Point3D(x6, y6, z6), Point3D(x_e_6, y_e_6, z_e_6))
	
	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	ego_size_z = (diagnal_length / 2.0)*math.sin(math.radians(rotate_degree))

	if(l6.distance(mp_point) <= 1):
		lane_bound = mp_z - 2.2
		if (ego.state.transform.position.z - ego_size_z <= lane_bound):
			util.print_debug("--- Cross the boundary --- ")
			return True
	if(l1.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.2
		if (ego.state.transform.position.z + ego_size_z >= lane_bound):
			util.print_debug("--- Cross the boundary --- ")
			return True
	return False

def isHitYellowLine(ego, sim, init_degree):

	lane_center = sim.map_point_on_lane(ego.state.transform.position)
	
	ego_x = ego.state.transform.position.x 
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	ego_point = Point3D(ego_x, ego_y, ego_z)

	mp_x = lane_center.position.x
	mp_y = lane_center.position.y
	mp_z = lane_center.position.z
	mp_point = Point3D(mp_x, mp_y, mp_z)

	x1, y1, z1 = 145.000030517578, 10.1931667327881, 4.20298147201538
	x_e_1, y_e_1, z_e_1 = 132.136016845703, 10.1280860900879, 4.20766830444336
	x6, y6, z6 = 24.9999923706055, 10.1931667327881, 0.026848778128624
	x_e_6, y_e_6, z_e_6 = 82.6629028320313, 10.1278924942017, 0.0420729108154774
	l1 = Line3D(Point3D(x1, y1, z1), Point3D(x_e_1, y_e_1, z_e_1))
	l6 = Line3D(Point3D(x6, y6, z6), Point3D(x_e_6, y_e_6, z_e_6))
	
	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	ego_size_z = (diagnal_length / 2.0)*math.sin(math.radians(rotate_degree))

	if(l1.distance(mp_point) <= 1):
		lane_bound = mp_z - 2.2
		if (ego.state.transform.position.z - ego_size_z <= lane_bound):
			util.print_debug(" --- Cross the yellow line")
			return True
	if(l6.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.2
		if (ego.state.transform.position.z + ego_size_z >= lane_bound):
			util.print_debug(" --- Cross the yellow line")
			return True
	return False
	

def isCrossedLine(ego, sim, init_degree):

	lines = []
	#start x, y, z
	# x1, y1, z1 = 1651.42358398438, 87.6020050048828, -601.2451171875
	# x2, y2, z2 = 1651.45263671875, 87.6022644042969, -605.143371582031
	# x3, y3, z3 = 1651.287109375, 87.6029052734375, -609.455932617188
	# x4, y4, z4 = 1651.10766601563, 87.6033096313477, -613.207153320313
	# x5, y5, z5 = 1651.53503417969, 87.6033706665039, -617.412658691406
	# x6, y6, z6 = 1651.48498535156, 87.6036529541016, -621.334228515625

	# #end x, y, z
	# x_e_1, y_e_1, z_e_1 = 1561.34594726563, 87.7077407836914, -608.11669921875   
	# x_e_2, y_e_2, z_e_2 = 1561.48486328125, 87.7079086303711, -612.000671386719
	# x_e_3, y_e_3, z_e_3 = 1560.76818847656, 87.7091827392578, -616.361694335938
	# x_e_4, y_e_4, z_e_4 = 1560.44677734375, 87.7097396850586, -620.123901367188
	# x_e_5, y_e_5, z_e_5 = 1560.607421875, 87.7100067138672, -624.344970703125
	# x_e_6, y_e_6, z_e_6 = 1560.93957519531, 87.7099990844727, -628.242858886719
		
	# l1 = Line3D(Point3D(x1, y1, z1), Point3D(x_e_1, y_e_1, z_e_1))
	# l2 = Line3D(Point3D(x2, y2, z2), Point3D(x_e_2, y_e_2, z_e_2))
	# l3 = Line3D(Point3D(x3, y3, z3), Point3D(x_e_3, y_e_3, z_e_3))
	# l4 = Line3D(Point3D(x4, y4, z4), Point3D(x_e_4, y_e_4, z_e_4))
	# l5 = Line3D(Point3D(x5, y5, z5), Point3D(x_e_5, y_e_5, z_e_5))
	# l6 = Line3D(Point3D(x6, y6, z6), Point3D(x_e_6, y_e_6, z_e_6))
	# lines.append(l1)
	# lines.append(l2)
	# lines.append(l3)
	# lines.append(l4)
	# lines.append(l5)
	# lines.append(l6)

	lane_center = sim.map_point_on_lane(ego.state.transform.position)
	right_z = lane_center.position.z - 2.34
	left_z = lane_center.position.z + 2.34
									
	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	ego_size_z = (diagnal_length / 2.0)*math.sin(math.radians(rotate_degree))
	if not(ego.state.transform.position.z+ego_size_z < left_z and ego.state.transform.position.z-ego_size_z > right_z): 
		print(" === Ego cross line === ")
		return True
	else:
		return False

def debugPos(ego, npc):
	egoRotation = ego.state.rotation.y
	npcRotation = npc.state.rotation.y
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	npc_x = npc.state.transform.position.x
	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	print(" ^^^^^^^^ Ego Rotation: " + str(egoRotation) + ", NPC rotation: " + str(npcRotation) + " ^^^^^^^")
	print("Ego: " + str(ego_x) + ", " + str(ego_y) + ", " + str(ego_z))
	print("NPC: " + str(npc_x) + ", " + str(npc_y) + ", " + str(npc_z))

def findDistance(ego, npc):
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	npc_x = npc.state.transform.position.x
	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	dis = math.pow(npc_x - ego_x , 2) + math.pow(npc_y - ego_y, 2) + math.pow(npc_z - ego_z, 2) 
	dis = math.sqrt(dis)
	return dis

def isEgoFault(ego, npc, sim, init_degree):	
	if npc is None:
        	return True

	isCrossed = isCrossedLine(ego, sim, init_degree)
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	npc_x = npc.state.transform.position.x
	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	debugPos(ego, npc)

	if (isHitYellowLine(ego, sim, init_degree)):
		return False

	# Longitudinal hit
	if isCrossed == True:
		if ego_x + 4 < npc_x and (npc.state.rotation.y < 271 and npc.state.rotation.y > 269):		

			print(" --- Ego cross , NPC is behind, NPC FAULT --- ")
			return False
		else:
			print(" --- Ego cross, side collision or front colision to NPC, EGO FAULT --- ")
			return True
	else:
		#if ego_x - 4.3 > npc_x and npc_z > ego_z - 2 and npc_z < ego_z + 2:
		if ego_x - 4 > npc_x and (npc.state.rotation.y > 271 or npc.state.rotation.y < 269):
			# NPC is in front
			print(" --- Ego stays in line , NPC is in front, EGO FAULT --- ")
			return True
		else:
			print(" --- Ego stays in line, side or rear collision to EGO, NPC FAULT --- ")
			return False
		

