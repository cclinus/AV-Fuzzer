import carla
import random
import time
import math
import sys
import os
import yaml
from datetime import datetime


def is_crossed_line(vehicle):

    y = vehicle.get_location().y
    return abs(y) < 6.5


def is_ego_fault(ego, npc):
    if not npc:
        return True

    ego_loc = ego.get_location()
    npc_loc = npc.get_location()

    #ego_yaw = ego.get_transform().rotation.yaw % 360
    npc_yaw = npc.get_transform().rotation.yaw % 360

    if is_crossed_line(ego):
        if ego_loc.x  > npc_loc.x and  (npc_yaw > 25 or npc_yaw < 345):
            print("Ego crossed but NPC is behind — NPC FAULT")
            return False
        else:
            print("Ego crossed and hit side/rear of NPC — EGO FAULT")
            return True
    else:
        if ego_loc.x  < npc_loc.x and (npc_yaw < 25 or npc_yaw > 345):
            print(npc_yaw)
            print("Ego stayed but NPC was in front — EGO FAULT")
            return True
        else:
            print(npc_yaw)
            print("Ego stayed and NPC hit rear/side — NPC FAULT")
            return False


