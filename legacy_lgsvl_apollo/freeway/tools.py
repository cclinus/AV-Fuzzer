#! /usr/bin/python3 

import os, math
import pickle
import sys
from Chromosome import Chromosome   
from os import listdir
from os.path import isfile, join
from os import walk   
import numpy as np                                                                              

######################################################################################3

def getSimilarityBetweenNpcs(npc1, npc2):
    accumD = 0.0
    horD1 = 0.0
    horD2 = 0.0
    v1 = 0.0
    v2 = 0.0

    x = []
    y = []
    x1 = []
    y1 = []

    for i in range(len(npc1)):
        v1 += npc1[i][0]
        v2 += npc2[i][0]
        a1 = npc1[i][1] 
        a2 = npc2[i][1]

        if a1 == 1:
            horD1 += -34.0
        elif a1 == 2:
            horD1 += 34.0
        if a2 == 1:
            horD2 += -34.0
        elif a2 == 2:
            horD2 += 34.0
        x.append(horD1)
        y.append(v1)
        x1.append(horD2)
        y1.append(v2)

        curED = math.sqrt(math.pow(v1 - v2, 2) + math.pow(horD1 - horD2, 2))
        accumD += curED

    return accumD 
######################################################################################

def getSimilaritybetweenScenarios(scenario1, scenario2):

    npcSize = len(scenario1)
    timeSize = len(scenario1[0])

    scenarioNpc = 0.0

    for i in range(npcSize):
        npc1 = scenario1[i]
        npc2 = scenario2[i]
        npcSimi = getSimilarityBetweenNpcs(npc1, npc2)
        scenarioNpc += npcSimi

    return scenarioNpc/npcSize

###################################################################################### 

    
