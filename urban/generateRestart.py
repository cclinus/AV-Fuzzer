#! /usr/bin/python3
import os
import sys
import copy
import tools
import collections
import pickle
from Chromosome import Chromosome
from os import listdir
from os.path import isfile, join
from os import walk   

def getAllCheckpoints(ck_path):
	onlyfiles = [f for f in listdir(ck_path)]
	prevPopPool = []

	for i in range(len(onlyfiles)):
		with open(ck_path+'/'+onlyfiles[i], "rb") as f:
			if "generation" not in onlyfiles[i]:
				continue
			try:
				prevPop = pickle.load(f)
				prevPopPool.append(prevPop)
			except Exception:
				pass

	return prevPopPool

def generateRestart(ck_path, scenarioNum, bounds):
	
	prevPopPool = getAllCheckpoints(ck_path)			 
	
	newPopCandiate = []
	newScenarioList = []
	popSize = len(prevPopPool[0])
	npcSize = len(prevPopPool[0][0].scenario)
	scenarioSize = len(prevPopPool[0])
	popPoolSize = len(prevPopPool)
	dictScenario = {}

	for i in range(scenarioNum):
		chromosome = Chromosome(bounds, npcSize, len(prevPopPool[0][0].scenario[0]))
		chromosome.rand_init()
		newPopCandiate.append(chromosome)

	# Go through every scenario

	for i in range(scenarioNum):
		similarity = 0;
		for j in range(popPoolSize):
			simiPop = 0;
			for k in range(scenarioSize):
				scenario1 = newPopCandiate[i].scenario
				scenario2 = prevPopPool[j][k].scenario
				simi= tools.getSimilaritybetweenScenarios(scenario1, scenario2)
				simiPop += simi

			simiPop /= scenarioSize
			similarity += simiPop
		similarity /= popPoolSize
		dictScenario[i] = similarity

	sorted_x = sorted(dictScenario.items(), key=lambda kv: kv[1], reverse=True)
	sorted_dict = collections.OrderedDict(sorted_x)

	index = sorted_dict.keys()

	j = 0

	for i in index:
		if j == popSize:
			break
		newPopCandiate[i].func()
		newScenarioList.append(newPopCandiate[i])
		j += 1

	return newScenarioList

def IsDifferenceEnough(ck_path, afterScenario, beforeScenario):
	prevPopPool = getAllCheckpoints(ck_path)
	popPoolSize = len(prevPopPool)
	scenarioSize = len(prevPopPool[0])

	similarityBefore = 0
	similarityAfter = 0

	for j in range(popPoolSize):
		simiPopBefore = 0
		simiPopAfter = 0
		for k in range(scenarioSize):
			referScenario = prevPopPool[j][k].scenario
			simiBefore = tools.getSimilaritybetweenScenarios(beforeScenario.scenario, referScenario)
			simiAfter = tools.getSimilaritybetweenScenarios(afterScenario.scenario, referScenario)

			simiPopBefore += simiBefore
			simiPopAfter += simiAfter

		simiPopBefore /= scenarioSize
		similarityBefore += simiPopBefore
		simiPopAfter /= scenarioSize
		similarityAfter += simiPopAfter

	similarityBefore /= popPoolSize
	similarityAfter /= popPoolSize

	return similarityAfter < similarityBefore 

def getSimularityOfScenarioVsPrevPop(scenario, prePopPool):
	
	similarity = 0
	for i in prePopPool:
		popSimilarity = 0
		for j in i:
			simi = tools.getSimilaritybetweenScenarios(j.scenario, scenario.scenario)
			popSimilarity += simi
		popSimilarity /= len(i)
		similarity += popSimilarity
	similarity /= len(prePopPool)

	return similarity	

if __name__ == '__main__':
    prevPopPool = getAllCheckpoints('GaCheckpoints')
    npcSize = len(prevPopPool[0][0].scenario)
    chromosome1 = Chromosome([[0, 34], [0, 3]], npcSize, len(prevPopPool[0][0].scenario[0]))
    chromosome1.rand_init()
    chromosome2 = Chromosome([[0, 34], [0, 3]], npcSize, len(prevPopPool[0][0].scenario[0]))
    chromosome2.rand_init()
    checkIfRemutataion('GaCheckpoints', chromosome1, chromosome2)

