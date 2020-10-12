
import math
import random
import pprint
import pickle
import sys
import os
from datetime import datetime
import util

class Chromosome:
    def __init__(self, bounds, NPC_size, time_size):
        self.y = 0
        self.scenario = [[[] for i in range(time_size)] for j in range(NPC_size)] # This scenario
        self.bounds = bounds
        self.code_x1_length = NPC_size 
        self.code_x2_length = time_size
        self.timeoutTime = 300 # in seconds, timeout timer for simulator execution per each scenario simulation

    def fix_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = (self.bounds[0][0] + self.bounds[0][1]) / float(2) #random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = 3  # Keep straight #random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

    def rand_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

    def foo_obj_func(self):
        speedSum = 0
        for npc in self.scenario:
            for nt in npc:
                speedSum += nt[0]
                speedSum += nt[0] * 34
        return speedSum

    def decoding(self):
        fitness_score = 0

        # Send scenario object to simulation script
        s_f = open('scenario.obj', 'wb')
        pickle.dump(self.scenario, s_f)
        s_f.truncate() 
        s_f.close()       
        
        for x in range(0, 100):	

            if os.path.isfile('result.obj') == True:
                os.remove("result.obj")

            os.system("python3 simulation.py scenario.obj result.obj")
            resultObj = None

            # Read fitness score
            if os.path.isfile('result.obj') == True:
                f_f = open('result.obj', 'rb')
                resultObj = pickle.load(f_f)
                f_f.close()

            if resultObj != None and resultObj['fitness'] != '':
                return resultObj
                break
            else:
                util.print_debug(" ***** " + str(x) + "th/10 trial: Fail to get fitness, try again ***** ")

        return None

    # Get fitness score of the scenario
    def func(self, gen=None, lisFlag=False):

        resultObj = self.decoding()
        self.y = float(resultObj['fitness'])
        if resultObj['fault'] == 'ego':
                # An accident        
                util.print_debug(" ***** Found an accident where ego is at fault ***** ")
                # Dump the scenario where causes the accident
                if os.path.exists('AccidentScenario') == False:
                    os.mkdir('AccidentScenario')
                now = datetime.now()
                date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
                ckName = 'AccidentScenario/accident-gen' + str(gen) + '-' + date_time
                if lisFlag == True:
                    ckName = ckName + "-LIS"
                a_f = open(ckName, 'wb')
                pickle.dump(self, a_f)
                a_f.truncate() 
                a_f.close()
            
# Test
if __name__ == '__main__':
    a = [[10, 30], [0, 2]]
    chromosome = Chromosome(a, 5, 10, 10)
    pprint.pprint(chromosome.scene)

