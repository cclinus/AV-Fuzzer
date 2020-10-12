import GeneticAlgorithm


####################################################################################
bounds = [[0, 67], [0, 3]]                      #[[speed range], [actions]]
mutationProb = 0.4                             # mutation rate
crossoverProb = 0.4                             # crossover rate
popSize = 4
numOfNpc = 2
numOfTimeSlice = 5
maxGen = 100
####################################################################################

ge = GeneticAlgorithm.GeneticAlgorithm(bounds, mutationProb, crossoverProb, popSize, numOfNpc, numOfTimeSlice, maxGen)
#ge.set_checkpoint('GaCheckpoints/last_gen.obj')
ge.ga()
