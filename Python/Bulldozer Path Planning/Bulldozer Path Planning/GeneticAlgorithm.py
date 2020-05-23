class GeneticAlgorithm:
    def __init__(self,population_size,crossover_prob,mutation_prob):
        self._population_size = population_size
        self._crossover_prob = crossover_prob
        self._mutation_prob = mutation_prob
        self._population = self.initalisePopulation()
