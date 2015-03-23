/*
 * Main.cpp
 *
 *  Created on: Sep 20, 2012
 *      Author: peter
 */
#include "Genome.h"
#include "Population.h"
#include "NeuralNetwork.h"
#include "Parameters.h"
#include <iostream>
#include <vector>

using namespace NEAT;

double evaluate(Genome& genome) {
	NeuralNetwork net;
    genome.BuildPhenotype(net);

    double error = 0;

    for(int i=0; i<4; i++) {
    	std::vector<double> inputs;
    	inputs.push_back((i<2) ? 1 : 0);
    	inputs.push_back((i%2==0) ? 1 : 0);
    	inputs.push_back(1);
    	double output = ((i == 1) || (i == 2)) ? 1 : 0;
        net.Flush();
        net.Input(inputs);
        for(int t=0; t<3; t++) {
        	net.Activate();
        }
        double o = net.Output()[0];
        error += fabs(output - o);
    }
    return ((4 - error) * (4 - error));
}

int main()
{
	std::cout<<"Starting\n";
	Parameters params;
	params.PopulationSize = 150;
	params.WeightDiffCoeff = 0.4;
	params.DynamicCompatibility = true;
	params.CompatTreshold = 3.0;
	params.YoungAgeTreshold = 15;
	params.SpeciesMaxStagnation = 15;
	params.OldAgeTreshold = 35;
	params.MinSpecies = 5;
	params.MaxSpecies = 25;
	params.RouletteWheelSelection = false;
	params.RecurrentProb = 0;
	params.OverallMutationRate = 0.33;

	params.MutateWeightsProb = 0.90;

	params.WeightMutationMaxPower = 2.5;
	params.WeightReplacementMaxPower = 5.0;
	params.MutateWeightsSevereProb = 0.5;
	params.WeightMutationRate = 0.25;

	params.MaxWeight = 20;

	params.MutateAddNeuronProb = 0.03;
	params.MutateAddLinkProb = 0.05;
	params.MutateRemLinkProb = 0.00;

	params.MinActivationA  = 4.9;
	params.MaxActivationA  = 4.9;

	params.ActivationFunction_SignedSigmoid_Prob = 0.0;
	params.ActivationFunction_UnsignedSigmoid_Prob = 1.0;
	params.ActivationFunction_Tanh_Prob = 0.0;
	params.ActivationFunction_SignedStep_Prob = 0.0;
    Genome g(0, 3, 10, 1, false, UNSIGNED_SIGMOID,UNSIGNED_SIGMOID, 0, params);
	Population pop(g, params, true, 1.0, 0);

    for(int generations = 0; generations<1000; generations++) {
    	double best = 0;
    	for(unsigned int i=0; i < pop.m_Species.size(); i++) {
			for(unsigned int j=0; j < pop.m_Species[i].m_Individuals.size();
					j++) {

				double f = evaluate(pop.m_Species[i].m_Individuals[j]);
				pop.m_Species[i].m_Individuals[j].SetFitness(f);
				pop.m_Species[i].m_Individuals[j].SetEvaluated();
				if (f > best) {
					best = f;
				}
			}
    	}
    	std::cout << "Generation " << generations << ", best fit " << best << "\n";
    	pop.Epoch();
		if (best > 15.5) {
			std::cout << "Solved in " << generations << " generations\n";
			break;
		}
    }


    return 0;
}

