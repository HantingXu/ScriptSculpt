#pragma once

#include <iostream>
#include "sceneStruct.h"
#include "LetterAlignment.h"
#include "openGA.hpp"

class GASolver {
private:
	LetterAlignment* letterAlignment;
public:
	GASolver(LetterAlignment* letterAlign);
	void initGenes(GASolution& alignmentSol, const std::function<double(void)>& rnd01);
	bool evalSolution(const GASolution& alignmentSol, GACost& cost);
	GASolution mutate(const GASolution& X_base, const std::function<double(void)>& rnd01, double shrink_scale);
	GASolution crossover(const GASolution& X1, const GASolution& X2, const std::function<double(void)>& rnd01);
	double calculateSOTotalFitness(const GA_Type::thisChromosomeType& X);
	void SOReportGeneration(int generation_number, const EA::GenerationType<GASolution, GACost>& last_generation, const GASolution& best_genes);
};
