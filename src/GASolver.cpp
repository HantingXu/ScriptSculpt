#include "GASolver.h"


GASolver::GASolver(LetterAlignment* letterAlign)
{
	letterAlignment = letterAlign;
}

void GASolver::initGenes(GASolution& alignSol, const std::function<double(void)>& rnd01)
{
	letterAlignment->setGASolution(alignSol);
}

bool GASolver::evalSolution(const GASolution& alignSol, GACost& cost)
{
	letterAlignment->setLetters(alignSol);
	cost.objective1 = letterAlignment->refinedAlignment();
	return true;
}

GASolution GASolver::mutate(const GASolution& X_base, const std::function<double(void)>& rnd01, double shrink_scale)
{
	GASolution X_new;
	const double mu = 0.2 * shrink_scale; // mutation radius (adjustable)
	bool in_range;
	do {
		in_range = true;
		X_new = X_base;
		for (int i = 0; i < 15; i++)
		{
			X_new.var[i] += mu * (rnd01() - rnd01());
			in_range = in_range && (X_new.var[i] >= 0.0 && X_new.var[i] < 10.0);
		}
	} while (!in_range);
	return X_new;
}

GASolution GASolver::crossover(const GASolution& X1, const GASolution& X2, const std::function<double(void)>& rnd01)
{
	GASolution X_new;
	double r;

	for (int i = 0; i < 15; i++)
	{
		r = rnd01();
		X_new.var[i] = r * X1.var[i] + (1.0 - r) * X2.var[i];
	}
	return X_new;
}

double GASolver::calculateSOTotalFitness(const GA_Type::thisChromosomeType& X)
{
	float final_cost = 0.0f;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

void GASolver::SOReportGeneration(int generation_number, const EA::GenerationType<GASolution, GACost>& last_generation, const GASolution& best_genes)
{
}