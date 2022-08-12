// TIPEcppTest1.cpp : Ce fichier contient la fonction 'main'. L'exécution du programme commence et se termine à cet endroit.

#include <iostream>
#include <ilcplex/ilocplex.h>
#include <ilcplex/ilocplexi.h>
#include <chrono>
#include <fstream>
#include <vector>
#include <map>
#include "utilities.h"
#include "FileManager.h"
using namespace std;

//Here we define a Matrix decision variable
//IloNumVarArray is an 1-dimentionnal decision variable
typedef IloArray<IloNumVarArray> NumVar2D;

bool ContainsValue(map<int, bool> dict, bool Value)
{
	for (pair<const int, bool> i : dict)
	{
		if (i.second == Value)
		{
			return true;
		}
	}
	return false;
}
int GetNextValue(map<int, int> dict, int curValue)
{
	bool next = false;
	for (pair<const int, int> i : dict)
	{
		if (i.first == curValue)
		{
			next = true;
			continue;
		}
		if (next)
		{
			return i.first;
		}
	}
}

#pragma region LazyContraintsCallbacks

vector<vector<int>> CalculateSubtours(map<int, int> G1)
{
	vector<vector<int>> subToursList;
	// Visited[i] <- false ∀ i ∈ V1, i != 0
	map<int, bool> Visited;
	int i = -1;
	for (pair<const int, int> i0 : G1)
	{
		if (i == -1) i = i0.first;
		Visited[i0.first] = (i0.first == 0);
		cout << "Visited[" << i0.first << "]=" << (i0.first == 0) << endl;
	}
	// While there exists i ∈ V1\{0} with Visited[i] == false do
	while (ContainsValue(Visited, false))
	{
		// Gets the next value of i, to pursue the loop
		i = GetNextValue(G1, i);
		if (!Visited[i])
		{
			cout << "=====" << endl;
			//cout << "i: " << i << " Visited count: " << Visited.count(i) << " bool: " << Visited[i] << endl;
			// start <- i , S <- {i}
			int start = i;
			vector<int> S;
			S.push_back(i);
			// Visited[i] <- true , containsDepot <- false
			Visited[i] = true;
			bool containsDepot = false;
			// While the successor j of i(xij = 1) is not equal to start do
			int j = G1[i];
			while (j != start)
			{
				i = j;
				Visited[i] = true;
				S.push_back(i);
				if (i == 0)
				{
					containsDepot = true;
				}
				j = G1[i];
			}
			if (!containsDepot)
			{
				// subToursList <- subToursList U {S}
				// Some debug
				cout << "S: {";
				for (int g : S)
				{
					cout << g;
					if (g != S.back()) cout << ",";
				}
				cout << "}" << endl;
				subToursList.push_back(S);
			}
		}
	}
	return subToursList;
}
// Lazy constraint callback to enforce the capacity constraints.
// If used then the callback is invoked for every integer feasible solution
// CPLEX finds. For each location j it checks whether constraint
//    sum(c in C) supply[c][j] <= (|C| - 1) * used[j]
// is satisfied. If not then it adds the violated constraint as lazy constraint.
ILOLAZYCONSTRAINTCALLBACK2(LazyCallback, NumVar2D, Xmatrix, IloInt, n)
{
	map<int, int> G1;
	cout << "Lazy here!" << endl;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			const auto x_value = getValue(Xmatrix[i][j]);
			float value = getValue(Xmatrix[i][j]);
			if (value == 1)
			{
				cout << i << "->" << j << endl;
				//We assume that the path is unique (should be since it respects go-to and come-from constraints)
				G1[i] = j;
			}
		}
	}
	cout << "Calculating subtours..." << endl;
	vector<vector<int>> subToursList = CalculateSubtours(G1);
	cout << "Subtours calculation done!" << endl;
	// Here we go through each subtour to verify if it breaks the SECs
	for (vector<int> sub : subToursList)
	{
		if (2 <= sub.size() && sub.size() <= n - 1)
		{
			float sum = 0;
			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < n; j++)
				{
					sum += getValue(Xmatrix[i][j]);
				}
			}
			if (sum <= sub.size() - 1)
			{
				// Success! This subset does not break the SECs
				cout << "One subset is correct" << endl;
			}
			else
			{
				// SEC violated, thus we'll add a lazy constraint on this subset
				cout << "Adding lazy, SEC constraint " << sum << " <= " << int(sub.size() - 1) << " is violated" << endl;
				//ct3_4
				IloExpr expr3_4(getEnv());
				for (int i : sub)
				{
					for (int j : sub)
					{
						expr3_4 += Xmatrix[i][j];
					}
				}
				add(expr3_4 <= int(sub.size() - 1));
			}
		}
	}

	//IloInt const nbLocations = used.getSize();
	//IloInt const nbClients = supply.getSize();
	//for (IloInt j = 0; j < nbLocations; ++j)
	//{
	//	IloNum isUsed = getValue(used[j]);
	//	IloNum served = 0.0; // Number of clients currently served from j
	//	for (IloInt c = 0; c < nbClients; ++c)
	//		served += getValue(supply[c][j]);
	//	if (served > (nbClients - 1.0) * isUsed + EPS)
	//	{
	//		IloNumExpr sum = IloExpr(getEnv());
	//		for (IloInt c = 0; c < nbClients; ++c)
	//			sum += supply[c][j];
	//		sum -= (nbClients - 1) * used[j];
	//		cout << "Adding lazy capacity constraint " << sum << " <= 0" << endl;
	//		add(sum <= 0.0).end();
	//	}
	//}
	cout << "end lazy" << endl;
}
#pragma endregion

/*IloExtractable->IloRangeBase->IloRange*/
int main()
{
#pragma region DataSetup
	auto start_0 = chrono::high_resolution_clock::now();

	auto func_out = FileManager::read_file("C:\\Users\\marcb\\Downloads\\20170608T121355407419\\tbl_truck_travel_data_PG.csv");
	//Stops
	const int n = sqrt(func_out.size() - 1);
	cout << "n: " << n << endl;
	//Distances matrix, from 1..n
	float** Distance = new float* [n];
	Distance = FileManager::read_standardized_csv(func_out);

	/*vector<int> arg;
	for (int i = 0; i < n; i++)
	{
		arg.push_back(i);
	}
	vector<vector<int>> sub = utilities::subset(arg);
	utilities::print_subsets(sub);*/

	auto start_1 = chrono::high_resolution_clock::now();

#pragma endregion

	std::cout << "Hello World!\n";
	//Our environnement, basically everything
	IloEnv env;
	//Our mathematical model is defined here
	IloModel Model(env);

#pragma region DecisionVar

	//Our decision variable X[][] -> A Matrix
	//		   env, numberOfRows
	NumVar2D X(env, n);

	for (int i = 0; i < n; i++)
	{
		X[i] = IloNumVarArray(env, n, 0, IloInfinity, ILOBOOL);
	}

#pragma endregion

#pragma region ObjectiveFunction

	IloExpr expr0(env);

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			expr0 += Distance[i][j] * X[i][j];
		}
	}

	Model.add(IloMinimize(env, expr0));
	IloRange range();
#pragma endregion

#pragma region Constraints
	//i != j
	for (int i = 0; i < n; i++)
	{
		//X[i][i] == 0;
		IloExpr expr1(env);
		expr1 = X[i][i];
		Model.add(expr1 == 0);
	}

	//Go-to constraints
	for (int i = 0; i < n; i++)
	{
		//ct3_2
		IloExpr expr3_2(env);
		for (int j = 0; j < n; j++)
		{
			expr3_2 += X[i][j];
		}
		Model.add(expr3_2 == 1);
	}

	//Come-from constraints
	for (int j = 0; j < n; j++)
	{
		//ct3_3
		IloExpr expr3_3(env);
		for (int i = 0; i < n; i++)
		{
			expr3_3 += X[i][j];
		}
		Model.add(expr3_3 == 1);
	}

	//SECs
	/*for (int s = 0; s < sub.size(); s++)
	{
		if (2 <= sub[s].size() && sub[s].size() <= n - 1)
		{
			//ct3_4
			IloExpr expr3_4(env);
			for (int i : sub[s])
			{
				for (int j : sub[s])
				{
					expr3_4 += X[i][j];
				}
			}
			Model.add(expr3_4 <= int(sub[s].size() - 1));
		}
	}*/

	//Contrainte d'intégralité sur X[i][j]
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			//ct 3_5
			IloExpr expr3_5(env);
			expr3_5 = X[i][j];
			Model.add(expr3_5 == 0 || expr3_5 == 1);
		}
	}
#pragma endregion

#pragma region Solving

	// Solving
	IloCplex cplex(Model);
	// Export the model, useful for debugging
	cplex.exportModel("model.lp");
	// Set the output as stdout
	cplex.setOut(std::cout);

	// Disabling presolve for callbacks
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

	// Set the maximum number of threads to 1.
	// This instruction is redundant: If MIP control callbacks are registered,
	// then by default CPLEX uses 1 (one) thread only.
	// Note that the current example may not work properly if more than 1 threads
	// are used, because the callback functions modify shared global data.
	// We refer the user to the documentation to see how to deal with multi-thread
	// runs in presence of MIP control callbacks.
	cplex.setParam(IloCplex::Param::Threads, 1);

	// Turn on traditional search for use with control callbacks
	cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);

	//Registering callbacks
	cplex.use(LazyCallback(env, X, n));
	bool solved = false;

	try
	{
		// Try to solve with CPLEX (and hope it does not raise an exception!)
		solved = cplex.solve();
	}
	catch (const IloException& e)
	{
		cerr << "CPLEX Raised an exception:" << endl;
		cerr << e << endl;
		//release all the allocated resources
		Model.end();
		cplex.end();
		env.end();
		throw;
	}

	//Counters
	auto end = chrono::high_resolution_clock::now();
	auto ElapsedTotal = chrono::duration_cast<chrono::milliseconds>(end - start_0);
	auto ElapsedSetup = chrono::duration_cast<chrono::milliseconds>(start_1 - start_0);
	auto ElapsedSolving = chrono::duration_cast<chrono::milliseconds>(end - start_1);

	cout << "==========DONE==========" << endl;
	cout << "Total elapsed time(ms) : " << ElapsedTotal.count() << endl;
	cout << "|\tSetup elapsed time(ms): " << ElapsedSetup.count() << endl;
	cout << "|\tSolving elapsed time(ms): " << ElapsedSolving.count() << endl;

	if (solved)
	{
		// If CPLEX successfully solved the model, print the results
		double objective = cplex.getObjValue();

		//Solving output
		cout << "Solution (" << cplex.getStatus() << ") with objective " << objective << endl;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				float value = cplex.getValue(X[i][j]);
				if (value == 1)
				{
					cout << i << "->" << j << endl;
				}
			}
		}
	}
	else
	{
		cerr << "\n\nCplex error!" << endl;
		cerr << "\tStatus: " << cplex.getStatus() << endl;
		cerr << "\tSolver status: " << cplex.getCplexStatus() << endl;
	}

	//release all the allocated resources
	Model.end();
	cplex.end();
	env.end();
#pragma endregion
}

// Exécuter le programme : Ctrl+F5 ou menu Déboguer > Exécuter sans débogage
// Déboguer le programme : F5 ou menu Déboguer > Démarrer le débogage

// Astuces pour bien démarrer :
//   1. Utilisez la fenêtre Explorateur de solutions pour ajouter des fichiers et les gérer.
//   2. Utilisez la fenêtre Team Explorer pour vous connecter au contrôle de code source.
//   3. Utilisez la fenêtre Sortie pour voir la sortie de la génération et d'autres messages.
//   4. Utilisez la fenêtre Liste d'erreurs pour voir les erreurs.
//   5. Accédez à Projet > Ajouter un nouvel élément pour créer des fichiers de code, ou à Projet > Ajouter un élément existant pour ajouter des fichiers de code existants au projet.
//   6. Pour rouvrir ce projet plus tard, accédez à Fichier > Ouvrir > Projet et sélectionnez le fichier .sln.