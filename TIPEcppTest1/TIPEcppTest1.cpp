// TIPEcppTest1.cpp : Ce fichier contient la fonction 'main'. L'exécution du programme commence et se termine à cet endroit.

#include <iostream>
#include <ilcplex/ilocplex.h>
#include <chrono>
#include <fstream>
#include <vector>
#include "utilities.h"
#include "FileManager.h"
using namespace std;

//Here we define a Matrix decision variable
//IloNumVarArray is an 1-dimentionnal decision variable
typedef IloArray<IloNumVarArray> NumVar2D;

int main()
{
#pragma region DataSetup
	auto start_0 = chrono::high_resolution_clock::now();

	auto func_out = FileManager::read_file("C:\\Users\\marcb\\Downloads\\20170608T121355407419\\tbl_truck_travel_data_PG.csv", false);
	//Stops
	const int n = sqrt(func_out.size() - 1);
	cout << "n: " << n << endl;
	//Distances matrix, from 1..n
	float** Distance = new float* [n];
	Distance = FileManager::read_standardized_csv(func_out, false);

	vector<int> arg;
	for (int i = 0; i < n; i++)
	{
		arg.push_back(i);
	}
	vector<vector<int>> sub = utilities::subset(arg);
	utilities::print_subsets(sub);

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
	for (int s = 0; s < sub.size(); s++)
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
	}

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

	//Solving
	IloCplex cplex(Model);

	if (!cplex.solve())
	{
		env.error() << "Failed to optimize the problem!" << endl;
		throw(-1);
	}
	double objective = cplex.getObjValue();

	//Counters
	auto end = chrono::high_resolution_clock::now();
	auto ElapsedTotal = chrono::duration_cast<chrono::milliseconds>(end - start_0);
	auto ElapsedSetup = chrono::duration_cast<chrono::milliseconds>(start_1 - start_0);
	auto ElapsedSolving = chrono::duration_cast<chrono::milliseconds>(end - start_1);

	cout << "==========DONE==========" << endl;
	cout << "Total elapsed time(ms) : " << ElapsedTotal.count() << endl;
	cout << "|\tSetup elapsed time(ms): " << ElapsedSetup.count() << endl;
	cout << "|\tSolving elapsed time(ms): " << ElapsedSolving.count() << endl;

	//Solving output
	cout << "Solution (optimal) with objective " << objective << endl;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			float value = cplex.getValue(X[i][j]);
			if (value == 1)
			{
				std::cout << i << "->" << j << endl;
			}
		}
	}
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