#pragma once

#include <string>
#include <vector>
#include <list>
#include "../Eigen/Dense"
#include <fstream>
#include <iostream>


static void saveDataToText(std::vector<Eigen::VectorXd> data, std::string str)
{
	std::ofstream fout;
	fout.open(str);

	for (unsigned int i = 0; i < data.size(); i++)
	{
		for (int j = 0; j < data[i].rows(); j++)
		{
			fout << data[i](j) << '\t';
		}
		fout << std::endl;
	}


	fout.close();
}

static void saveDataToText(std::list<Eigen::VectorXd> data, std::string str)
{
	std::ofstream fout;
	fout.open(str);
	std::list<Eigen::VectorXd>::iterator iter = data.begin();
	std::list<Eigen::VectorXd>::iterator end_iter = data.end();
	for (iter; iter != end_iter; iter++)
	{
		for (int j = 0; j < (*iter).rows(); j++)
		{
			fout << (*iter)(j) << '\t';
		}
		fout << std::endl;
	}


	fout.close();
}


static void saveSingleDataToText(Eigen::VectorXd data, std::string str)
{
	std::ofstream fout;
	fout.open(str);


		for (int j = 0; j < data.rows(); j++)
		{
			fout << data(j) << '\t';
		}
		fout << std::endl;

	fout.close();
}


static std::vector<Eigen::VectorXd> loadDataFromText(std::string str, int colNum)
{
	std::ifstream fin;
	fin.open(str);

	std::vector<Eigen::VectorXd> data;
	Eigen::VectorXd tmpOneData(colNum);
	while (!fin.eof())
	{
		for (int i = 0; i < colNum; i++)
			fin >> tmpOneData(i);

		data.push_back(tmpOneData);
	}
	data.pop_back();

	fin.close();

	return data;
}
     
static Eigen::VectorXd loadVectorFromText(std::string str, int colNum)
{
	std::ifstream fin;
	fin.open(str);

	Eigen::VectorXd tmpOneData(colNum);
	while (!fin.eof())
	{
		for (int i = 0; i < colNum; i++)
			fin >> tmpOneData(i);

	}

	return tmpOneData;
}
