#pragma once
#include "../Physics/Body.h"
#include "../Physics/Constraints.h"


int AddSpheres(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex
	,const int stressLevel, const float startHeight, bool isDense);
int AddDiamonds(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex
	,const int stressLevel, const float startHeight, bool isDense);
int AddFloor(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex
	, bool isDense);

int AddStack(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);
int AddMover(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);
int AddChain(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);

int AddHinge(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);
int AddVelocity(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);
int AddOrientation(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);

int AddSpinner(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);
int AddRagdoll(std::vector< Body >& bodies, std::vector<Constraint*> & constraints
	,std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);

int AddConvex(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);


int AddSandbox(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex);