//! \file PathSearch.h
//! \brief Defines the <code>fullsail_ai::algorithms::PathSearch</code> class interface.
//! \author Cromwell D. Enage, 2009; Jeremiah Blanchard, 2012
#ifndef _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_
#define _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_

// change this to start the program on whatever default map as you like from the table below
#define USEDEFAULTMAP hex035x035

#define hex006x006 "./Data/hex006x006.txt"
#define hex014x006 "./Data/hex014x006.txt"
#define hex035x035 "./Data/hex035x035.txt"
#define hex054x045 "./Data/hex054x045.txt"
#define hex098x098 "./Data/hex098x098.txt"
#define hex113x083 "./Data/hex113x083.txt"

// change this to 1(true), and change the data below when you want to test specific starting and goal locations on startup
#define OVERRIDE_DEFAULT_STARTING_DATA 0

// Make sure your start and goal are valid locations!
#define DEFAULT_START_ROW 0
#define DEFAULT_START_COL 0
#define DEFAULT_GOAL_ROW ?
#define DEFAULT_GOAL_COL ?

#include "../TileSystem/Tile.h"
#include "../TileSystem/TileMap.h"
#include "../platform.h"
#include "../PriorityQueue.h"
#include <vector>
#include <map>
#include <queue>
#include <cmath>

namespace fullsail_ai { namespace algorithms {

	bool areAdjacent(Tile const* lhs, Tile const* rhs);
	float distance(Tile const* lhs, Tile const* rhs);

	struct PlannerNode;

	struct SearchNode
	{

		Tile* data;
		std::vector<SearchNode*> edges;
		PlannerNode* myPlanner;

		float heuristicCost;
		float givenCost = 0;
		float finalCost = 0;

	};

	struct PlannerNode
	{

		PlannerNode* parent;
		SearchNode* Vertex;

	};

	bool BreadthFirstSort(SearchNode* const&, SearchNode* const&);

	class PathSearch
	{

		std::map<Tile*, SearchNode*> SearchMap;
		TileMap* currentTileMap;

		SearchNode* startNode;
		SearchNode* goalNode;

		PriorityQueue<SearchNode*>* searchQueue;
		//std::queue<SearchNode*> searchQueue;
		std::vector<PlannerNode*> cleanMe;
		std::vector<PlannerNode*> solution;
		std::map<SearchNode*, bool> nodeState;

		long currentTime;
		long startTime;

		bool reachedGoal = false;

	public:
		//! \brief Default constructor.
		DLLEXPORT PathSearch();

		//! \brief Destructor.
		DLLEXPORT ~PathSearch();

		//! \brief Sets the tile map.
		//!
		//! Invoked when the user opens a tile map file.
		//!
		//! \param   _tileMap  the data structure that this algorithm will use
		//!                    to access each tile's location and weight data.
		DLLEXPORT void initialize(TileMap* _tileMap);

		//! \brief Enters and performs the first part of the algorithm.
		//!
		//! Invoked when the user presses one of the play buttons.
		//!
		//! \param   startRow         the row where the start tile is located.
		//! \param   startColumn      the column where the start tile is located.
		//! \param   goalRow          the row where the goal tile is located.
		//! \param   goalColumn       the column where the goal tile is located.
		DLLEXPORT void enter(int startRow, int startColumn, int goalRow, int goalColumn);

		//! \brief Returns <code>true</code> if and only if no nodes are left open.
		//!
		//! \return  <code>true</code> if no nodes are left open, <code>false</code> otherwise.
		DLLEXPORT bool isDone() const;

		//! \brief Performs the main part of the algorithm until the specified time has elapsed or
		//! no nodes are left open.
		DLLEXPORT void update(long timeslice);

		//! \brief Returns an unmodifiable view of the solution path found by this algorithm.
		DLLEXPORT std::vector<Tile const*> const getSolution() const;

		//! \brief Resets the algorithm.
		DLLEXPORT void exit();

		//! \brief Uninitializes the algorithm before the tile map is unloaded.
		DLLEXPORT void shutdown();
	};
}}  // namespace fullsail_ai::algorithms

#endif  // _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_

