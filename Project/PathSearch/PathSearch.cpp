#include "PathSearch.h"
#include <ctime>
#include <vld.h>
#include <iostream>

#define uint unsigned int

namespace fullsail_ai {
	namespace algorithms {

		bool areAdjacent(Tile const* lhs, Tile const* rhs)
		{
			int _lX = lhs->getColumn();
			int _lY = lhs->getRow();
			int _rX = rhs->getColumn();
			int _rY = rhs->getRow();

			if (!lhs->getWeight() || !rhs->getWeight())
				return false;

			if (_lX == _rX)
			{

				if (_lY == (_rY - 1) ||
					_lY == (_rY + 1))
					return true;

			}

			if (_lY == _rY)
			{

				if (_lX == (_rX - 1) ||
					_lX == (_rX + 1))
					return true;

			}

			//even row
			if (!(_lY % 2))
			{

				if ((_lX - 1) == _rX)
				{

					if (_lY == (_rY - 1) ||
						_lY == (_rY + 1))
						return true;

				}

			}
			//odd row
			else
			{

				if ((_lX + 1) == _rX)
				{

					if (_lY == (_rY - 1) ||
						_lY == (_rY + 1))
						return true;

				}

			}

			return false;

		}
		bool BreadthFirstSort(SearchNode* const& lhs, SearchNode* const& rhs)
		{

			//return lhs->heuristicCost > rhs->heuristicCost;
			return lhs->givenCost > rhs->givenCost;
			//return (lhs->givenCost + lhs->heuristicCost) > (rhs->givenCost + rhs->heuristicCost);

		}
		float distance(Tile* lhs, Tile* rhs)
		{

			float _xDelta = lhs->getXCoordinate() - rhs->getXCoordinate();
			float _yDelta = lhs->getYCoordinate() - rhs->getYCoordinate();

			return sqrt(_xDelta * _xDelta + _yDelta * _yDelta);

		}

		PathSearch::PathSearch()
		{

			searchQueue = new PriorityQueue<SearchNode*>(BreadthFirstSort);

		}

		PathSearch::~PathSearch()
		{

			int _nRows = currentTileMap->getRowCount();
			int _nCols = currentTileMap->getColumnCount();

			for (int y = 0; y < _nRows; y++)
				for (int x = 0; x < _nCols; x++)
					delete SearchMap[currentTileMap->getTile(y, x)];

		}

		void PathSearch::initialize(TileMap* _tileMap)
		{

			currentTileMap = _tileMap;

			int _debugTileX = 2;
			int _debugTileY = 2;

			int _nRows = _tileMap->getRowCount();
			int _nCols = _tileMap->getColumnCount();

			for (int y = 0; y < _nRows; y++)
				for (int x = 0; x < _nCols; x++)
				{

					SearchNode* _nNode = new SearchNode;
					_nNode->data = _tileMap->getTile(y, x);
					SearchMap[_tileMap->getTile(y, x)] = _nNode;

				}

			for (int y = 0; y < _nRows; y++)
				for (int x = 0; x < _nCols; x++)
					for (int _y = (y - 1); _y < (y + 2); _y++)
						for (int _x = (x - 1); _x < (x + 2); _x++)
							if (_x >= 0 && _x < _nCols && _y >= 0 && _y <= _nRows && (x != _x || y != _y) && _tileMap->getTile(y, x) && _tileMap->getTile(y, x)->getWeight())
							{

								Tile* _thisTile = _tileMap->getTile(y, x);
								Tile* _adjTile = _tileMap->getTile(_y, _x);

								/*if (!_thisTile->getFill())
									_thisTile->setFill(0xFFFFFF00);

								if (x == _debugTileX && y == _debugTileY)
								{

									_thisTile->setFill(0xFF00FF00);
									_adjTile->setFill(0xFFFF0000);

								}*/


								if (_thisTile != _adjTile && _thisTile && _adjTile && areAdjacent(_thisTile, _adjTile))
								{
									SearchMap[_thisTile]->edges.push_back(SearchMap[_adjTile]);

									/*if (x == _debugTileX && y == _debugTileY)
										_adjTile->setFill(0xFF0000FF);*/

								}

							}


		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{

			startNode = SearchMap[currentTileMap->getTile(startRow, startColumn)];
			goalNode = SearchMap[currentTileMap->getTile(goalRow, goalColumn)];

			startNode->heuristicCost = startNode->finalCost = distance(startNode->data, goalNode->data);

			PlannerNode* _startPlanner = new PlannerNode;
			_startPlanner->parent = nullptr;
			_startPlanner->Vertex = startNode;
			startNode->myPlanner = _startPlanner;
			cleanMe.push_back(_startPlanner);

			searchQueue->push(startNode);
			nodeState[startNode] = true;

			startTime = clock();

		}

		void PathSearch::update(long timeslice)
		{

			startTime = clock();

			while (1)
			{

				currentTime = clock();
				float _elapsedTime = currentTime - startTime;

				if (_elapsedTime < timeslice)
				{
					SearchNode* _currentNode = searchQueue->front();
					searchQueue->pop();

					if (_currentNode != goalNode)
					{

						for (int i = 0; i < _currentNode->edges.size(); i++)
							if (!nodeState[_currentNode->edges[i]])
							{

								PlannerNode* _newPlanner = new PlannerNode;
								_newPlanner->parent = _currentNode->myPlanner;
								_newPlanner->Vertex = _currentNode->edges[i];
								_currentNode->edges[i]->myPlanner = _newPlanner;
								_currentNode->edges[i]->heuristicCost = distance(_currentNode->edges[i]->data, goalNode->data);
								_currentNode->edges[i]->givenCost = _currentNode->givenCost + distance(_currentNode->edges[i]->data, _currentNode->myPlanner->Vertex->data);
								cleanMe.push_back(_newPlanner);

								searchQueue->push(_currentNode->edges[i]);
								nodeState[_currentNode->edges[i]] = true;

							}

					}
					else
					{

						PlannerNode* _cPlanner = goalNode->myPlanner;

						while (1)
						{

							solution.push_back(_cPlanner);

							if (_cPlanner->Vertex == startNode)
								break;

							if (_cPlanner->Vertex != startNode && _cPlanner->Vertex != goalNode)
								_cPlanner->Vertex->data->setFill(0xFF0000FF);

							_cPlanner = _cPlanner->parent;

						}

						reachedGoal = true;
						return;

					}

				}
				else return;

			}

		}

		void PathSearch::exit()
		{

			for (uint i = 0; i < cleanMe.size(); i++)
				delete cleanMe[i];

			cleanMe.clear();
			solution.clear();
			nodeState.clear();

			startNode->myPlanner = goalNode->myPlanner = nullptr;

		}

		void PathSearch::shutdown()
		{

			for (uint i = 0; i < cleanMe.size(); i++)
				delete cleanMe[i];

			cleanMe.clear();
			solution.clear();
			nodeState.clear();

		}

		bool PathSearch::isDone() const
		{
			return reachedGoal;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			std::vector<Tile const*> temp;

			for (int i = 0; i < solution.size(); i++)
				temp.push_back(solution[i]->Vertex->data);

			return temp;
		}
	}
}  // namespace fullsail_ai::algorithms

