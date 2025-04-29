/***************************************************************************
 *   Copyright (C) 2016 by OpenCPN Development Team                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 ***************************************************************************/

#ifndef _WEATHER_ROUTING_ROUTE_SIMPLIFIER_H_
#define _WEATHER_ROUTING_ROUTE_SIMPLIFIER_H_

#include <wx/colour.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/string.h>

#include "RouteMap.h"
#include "RouteMapOverlay.h"
#include <list>
#include <vector>

class RouteSimplifier {
public:
  RouteSimplifier(RouteMapOverlay* routemap);
  ~RouteSimplifier();

  /**
   * Main simplification function
   * @param maxTimePenalty Maximum acceptable time penalty as a percentage
   * @param maxWaypoints Maximum number of waypoints in the simplified route
   *                     (0 for no limit)
   * @return true if simplification succeeded, false otherwise
   *
   * Performs the route simplification using Douglas-Peucker algorithm
   * followed by validation and adjustment to meet the specified constraints.
   * The simplified route will have at most maxWaypoints points and will
   * not exceed the specified time penalty compared to the original route.
   */
  bool SimplifyRoute(double maxTimePenalty, int maxWaypoints = 0);

  /**
   * Get the resulting simplified route
   * @return List of Position pointers representing the simplified route
   *
   * Should be called after SimplifyRoute() to retrieve the results.
   */
  std::list<Position*> GetSimplifiedRoute() const { return m_simplifiedRoute; }

  /**
   * Get the number of waypoints in the original route
   * @return Count of positions in the original route
   */
  int GetOriginalPointCount() const { return m_originalRoute.size(); }
  /**
   * Get the number of waypoints in the simplified route
   * @return Count of positions in the simplified route
   */
  int GetSimplifiedPointCount() const { return m_simplifiedRoute.size(); }
  /**
   * Calculate the time penalty of the simplified route
   * @return Time penalty as a percentage (e.g., 5.0 for 5% penalty)
   *
   * Compares the travel time of the simplified route to the original
   * and returns the percentage increase in travel time.
   */
  double GetTimePenalty() const;

private:
  /**
   * Extract positions from the route map
   * @param routemap The RouteMapOverlay to extract positions from
   * @return List of Position pointers representing the route
   *
   * Traverses the route map structure to extract all positions,
   * either from the destination position's parent chain or from
   * the plot data if no destination position is available.
   */
  std::list<Position*> ExtractPositionsFromRouteMap(RouteMapOverlay* routemap);
  /**
   * Apply Douglas-Peucker simplification algorithm
   * @param route Reference to the route to simplify (modified in place)
   * @param epsilon Distance threshold for point removal
   *
   * Implements the classic Douglas-Peucker algorithm to reduce the
   * number of points while maintaining the route's general shape.
   * Points further than epsilon from the simplified line are kept.
   */
  void ApplyDouglasPeucker(std::list<Position*>& route, double epsilon);
  void DouglasPeuckerImpl(double* pointList, int startIndex, int endIndex,
                          double epsilon, std::vector<int>& keep);
  /**
   * Validate a route segment using propagation
   * @param start Starting position of the segment
   * @param end Ending position of the segment
   * @param validated_end Output parameter for the validated end position
   * @return true if the segment is valid, false otherwise
   *
   * Attempts to propagate from start to end using the weather routing
   * engine to ensure the segment is navigable. If direct propagation
   * fails, tries to find a nearby valid position.
   */
  bool ValidateSegmentWithPropagation(Position* start, Position* end,
                                      Position*& validated_end);
  /**
   * Insert intermediate waypoints between two positions
   * @param route Reference to the route being built
   * @param start Starting position
   * @param end Ending position
   *
   * When a direct segment between two positions is not valid,
   * this function recursively adds intermediate waypoints from
   * the original route using a binary search approach.
   */
  void InsertIntermediateWaypoints(std::list<Position*>& route, Position* start,
                                   Position* end);
  /**
   * Calculate the total travel time for a route
   * @param route The route to calculate time for
   * @return Total travel time in hours
   *
   * Sums up the propagation time between consecutive waypoints
   * in the route using the weather routing configuration.
   */
  double CalculateRouteTime(const std::list<Position*>& route);
  /**
   * Calculate appropriate epsilon value for Douglas-Peucker
   * @param maxTimePenalty Maximum acceptable time penalty
   * @return Epsilon value to use for simplification
   *
   * Determines an appropriate distance threshold based on the
   * allowed time penalty and the geographic extent of the route.
   * Larger epsilon values result in more aggressive simplification.
   */
  double CalculateEpsilon(double maxTimePenalty);

  // Member variables
  RouteMapOverlay* m_routemap;
  RouteMapConfiguration m_configuration;
  std::list<Position*> m_originalRoute;
  std::list<Position*> m_simplifiedRoute;
  std::vector<Position*>
      m_newPositions;  // Store newly created positions for cleanup
};

#endif