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

#include <wx/wx.h>

#include "ocpn_plugin.h"
#include "Utilities.h"
#include "Boat.h"
#include "GribRecordSet.h"
#include "RouteSimplifier.h"
#include "georef.h"

#include <algorithm>

/**
 * Helper function to find the closest position in an IsoRoute to a given point
 *
 * @param route The IsoRoute to search
 * @param lat Latitude of the target point
 * @param lon Longitude of the target point
 * @return Pointer to the closest position, or nullptr if none found
 */
Position* FindClosestPositionInRoute(IsoRoute* route, double lat, double lon) {
  if (!route || !route->skippoints) return nullptr;

  Position* closest = nullptr;
  double minDist = INFINITY;

  // Start with the first skippoint
  SkipPosition* skippos = route->skippoints;

  // Iterate through the circular list
  do {
    Position* pos = skippos->point;
    if (pos) {
      double dist = DistGreatCircle_Plugin(pos->lat, pos->lon, lat, lon);

      if (dist < minDist) {
        minDist = dist;
        closest = pos;
      }
    }

    skippos = skippos->next;
  } while (skippos != route->skippoints);

  return closest;
}

RouteSimplifier::RouteSimplifier(RouteMapOverlay* routemap)
    : m_routemap(routemap) {
  if (routemap) {
    m_configuration = routemap->GetConfiguration();
    m_originalRoute = ExtractPositionsFromRouteMap(routemap);
  }
}

RouteSimplifier::~RouteSimplifier() {
  // Clean up any positions we created during simplification
  for (Position* pos : m_newPositions) {
    delete pos;
  }
}

std::list<Position*> RouteSimplifier::ExtractPositionsFromRouteMap(
    RouteMapOverlay* routemap) {
  std::list<Position*> positions;

  if (!routemap) return positions;

  // Get the last destination position (always set after route completion)
  Position* pos = routemap->GetLastDestination();
  if (!pos) {
    // No route has been calculated yet
    return positions;
  }

  // Traverse back through parents to build the complete route
  while (pos) {
    positions.push_front(pos);  // Add to front to maintain correct order
    pos = pos->parent;
  }

  return positions;
}

double RouteSimplifier::GetTimePenalty() const {
  if (m_originalRoute.empty() || m_simplifiedRoute.empty()) return 0.0;

  double originalTime =
      const_cast<RouteSimplifier*>(this)->CalculateRouteTime(m_originalRoute);
  double simplifiedTime =
      const_cast<RouteSimplifier*>(this)->CalculateRouteTime(m_simplifiedRoute);

  if (originalTime <= 0) return 0.0;

  return ((simplifiedTime - originalTime) / originalTime) * 100.0;
}

double RouteSimplifier::CalculateEpsilon(double maxTimePenalty) {
  // Calculate an appropriate epsilon value based on the maximum time penalty
  // and the geographic extents of the route

  // Start with a base epsilon that scales with the penalty
  double epsilon = 0.001 * maxTimePenalty;

  // Get route bounds to scale epsilon appropriately
  double minLat = 90.0, maxLat = -90.0, minLon = 180.0, maxLon = -180.0;

  // Calculate from original route
  for (Position* pos : m_originalRoute) {
    minLat = std::min(minLat, pos->lat);
    maxLat = std::max(maxLat, pos->lat);
    minLon = std::min(minLon, pos->lon);
    maxLon = std::max(maxLon, pos->lon);
  }

  double latRange = maxLat - minLat;
  double lonRange = maxLon - minLon;

  // Scale epsilon based on route size
  double routeSize = std::max(latRange, lonRange);
  return epsilon * routeSize;
}

double RouteSimplifier::CalculateRouteTime(const std::list<Position*>& route) {
  double totalTime = 0.0;

  auto it1 = route.begin();
  auto it2 = std::next(it1);

  while (it2 != route.end()) {
    double heading;
    int data_mask = 0;

    double time = (*it1)->PropagateToPoint(
        (*it2)->lat, (*it2)->lon, m_configuration, heading, data_mask, false);

    if (std::isnan(time)) {
      // This shouldn't happen if validation was done correctly
      return INFINITY;
    }

    totalTime += time;

    it1 = it2;
    ++it2;
  }

  return totalTime;
}

void RouteSimplifier::ApplyDouglasPeucker(std::list<Position*>& route,
                                          double epsilon) {
  if (route.size() <= 2) return;  // Nothing to simplify

  // Convert list of positions to arrays for the DP algorithm
  int size = route.size();
  double* points = new double[size * 2];

  int i = 0;
  std::vector<Position*> posArray;
  for (Position* pos : route) {
    points[i * 2] = pos->lon;
    points[i * 2 + 1] = pos->lat;
    posArray.push_back(pos);
    i++;
  }

  // Apply Douglas-Peucker algorithm
  std::vector<int> keep;
  keep.push_back(0);         // Always keep start point
  keep.push_back(size - 1);  // Always keep end point

  // Call our implementation
  DouglasPeuckerImpl(points, 0, size - 1, epsilon, keep);

  // Sort indices to maintain proper order
  std::sort(keep.begin(), keep.end());

  // Rebuild the route with only the kept points
  std::list<Position*> simplified;
  for (int idx : keep) {
    simplified.push_back(posArray[idx]);
  }

  route = simplified;
  delete[] points;
}

void RouteSimplifier::DouglasPeuckerImpl(double* pointList, int startIndex,
                                         int endIndex, double epsilon,
                                         std::vector<int>& keep) {
  // Find the point with the maximum distance from the line segment
  double maxDistance = 0;
  int maxIndex = 0;

  // Get line segment endpoints
  double x1 = pointList[2 * startIndex];
  double y1 = pointList[2 * startIndex + 1];
  double x2 = pointList[2 * endIndex];
  double y2 = pointList[2 * endIndex + 1];

  // Line segment length squared
  double lineLength = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

  if (lineLength < 0.000001) {
    // Points are practically coincident - nothing to do
    return;
  }

  // Check all points between start and end
  for (int i = startIndex + 1; i < endIndex; i++) {
    double x = pointList[2 * i];
    double y = pointList[2 * i + 1];

    // Calculate perpendicular distance from point to line segment
    double dist;

    if (lineLength == 0) {
      // Points are identical, use point-to-point distance
      dist = (x - x1) * (x - x1) + (y - y1) * (y - y1);
    } else {
      // Use perpendicular distance formula
      double t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / lineLength;

      if (t < 0) {
        // Beyond the startIndex end of the segment
        dist = (x - x1) * (x - x1) + (y - y1) * (y - y1);
      } else if (t > 1) {
        // Beyond the endIndex end of the segment
        dist = (x - x2) * (x - x2) + (y - y2) * (y - y2);
      } else {
        // Perpendicular distance to line
        double projx = x1 + t * (x2 - x1);
        double projy = y1 + t * (y2 - y1);
        dist = (x - projx) * (x - projx) + (y - projy) * (y - projy);
      }
    }

    if (dist > maxDistance) {
      maxDistance = dist;
      maxIndex = i;
    }
  }

  // If max distance is greater than epsilon, recursively simplify
  if (maxDistance > epsilon * epsilon) {
    // Add the point to the keep list
    keep.push_back(maxIndex);

    // Recursive calls
    DouglasPeuckerImpl(pointList, startIndex, maxIndex, epsilon, keep);
    DouglasPeuckerImpl(pointList, maxIndex, endIndex, epsilon, keep);
  }
}

bool RouteSimplifier::ValidateSegmentWithPropagation(Position* start,
                                                     Position* end,
                                                     Position*& validated_end) {
  // Create a temporary configuration for propagation
  RouteMapConfiguration config = m_configuration;

  // Calculate bearing from start to end
  double bearing, distance;
  DistanceBearingMercator_Plugin(start->lat, start->lon, end->lat, end->lon,
                                 &bearing, &distance);

  // Set up a very narrow search angle around this bearing
  double narrow_angle = 5.0;  // Narrow angle for precise search

  // Create a temporary configuration with modified parameters
  config.MaxSearchAngle = narrow_angle;

  // Store the original config values we need to modify
  double origFromDegree = config.FromDegree;
  double origToDegree = config.ToDegree;
  double origByDegrees = config.ByDegrees;
  std::list<double> origDegreeSteps = config.DegreeSteps;

  // Clear existing DegreeSteps and create a focused set around the desired
  // bearing
  config.DegreeSteps.clear();

  // Calculate angle relative to true wind
  double twdOverWater;
  double twsOverWater;
  int data_mask = 0;
  if (!start->GetWindData(config, twdOverWater, twsOverWater, data_mask)) {
    // Couldn't get wind data, use a default approach
    // This would need to be handled better in a real implementation
    return false;
  }

  // Calculate the true wind angle that would give us the desired bearing
  double twa = heading_resolve(bearing - twdOverWater);

  // Set up a narrow range around this TWA
  config.FromDegree = twa - narrow_angle;
  config.ToDegree = twa + narrow_angle;
  config.ByDegrees = narrow_angle / 2;

  // Generate degree steps based on the narrow range
  for (double angle = config.FromDegree; angle <= config.ToDegree;
       angle += config.ByDegrees) {
    config.DegreeSteps.push_back(heading_resolve(angle));
  }

  // Set end position as the target
  config.EndLat = end->lat;
  config.EndLon = end->lon;

  // Try propagation with narrow angle
  IsoRouteList routelist;
  bool propagationSuccess = start->Propagate(routelist, config);

  // Restore original config values
  config.FromDegree = origFromDegree;
  config.ToDegree = origToDegree;
  config.ByDegrees = origByDegrees;
  config.DegreeSteps = origDegreeSteps;

  if (propagationSuccess && !routelist.empty()) {
    // Find the position closest to our target end point
    double minDistance = INFINITY;
    Position* bestPosition = nullptr;

    for (IsoRoute* route : routelist) {
      // Use our helper function instead of route->ClosestPosition
      Position* pos = FindClosestPositionInRoute(route, end->lat, end->lon);
      if (pos) {
        double dist =
            DistGreatCircle_Plugin(pos->lat, pos->lon, end->lat, end->lon);

        if (dist < minDistance) {
          minDistance = dist;
          bestPosition = pos;
        }
      }
    }

    // Check if we found a position close enough to the target
    if (bestPosition &&
        minDistance < 0.5) {  // Within acceptable distance (0.5 nm)
      // Create a new position based on the found one
      validated_end = new Position(
          bestPosition->lat, bestPosition->lon, start,
          bestPosition->parent_heading, bestPosition->parent_bearing,
          bestPosition->polar, bestPosition->tacks, bestPosition->jibes,
          bestPosition->sail_plan_changes, bestPosition->data_mask);

      // Store for later cleanup
      m_newPositions.push_back(validated_end);
      return true;
    }
  }

  // If direct propagation failed, try to propagate to end point
  double heading;
  double time = start->PropagateToPoint(end->lat, end->lon, config, heading,
                                        data_mask, false);

  if (!std::isnan(time)) {
    // Direct propagation succeeded
    validated_end = end;
    return true;
  }

  return false;
}

void RouteSimplifier::InsertIntermediateWaypoints(std::list<Position*>& route,
                                                  Position* start,
                                                  Position* end) {
  // Find positions in the original route between start and end
  auto startIt =
      std::find(m_originalRoute.begin(), m_originalRoute.end(), start);
  auto endIt = std::find(m_originalRoute.begin(), m_originalRoute.end(), end);

  if (startIt == m_originalRoute.end() || endIt == m_originalRoute.end())
    return;

  // Get distance between start and end positions in the original route
  int distance = std::distance(startIt, endIt);

  if (distance <= 2) {
    // Points are already adjacent in original route - nothing to insert
    return;
  }

  if (distance <= 4) {
    // For short segments, just insert all intermediate points
    for (auto it = std::next(startIt); it != endIt; ++it) {
      route.push_back(*it);
    }
    return;
  }

  // For longer segments, try a binary approach
  auto midIt = startIt;
  std::advance(midIt, distance / 2);

  // Try to validate segment from start to midpoint
  Position* validated_mid = nullptr;
  if (ValidateSegmentWithPropagation(start, *midIt, validated_mid)) {
    route.push_back(validated_mid);

    // Recursively handle second half
    InsertIntermediateWaypoints(route, validated_mid, end);
  } else {
    // If first half fails, insert all points from first half
    for (auto it = std::next(startIt); it != midIt; ++it) {
      route.push_back(*it);
    }

    // Then insert midpoint and handle second half
    route.push_back(*midIt);
    InsertIntermediateWaypoints(route, *midIt, end);
  }
}

bool RouteSimplifier::SimplifyRoute(double maxTimePenalty, int maxWaypoints) {
  // Clear any previous results
  m_simplifiedRoute.clear();

  // Make sure we have a valid route to simplify
  if (m_originalRoute.size() < 3) return false;  // Nothing to simplify

  // Calculate appropriate epsilon for Douglas-Peucker
  double epsilon = CalculateEpsilon(maxTimePenalty);

  // Apply Douglas-Peucker to get candidate waypoints
  std::list<Position*> candidateRoute = m_originalRoute;
  ApplyDouglasPeucker(candidateRoute, epsilon);

  // Initialize the simplified route with the start point
  m_simplifiedRoute.push_back(m_originalRoute.front());

  // Process each segment between candidate points
  auto it1 = candidateRoute.begin();
  auto it2 = std::next(it1);

  while (it2 != candidateRoute.end()) {
    // Try to validate the segment with propagation
    Position* validated_end = nullptr;
    if (ValidateSegmentWithPropagation(*it1, *it2, validated_end)) {
      // Add the validated end point to our route
      m_simplifiedRoute.push_back(validated_end);
    } else {
      // If validation fails, recursively insert intermediate waypoints
      InsertIntermediateWaypoints(m_simplifiedRoute, *it1, *it2);
      // Add the original end point
      m_simplifiedRoute.push_back(*it2);
    }

    it1 = it2;
    it2 = std::next(it2);
  }

  // Iteratively adjust epsilon if needed to satisfy maxWaypoints
  if (maxWaypoints > 0 && m_simplifiedRoute.size() > maxWaypoints) {
    // Try progressively larger epsilon values
    for (int i = 0; i < 3; i++) {
      epsilon *= 2.0;  // Double epsilon each attempt

      std::list<Position*> newCandidateRoute = m_originalRoute;
      ApplyDouglasPeucker(newCandidateRoute, epsilon);

      // If candidate route meets waypoint goal, repeat simplification
      if (newCandidateRoute.size() <= maxWaypoints) {
        m_simplifiedRoute.clear();
        m_simplifiedRoute.push_back(m_originalRoute.front());

        it1 = newCandidateRoute.begin();
        it2 = std::next(it1);

        while (it2 != newCandidateRoute.end()) {
          Position* validated_end = nullptr;
          if (ValidateSegmentWithPropagation(*it1, *it2, validated_end)) {
            m_simplifiedRoute.push_back(validated_end);
          } else {
            InsertIntermediateWaypoints(m_simplifiedRoute, *it1, *it2);
            m_simplifiedRoute.push_back(*it2);
          }

          it1 = it2;
          it2 = std::next(it2);
        }

        if (m_simplifiedRoute.size() <= maxWaypoints) break;
      }
    }
  }

  // Verify the simplified route meets the time penalty constraint
  double timePenalty = GetTimePenalty();

  if (timePenalty > maxTimePenalty) {
    // Time penalty is too high, use less aggressive simplification
    epsilon /= 4.0;  // Much smaller epsilon for less simplification

    std::list<Position*> newCandidateRoute = m_originalRoute;
    ApplyDouglasPeucker(newCandidateRoute, epsilon);

    m_simplifiedRoute.clear();
    m_simplifiedRoute.push_back(m_originalRoute.front());

    it1 = newCandidateRoute.begin();
    it2 = std::next(it1);

    while (it2 != newCandidateRoute.end()) {
      Position* validated_end = nullptr;
      if (ValidateSegmentWithPropagation(*it1, *it2, validated_end)) {
        m_simplifiedRoute.push_back(validated_end);
      } else {
        InsertIntermediateWaypoints(m_simplifiedRoute, *it1, *it2);
        m_simplifiedRoute.push_back(*it2);
      }

      it1 = it2;
      it2 = std::next(it2);
    }
  }

  return !m_simplifiedRoute.empty();
}