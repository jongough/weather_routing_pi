/******************************************************************************
 *
 * Project:  OpenCPN Weather Routing plugin
 * Author:   Sean D'Epagnier
 *
 ***************************************************************************
 *   Copyright (C) 2013 by Sean D'Epagnier                                 *
 *   sean@depagnier.com                                                    *
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
 ***************************************************************************
 */

#include <stdlib.h>
#include <math.h>

#include "Utilities.h"

/* some basic convenience functions */
double deg2rad(double degrees)
{
  return M_PI * degrees / 180.0;
}

double rad2deg(double radians)
{
  return 180.0 * radians / M_PI;
}

double heading_resolve(double degrees)
{
    while(degrees < -180)
        degrees += 360;
    while(degrees >= 180)
        degrees -= 360;
    return degrees;
}

double positive_degrees(double degrees)
{
    while(degrees < 0)
        degrees += 360;
    while(degrees >= 360)
        degrees -= 360;
    return degrees;
}

double rad2posdeg(double radians)
{
    return positive_degrees(rad2deg(radians));
}

double square(double x)
{
  return x*x;
}

double cube(double x)
{
  return x*x*x;
}