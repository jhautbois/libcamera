/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * awb.cpp - Common AWB control algorithm
 */
#include "awb.h"

#include <algorithm>

namespace libcamera {

namespace ipa {

/**
 * The function estimates the correlated color temperature using
 * from RGB color space input.
 * In physics and color science, the Planckian locus or black body locus is
 * the path or locus that the color of an incandescent black body would take
 * in a particular chromaticity space as the blackbody temperature changes.
 *
 * If a narrow range of color temperatures is considered (those encapsulating
 * daylight being the most practical case) one can approximate the Planckian
 * locus in order to calculate the CCT in terms of chromaticity coordinates.
 *
 * More detailed information can be found in:
 * https://en.wikipedia.org/wiki/Color_temperature#Approximation
 */
uint32_t estimateCCT(double red, double green, double blue)
{
	/* Convert the RGB values to CIE tristimulus values (XYZ) */
	double X = (-0.14282) * (red) + (1.54924) * (green) + (-0.95641) * (blue);
	double Y = (-0.32466) * (red) + (1.57837) * (green) + (-0.73191) * (blue);
	double Z = (-0.68202) * (red) + (0.77073) * (green) + (0.56332) * (blue);

	/* Calculate the normalized chromaticity values */
	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	/* Calculate CCT */
	double n = (x - 0.3320) / (0.1858 - y);
	return 449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33;
}

void awbGreyWorld(std::vector<RGB> &zones, AwbStatus &results)
{
	/*
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains. We could
	 * consider some variations, such as normalising all the zones first, or
	 * doing an L2 average etc.
	 */
	std::vector<RGB> &redDerivative(zones);
	std::vector<RGB> blueDerivative(redDerivative);
	std::sort(redDerivative.begin(), redDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.R < b.G * a.R;
		  });
	std::sort(blueDerivative.begin(), blueDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.B < b.G * a.B;
		  });

	/* Average the middle half of the values. */
	int discard = redDerivative.size() / 4;

	RGB sumRed(0, 0, 0);
	RGB sumBlue(0, 0, 0);
	for (auto ri = redDerivative.begin() + discard,
		  bi = blueDerivative.begin() + discard;
	     ri != redDerivative.end() - discard; ri++, bi++)
		sumRed += *ri, sumBlue += *bi;

	double redGain = sumRed.G / (sumRed.R + 1),
	       blueGain = sumBlue.G / (sumBlue.B + 1);

	/* Color temperature is not relevant in Grey world but still useful to estimate it :-) */
	results.temperatureK = estimateCCT(sumRed.R, sumRed.G, sumBlue.B);
	results.redGain = redGain;
	results.greenGain = 1.0;
	results.blueGain = blueGain;
}

} /* namespace ipa */

} /* namespace libcamera */
