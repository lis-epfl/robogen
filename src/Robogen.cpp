/*
 * @(#) RobogenOde.h   1.0   Mar 22, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include "Robogen.h"
#include "robogen.pb.h"

float fromOde(float x) {
	return x*1000.0;
}

double fromOde(double x) {
	return x*1000.0;
}

osg::Vec3 fromOde(osg::Vec3 x) {
	return x*1000.0;
}

void getRotationMatrixOde(osg::Quat quat, dQuaternion rotationMatrixOde) {
	dQuaternion quatOde;
	quatOde[0] = quat.w();
	quatOde[1] = quat.x();
	quatOde[2] = quat.y();
	quatOde[3] = quat.z();

	dQtoR(quatOde, rotationMatrixOde);

}

osg::Vec3 getRPYfromQuat(osg::Quat quat){
	// Implemented based on
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Conversion

	double q0 = quat.w();
	double q1 = quat.x();
	double q2 = quat.y();
	double q3 = quat.z();

	double roll = atan2( 2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
	double pitch = asin( 2 * (q0 * q2 - q3 * q1));
	double yaw = atan2(  2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

	return osg::Vec3d( roll, pitch, yaw );
}


int modulo(int x, int y) {
	while(x < 0) {
		x += y;
	}
	return (x % y);
}

void startRobogen() {
	GOOGLE_PROTOBUF_VERIFY_VERSION;
}

void exitRobogen(int exitCode) {
	google::protobuf::ShutdownProtobufLibrary();
#ifdef EMSCRIPTEN
	if (exitCode == EXIT_FAILURE)
		throw std::runtime_error("Fatal Error.");
#endif
	exit(exitCode);
}

