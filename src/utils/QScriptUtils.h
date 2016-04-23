/*
 * @(#) QScriptUtils.h   1.0   Dec 23, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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
#ifndef ROBOGEN_QSCRIPTUTILS_H_
#define ROBOGEN_QSCRIPTUTILS_H_

#ifdef QT5_ENABLED

#include <QScriptEngine>
#include <osg/Vec3>
#include <osg/Quat>

namespace robogen {
namespace qscript {

QScriptValue valFromVec3(QScriptEngine* engine,
							osg::Vec3 vec);

QScriptValue valFromQuat(QScriptEngine* engine,
							osg::Quat quat);


}
}


#endif



#endif /* ROBOGEN_QSCRIPTUTILS_H_ */
