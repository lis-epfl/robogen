/*
 * @(#) emscripten_test.cpp   1.0   Dec 10, 2015
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

#include <boost/filesystem.hpp>

class Base {
public:
	void doSomething() { js::log("doing something new"); }
};

class Derived : public Base {
public:
	//void doSomething() { Base::doSomething(); }
};

void testPassVal(emscripten::val v) {
	std::stringstream ss;
	if (!v["a"].isUndefined()) {
		ss << v["a"].as<float>();
		robogen::js::log(ss.str());
	}
}

void ls(std::string dir) {
    boost::filesystem::path p(dir);

    boost::filesystem::directory_iterator end_itr;

    // cycle through the directory
    for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
        robogen::js::log(itr->path().string());
    }
}

emscripten::val testReturnArray() {
	emscripten::val result(emscripten::val::array());
	result.set(0, 2.3);
	result.set(1, 4.3);
	return result;
}

std::vector<float> testReturnVector() {
	std::vector<float> a;
	a.push_back(2.3);
	a.push_back(4.5);
	return a;
}

struct TestStruct {
	double a;
	TestStruct(double x) : a(x) {}

};

osg::Vec3 testReturnVec3() {
	return osg::Vec3(2.3, 4.4, 6.7);
}

boost::shared_ptr<TestStruct> testReturnSharedPtr() {
	return boost::shared_ptr<TestStruct>(new TestStruct(2) );
}

TestStruct* testReturnRawPtr() {
	return new TestStruct(4.3);
}

void printTestStruct(TestStruct *testPtr) {
	std::stringstream ss;
	ss << testPtr->a;
	robogen::js::log(ss.str());
}


void runEmBindTest() {
	std::string cmd;


	{
		std::stringstream ss;
		ss << emscripten_run_script_int("27");
		robogen::js::log(ss.str());
	}

	/*{
		std::stringstream ss;
		//ss << "function () {\n";
		ss << "new Module.TestStruct(2.2);" << "\n";
		//ss << "}();";
		cmd = ss.str();
	}*/

	{
		TestStruct* testPtr = new TestStruct(5.5);
		printTestStruct(testPtr);
	}

	{
		TestStruct* testPtr = reinterpret_cast<TestStruct*>( emscripten_run_script_int("Module.testReturnRawPtr();"));
		printTestStruct(testPtr);
	}
	{
		boost::shared_ptr<TestStruct> testPtr = (boost::shared_ptr<TestStruct>) emscripten::val::global("Module")["testReturnSharedPtr"]().as<boost::shared_ptr<TestStruct>>();
		printTestStruct(testPtr.get());
	}
	/*{
		TestStruct* testPtr = (TestStruct*) emscripten::val::global("Module")["testReturnRawPtr"]().as<TestStruct*>();
		printTestStruct(testPtr);
	}*/



}
