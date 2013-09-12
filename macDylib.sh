#!/bin/bash

# Created by Titus Cieslewski (dev@titus-c.ch) for
# Robogen, Copyright 2012-2014 Laboratory For Intelligent Systems, EPFL
# This script is supposed to help with binary deployment on mac.
# Made for OS X 10.6.8 Snow Leopard

EXECS="robogen-brain-evolver robogen-file-viewer robogen-server-viewer robogen-simulator"

echo "Copying dylibs to local folder:"
mkdir dylibs
cp /usr/local/lib/libosg.3.0.1.dylib dylibs
# the following is not listed in otool but referenced from libosg.3.0.1:
cp /usr/local/lib/libOpenThreads.12.dylib dylibs
cp /usr/local/lib/libosgViewer.3.0.1.dylib dylibs
cp /usr/local/lib/libosgDB.3.0.1.dylib dylibs 
cp /usr/local/lib/libosgGA.3.0.1.dylib dylibs 
cp /usr/local/lib/libosgTerrain.3.0.1.dylib dylibs
cp /usr/local/lib/libOpenThreads.2.6.0.dylib dylibs
cp /usr/lib/libz.1.dylib dylibs
# Note: we omit libstdc++ as this seems to be on macs per default
# cp /usr/lib/libstdc++.6.dylib dylibs
cp /usr/lib/libSystem.B.dylib dylibs

echo "Done"

for EXEC in ${EXECS} ; do
	echo "Changing dylib links of ${EXEC}:"
	install_name_tool -change /usr/local/lib/libosg.3.0.1.dylib @executable_path/dylibs/libosg.3.0.1.dylib ${EXEC}
	install_name_tool -change /usr/local/lib/libosgViewer.3.0.1.dylib @executable_path/dylibs/libosgViewer.3.0.1.dylib ${EXEC}
	install_name_tool -change /usr/local/lib/libosgDB.3.0.1.dylib @executable_path/dylibs/libosgDB.3.0.1.dylib ${EXEC}
	install_name_tool -change /usr/local/lib/libosgGA.3.0.1.dylib @executable_path/dylibs/libosgGA.3.0.1.dylib ${EXEC}
	install_name_tool -change /usr/local/lib/libosgTerrain.3.0.1.dylib @executable_path/dylibs/libosgTerrain.3.0.1.dylib ${EXEC}
	install_name_tool -change /usr/local/lib/libOpenThreads.2.6.0.dylib @executable_path/dylibs/libOpenThreads.2.6.0.dylib ${EXEC}
	install_name_tool -change /usr/lib/libz.1.dylib @executable_path/dylibs/libz.1.dylib ${EXEC}
	install_name_tool -change /usr/lib/libSystem.B.dylib @executable_path/dylibs/libSystem.B.dylib ${EXEC}
	echo "Done"
done