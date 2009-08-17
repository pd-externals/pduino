#!/bin/sh

PDUINO_VERSION=$1
FIRMATA_VERSION=$2

PDUINO_DIR=Pduino-${PDUINO_VERSION}
FIRMATA_DIR=Firmata-${FIRMATA_VERSION}

cd /tmp
mkdir pduino-release
cd pduino-release
svn co https://pure-data.svn.sourceforge.net/svnroot/pure-data/trunk/externals/hardware/arduino $PDUINO_DIR

cd $PDUINO_DIR
svn co https://firmata.svn.sourceforge.net/svnroot/firmata/arduino/trunk $FIRMATA_DIR

#remove cruft
find . -name .DS_Store -delete
find . -name .svn -print0 | xargs -0 rm -r
rm -rf PICduino examples arduino-stress-test.pd

zip -9r ../${FIRMATA_DIR}.zip $FIRMATA_DIR
cd ..
zip -9r ${PDUINO_DIR}.zip $PDUINO_DIR

# add to CVS for my website
cp -a ${FIRMATA_DIR}.zip ${PDUINO_DIR}.zip ~/code/works/pd/
cd ~/code/works/pd
cvs add ${FIRMATA_DIR}.zip ${PDUINO_DIR}.zip






