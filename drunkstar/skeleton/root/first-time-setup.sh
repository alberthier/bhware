#!/bin/sh

export LDSHARED="ld -shared"
export CPP=cpp
export CC=gcc

MERCURIAL=mercurial-2.1
MERCURIAL_ARCHIVE=$MERCURIAL.tar.gz

tar xzf $MERCURIAL_ARCHIVE
cd $MERCURIAL
find . -name "*" -exec touch {} \;
python setup.py build
python setup.py install
cd ..
rm -rf $MERCURIAL $MERCURIAL_ARCHIVE

rm -f $0
