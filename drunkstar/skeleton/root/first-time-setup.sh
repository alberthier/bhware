#!/bin/sh

PYTHON=Python-2.7
PYTHON_ARCHIVE=$PYTHON.tar.bz2
PYSERIAL=pyserial-2.5
PYSERIAL_ARCHIVE=$PYSERIAL.tar.gz
MERCURIAL=mercurial-1.7.1
MERCURIAL_ARCHIVE=$MERCURIAL.tar.gz

bunzip2 -c $PYTHON_ARCHIVE | tar xf -
cd $PYTHON
./configure
make
make install
cd ..
rm -rf $PYTHON $PYTHON_ARCHIVE

gunzip -c $PYSERIAL_ARCHIVE | tar xf -
cd $PYSERIAL
python setup.py build
python setup.py install
cd ..
rm -rf $PYSERIAL $PYSERIAL_ARCHIVE

gunzip -c $MERCURIAL_ARCHIVE | tar xf -
cd $MERCURIAL
python setup.py build
python setup.py install
cd ..
rm -rf $MERCURIAL $MERCURIAL_ARCHIVE

rm -f $0
