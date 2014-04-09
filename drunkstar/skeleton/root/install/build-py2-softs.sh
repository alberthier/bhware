#!/bin/sh

PYTHON2=Python-2.7.6
PYTHON2_ARCHIVE=$PYTHON2.tgz
MERCURIAL=mercurial-2.9.2
MERCURIAL_ARCHIVE=$MERCURIAL.tar.gz

tar xzf $PYTHON2_ARCHIVE
cd $PYTHON2
find . -name "*" -exec touch {} \;
touch Include/Python-ast.h Python/Python-ast.c
./configure --enable-shared --disable-ipv6
make
make install
ldconfig
strip /usr/local/bin/python2.7
find /usr/local/lib -name "*.so" -exec strip {} \;
find /usr/local/lib/python2.7 -name "*.py" -exec rm -f {} \;
find /usr/local/lib/python2.7 -name "*.pyo" -exec rm -f {} \;
rm -rf /usr/local/lib/python2.7/test
rm -f /usr/local/lib/python2.7/config/libpython2.7.a
cd ..

tar xzf $MERCURIAL_ARCHIVE
cd $MERCURIAL
find . -name "*" -exec touch {} \;
python setup.py build
python setup.py install
ldconfig
find /usr/local/lib/python2.7/site-packages -name "*.py" -exec python -m py_compile {} \;
find /usr/local/lib/python2.7/site-packages -name "*.py" -exec rm -f {} \;
find /usr/local/lib/python2.7/site-packages -name "*.so" -exec strip {} \;
cd ..

tar cjf python-2.7.6_mercurial-2.9.2.tar.bz2 /usr/local
rm -rf /usr/local

ldconfig
