{ stdenv, fetchurl, python, buildPythonPackage }:

# stdenv.mkDerivation rec {
buildPythonPackage rec {
  name = "sip-4.14.7"; # kde410.pykde4 doesn't build with 4.15

  src = fetchurl {
    url = "mirror://sourceforge/pyqt/sip/${name}/${name}.tar.gz";
    sha256 = "1dv1sdwfmnq481v80k2951amzs9s87d4qhk0hpwrhb1sllh92rh5";
  };

  preConfigure = stdenv.lib.optionalString stdenv.isDarwin ''
    # prevent sip from complaining about python not being built as a framework
    sed -i -e 1564,1565d siputils.py
  '' + ''
    ${python.executable} ./configure.py \
      -d $out/lib/${python.libPrefix}/site-packages \
      -b $out/bin -e $out/include
  '';
  buildPhase = "make";
  installPhase = "make install";
  doCheck = false;
  NIX_CFLAGS_COMPILE = "-isystem ${python}/include/python"+python.majorVersion;
  NIX_LDFLAGS = "-lpython"+python.majorVersion;
  
  buildInputs = [ python ];

  meta = with stdenv.lib; {
    description = "Creates C++ bindings for Python modules";
    homepage    = "http://www.riverbankcomputing.co.uk/";
    license     = licenses.gpl2Plus;
    maintainers = with maintainers; [ lovek323 sander urkud ];
    platforms   = platforms.all;
  };
}
