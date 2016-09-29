{ fetchurl, buildPythonPackage, doCheck ? false, extradeps ? {} }:
with extradeps;
rec {
  
  "rosdep" = buildPythonPackage {
    name = "rosdep-0.11.5";
    src = fetchurl {
      url = "https://pypi.python.org/packages/ae/a5/4de74df30a62823845d2707600d0d1f64afcc5268057f36e6a7917846a5d/rosdep-0.11.5.tar.gz";
      sha256 = "0snrxx04lnxplywfjl69b2rk87yw9d7q2a8q84jym8bh883p5q26";
    };
    inherit doCheck;
    buildInputs = [ catkin_pkg rospkg rosdistro pyyaml argparse docutils python-dateutil setuptools six ];
  };
  
  "rosinstall-generator" = buildPythonPackage {
    name = "rosinstall-generator-0.1.12";
    src = fetchurl {
      url = "https://pypi.python.org/packages/60/82/d634551b4f7c09c024da89f9f4f0549b4fb0608203bc167098f90549af4e/rosinstall_generator-0.1.12.tar.gz";
      sha256 = "0zfq4pb4n5czphp6873i9hl58mn2v82pd6lvd07mvb67vzvbh84z";
    };
    inherit doCheck;
    buildInputs = [ argparse catkin_pkg rosdistro rospkg pyyaml setuptools docutils python-dateutil six ];
  };
  
  "wstool" = buildPythonPackage {
    name = "wstool-0.1.13";
    src = fetchurl {
      url = "https://pypi.python.org/packages/be/25/35798aa230398b99a17edcea3bcea894fac18b98e8de8894a2c31fca13f0/wstool-0.1.13.tar.gz";
      sha256 = "0i024xd4c6386q9y7vhkb82vz3ba77wvwvfyfqh2m17p9gwx7bjf";
    };
    inherit doCheck;
    buildInputs = [ vcstools pyyaml python-dateutil six ];
  };
  
  "rosinstall" = buildPythonPackage {
    name = "rosinstall-0.7.8";
    src = fetchurl {
      url = "https://pypi.python.org/packages/38/c5/2c466034d05ad84ca15179d1f496958c6caad9432928bf7436711cc3e64e/rosinstall-0.7.8.tar.gz";
      sha256 = "0h7d8ynv44c68sbfn28xw4k18k3ip6252x7r7bqw6b5cifzhia1b";
    };
    inherit doCheck;
    buildInputs = [ vcstools pyyaml rosdistro catkin_pkg wstool python-dateutil rospkg setuptools argparse docutils six ];
  };
  
  "markerlib" = buildPythonPackage {
    name = "markerlib-0.6.0";
    src = fetchurl {
      url = "https://pypi.python.org/packages/d1/ba/ce29589707d8679b648517d936a2f08e3e18143d49822339d3cc35819c92/markerlib-0.6.0.tar.gz";
      sha256 = "1bp3nq35rbl8vsakj3kcxlzd64aaz0s9m5xij3qg8nqz8hwkknrg";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "catkin_tools" = buildPythonPackage {
    name = "catkin_tools-0.4.2";
    src = fetchurl {
      url = "https://pypi.python.org/packages/2e/53/2b1bd541e5454e3ade3bbf0c3a3e4df37f4b085a674798739d93f87d095f/catkin_tools-0.4.2.tar.gz";
      sha256 = "0yylxh4wg9f1gsbjhyarhyy40m7xgir8nqr0nadvxs2gkzcdcbgf";
    };
    inherit doCheck;
    buildInputs = [ catkin_pkg setuptools pyyaml osrf-pycommon argparse docutils python-dateutil trollius six futures ];
  };
  
  "catkin_pkg" = buildPythonPackage {
    name = "catkin_pkg-0.2.10";
    src = fetchurl {
      url = "https://pypi.python.org/packages/13/b8/87671c4003ae2dd73e6dced6ec2cab5a8023d4baeb070acda148e0a53417/catkin_pkg-0.2.10.tar.gz";
      sha256 = "0l7hxq69sn9ymzmna9zap7zf1dhm2p220fvpf7pakmi9xgm6kl6l";
    };
    inherit doCheck;
    buildInputs = [ argparse docutils python-dateutil six ];
  };
  
  "bloom" = buildPythonPackage {
    name = "bloom-0.5.22";
    src = fetchurl {
      url = "https://pypi.python.org/packages/58/6b/837d432ddea56502e0f56e321d8407175ecf9db3221eb8e45e66d24b4207/bloom-0.5.22.tar.gz";
      sha256 = "0nhm4sps4h931fsiwm8zqp42cy6wgqy4flzqc9y9l14sv1ar46m9";
    };
    inherit doCheck;
    buildInputs = [ catkin_pkg setuptools empy python-dateutil pyyaml rosdep rosdistro vcstools argparse docutils six rospkg ];
  };
  
  "rospkg" = buildPythonPackage {
    name = "rospkg-1.0.40";
    src = fetchurl {
      url = "https://pypi.python.org/packages/ca/83/58130c1b6c3e41adaacde60762f9bdd80d8c2bf4bb05fcbe2ccf2e9621ef/rospkg-1.0.40.tar.gz";
      sha256 = "075y1lszcy4caz0jimz0kjk4hyn19zv6lcxp768vfhcx64sxdlfc";
    };
    inherit doCheck;
    buildInputs = [ pyyaml ];
  };
  
  "rosdistro" = buildPythonPackage {
    name = "rosdistro-0.4.7";
    src = fetchurl {
      url = "https://pypi.python.org/packages/88/f6/25fc3a45e6935bc2dd4f58af0e18c238ecb902e6c16882652caef56b3b97/rosdistro-0.4.7.tar.gz";
      sha256 = "1i7d3qv3bgfqc2pdcf2jzvmrxq64wx404mvddyghir6fdcrmfizl";
    };
    inherit doCheck;
    buildInputs = [ catkin_pkg rospkg pyyaml setuptools argparse docutils python-dateutil six ];
  };
  
  "pyyaml" = buildPythonPackage {
    name = "pyyaml-3.12";
    src = fetchurl {
      url = "https://pypi.python.org/packages/4a/85/db5a2df477072b2902b0eb892feb37d88ac635d36245a72a6a69b23b383a/PyYAML-3.12.tar.gz";
      sha256 = "1aqjl8dk9amd4zr99n8v2qxzgmr2hdvqfma4zh7a41rj6336c9sr";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "argparse" = buildPythonPackage {
    name = "argparse-1.4.0";
    src = fetchurl {
      url = "https://pypi.python.org/packages/18/dd/e617cfc3f6210ae183374cd9f6a26b20514bbb5a792af97949c5aacddf0f/argparse-1.4.0.tar.gz";
      sha256 = "1r6nznp64j68ih1k537wms7h57nvppq0szmwsaf99n71bfjqkc32";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "vcstools" = buildPythonPackage {
    name = "vcstools-0.1.39";
    src = fetchurl {
      url = "https://pypi.python.org/packages/ab/02/4ec8ab21f821866b6a0572af37e611929594fda588b54633607cc0a25cb4/vcstools-0.1.39.tar.gz";
      sha256 = "1harbyc4x9qv0w4g0z8ixwaq8028kfks5vjzsk735d96lgndy6w4";
    };
    inherit doCheck;
    buildInputs = [ pyyaml python-dateutil six ];
  };
  
  "osrf-pycommon" = buildPythonPackage {
    name = "osrf-pycommon-0.1.2";
    src = fetchurl {
      url = "https://pypi.python.org/packages/77/f4/43c60590497c1d4a04f4d608ef3f292c30aa64102d9c8adee06f672a1c48/osrf_pycommon-0.1.2.tar.gz";
      sha256 = "0h2xby5wahjgdrzbq9wvrwx621ka4bbmnhvbbmn2m1kc2abiapgx";
    };
    inherit doCheck;
    buildInputs = [ setuptools trollius six futures ];
  };
  
  "docutils" = buildPythonPackage {
    name = "docutils-0.12";
    src = fetchurl {
      url = "https://pypi.python.org/packages/37/38/ceda70135b9144d84884ae2fc5886c6baac4edea39550f28bcd144c1234d/docutils-0.12.tar.gz";
      sha256 = "1ylnjnw1x4b2y7blr6x35ncdzn69k253kw4cdkv6asdb21w73ny7";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "python-dateutil" = buildPythonPackage {
    name = "python-dateutil-2.5.3";
    src = fetchurl {
      url = "https://pypi.python.org/packages/3e/f5/aad82824b369332a676a90a8c0d1e608b17e740bbb6aeeebca726f17b902/python-dateutil-2.5.3.tar.gz";
      sha256 = "1v9j9fmf8g911yg6k01xa2db6dx3wv73zkk7fncsj7vagjqgs20l";
    };
    inherit doCheck;
    buildInputs = [ six ];
  };
  
  "empy" = buildPythonPackage {
    name = "empy-3.3.2";
    src = fetchurl {
      url = "https://pypi.python.org/packages/b7/56/72a285d257c7791616960493a04f14c05ca1bf7a83dd208485cf991563bd/empy-3.3.2.tar.gz";
      sha256 = "177avx6iv9sq2j2iak2il5lxqq0k4np7mpv5gasqmi3h4ypidw4r";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "trollius" = buildPythonPackage {
    name = "trollius-2.1";
    src = fetchurl {
      url = "https://pypi.python.org/packages/6e/72/5940cfb765488cfe1b62883a0d0e5438f4fc17cfefd4fb4654a5982be852/trollius-2.1.tar.gz";
      sha256 = "146c60hgcmgjkbf2hmiag52f9i3hka6shwbfybdsmlvqjnfms5nd";
    };
    inherit doCheck;
    buildInputs = [ six futures ];
  };
  
  "six" = buildPythonPackage {
    name = "six-1.10.0";
    src = fetchurl {
      url = "https://pypi.python.org/packages/b3/b2/238e2590826bfdd113244a40d9d3eb26918bd798fc187e2360a8367068db/six-1.10.0.tar.gz";
      sha256 = "0snmb8xffb3vsma0z67i0h0w2g2dy0p3gsgh9gi4i0kgc5l8spqh";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
  
  "futures" = buildPythonPackage {
    name = "futures-3.0.5";
    src = fetchurl {
      url = "https://pypi.python.org/packages/55/db/97c1ca37edab586a1ae03d6892b6633d8eaa23b23ac40c7e5bbc55423c78/futures-3.0.5.tar.gz";
      sha256 = "1pw1z4329xvlabdpwqa6b7v2fxf7hl64m4cgr22ckbym8m8m4hh5";
    };
    inherit doCheck;
    buildInputs = [  ];
  };
}
