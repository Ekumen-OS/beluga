# Beluga VDB

<div align="center">
<img alt="Shows the Beluga logo." src="images/57fe338a-3e7e-46b3-ad2f-17011d6d306a.png" height="200">
</div>

## 🌐 Overview

BelugaVDB is a library extension for `beluga` that integrates [OpenVDB](https://www.openvdb.org/), enabling advanced 3D localization capabilities. Currently, this extension uses the powerful volumetric data structure of OpenVDB to efficiently process maps and 3D pointcloud data, providing the localization of the autonomous system.


## 📦 External Dependencies

### OpenVDB

For Ubuntu 24.04 and newer versions OpenVDB can be installed using `apt`:

```bash
sudo apt install libopenvdb-dev
```

Older version must be isnstalled from sources as follow:

#### Step 1: Download OpenVDB

Clone [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) into your machine (for Jammy version 8.2.0 is recommended):

```bash
git clone -b v8.2.0 https://github.com/AcademySoftwareFoundation/openvdb.git
```

#### Step 2: Compile OpenVDB

Now you need to build OpenVDB:

```bash
cd openvdb && \
mkdir build && cd build && \
cmake .. && \
make -j$(nproc)
```

#### Step 3: Install OpenVDB

You can now install OpenVDB using:

```bash
sudo make install
```

🔸 For more information about OpenVDB please refer to the [official documentation](https://www.openvdb.org/documentation/doxygen/).
